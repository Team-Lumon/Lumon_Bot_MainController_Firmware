#define STM32_EXTMEMLOADER_STM32CUBETARGET
#if defined(STM32_EXTMEMLOADER_STM32CUBETARGET)
/* Includes ------------------------------------------------------------------*/
#include "stm32_device_info.h"
#include "stm32_loader_api.h"
#include <string.h>
#include <stdio.h>

#if !defined(STM32_EXTMEMLOADER_TRACE)
#define STM32_EXTMEMLOADER_TRACE(_MSG_)
#endif /* STM32_EXTMEMLOADER_TRACE */

#if defined(__ICCARM__)
#pragma section = ".bss"
#elif defined(__GNUC__)
extern int __bss_start__, __bss_end__;
extern int __estack_end__;
#endif /* __ICCARM__ */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
 uint32_t __Vectors = 0x0;
#endif /* __CC_ARM || __ARMCC_VERSION */

typedef enum {
  DEBUG_STATE_WAIT,
  DEBUG_STATE_INIT,
  DEBUG_STATE_WRITE,
  DEBUG_STATE_SECTORERASE,
  DEBUG_STATE_MASSERASE
} DEBUG_STATETypedef;

volatile DEBUG_STATETypedef exec;
uint8_t *Buff_address;
uint32_t Address;
uint32_t Size;
volatile const int condition = 1;

MEM_MAPSTAT MemoryMappedMode;

static uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal);

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/*nothing to be done*/
#elif defined(__GNUC__)
uint32_t g_pfnVectors[] __attribute__((section(".isr_vector"))) = {
};
void __attribute__((used,optimize("Os"))) Reset_Handler(void)
{
  asm(
      "ldr r0, =_estack\n"
      "nop\n"
      "mov     sp, r0\n");

  asm(
      "ldr r0, =main\n"
      "nop\n"
      "mov     pc, r0\n");
}
#elif defined(__ICCARM__)
extern const uint32_t __ICFEDIT_region_RAM_end__;
void Reset_Handler(void);
uint32_t __vector_table[] __attribute__((section(".vectors"))) = {
    (uint32_t)(&__ICFEDIT_region_RAM_end__), /* Stack pointer */
    (uint32_t)&Reset_Handler                 /* Reset handler */
};
void Reset_Handler(void)
{
  main();
}
#endif /* STM32_EXTFLASHLOADER_DEBUG_NA */

int main(void)
{
  exec = DEBUG_STATE_INIT;
  do
  {
    switch (exec)
    {
    case DEBUG_STATE_WAIT:{
      break;
    }
    case DEBUG_STATE_INIT :{
      Init();
      exec = DEBUG_STATE_WAIT;
      break;
    }
    case DEBUG_STATE_WRITE : {
      Write (Address, Size, (uint8_t *)Buff_address);
      exec = DEBUG_STATE_INIT;
      break;
    }
    case DEBUG_STATE_MASSERASE :{
      MassErase(0);
      exec = DEBUG_STATE_INIT;
      break;
    }
    case DEBUG_STATE_SECTORERASE :{
      SectorErase(Address,Size);
      exec = DEBUG_STATE_INIT;
      break;
    }
    }
  } while (condition);

  return 0;
}

uint32_t Init(void)
{
  uint32_t retr = 1;
  uint8_t *startadd;
  uint32_t size;

#if defined(__ICCARM__)
  startadd = __section_begin(".bss");
  size = __section_size(".bss");
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$PrgData$$ZI$$Base;
  extern uint32_t Image$$PrgData$$ZI$$Limit;
  startadd = (uint8_t *)&Image$$PrgData$$ZI$$Base;
  size     = (uint32_t)&Image$$PrgData$$ZI$$Limit - (uint32_t)startadd;
#elif defined ( __GNUC__ )
  startadd = (uint8_t*)& __bss_start__;
  size     = (uint8_t*)& __bss_end__ - (uint8_t*)& __bss_start__;
#endif /* __ICCARM__ */

  memset(startadd, 0, size * sizeof(uint8_t));

  if (extmemloader_Init() != 0)
  {
    retr = 0;
  }

  if (memory_mapmode(MEM_MAPENABLE)  != MEM_OK)
  {
    retr = 0;
  }

  MemoryMappedMode = MEM_MAPENABLE;

  return retr;
}

KeepInCompilation uint32_t Write(uint32_t Address, uint32_t Size, uint8_t *buffer)
{
  if (MemoryMappedMode == MEM_MAPENABLE)
  {
    if (memory_mapmode(MEM_MAPDISABLE)  != MEM_OK)
    {
      return 0;
    }
    MemoryMappedMode = MEM_MAPDISABLE;
  }

  if (memory_write(Address, Size, buffer) != MEM_OK)
  {
    return 0;
  }

  return 1;
}

KeepInCompilation uint32_t MassErase(uint32_t Parallelism)
{
  if (MemoryMappedMode == MEM_MAPENABLE)
  {
    if (memory_mapmode(MEM_MAPDISABLE)  != MEM_OK)
    {
      return 0;
    }
    MemoryMappedMode = MEM_MAPDISABLE;
  }

  if (memory_masserase() != MEM_OK)
  {
    return 0;
  }

  return 1;
}

KeepInCompilation uint32_t SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress)
{
  if (MemoryMappedMode == MEM_MAPENABLE)
  {
    if (memory_mapmode(MEM_MAPDISABLE)  != MEM_OK)
    {
      return 0;
    }
    MemoryMappedMode = MEM_MAPDISABLE;
  }

  if (memory_sectorerase(EraseStartAddress, EraseEndAddress, STM32EXTLOADER_DEVICE_SECTOR_SIZE) != MEM_OK)
  {
    return 0;
  }

  return 1;
}

KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t Missalignement)
{
  uint32_t verified_data = 0;
  uint32_t memory_addr = MemoryAddr;
  uint32_t ram_addr = RAMBufferAddr;
  uint32_t size = Size*4;
  uint64_t checksum;

  if (MemoryMappedMode == MEM_MAPDISABLE)
  {
    if (memory_mapmode(MEM_MAPENABLE) != MEM_OK)
    {
      return 0;
    }
    MemoryMappedMode = MEM_MAPENABLE;
  }

  checksum = CheckSum(memory_addr + (Missalignement & 0xFU), size - ((Missalignement >> 16U) & 0xFU), 0);
  while (size>verified_data)
  {
    if ( *(uint8_t*)memory_addr++ != *((uint8_t*)ram_addr + verified_data))
      return ((checksum<<32U) + (memory_addr + verified_data));

    verified_data++;
  }

  return (checksum<<32U);
}

static uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal)
{
  uint8_t missalignement_address = StartAddress % 4U;
  uint8_t missalignement_size = Size;
  uint32_t cnt;
  uint32_t init_val = InitVal;
  uint32_t start_addr;
  uint32_t val;

  start_addr = StartAddress - StartAddress % 4U;
  Size += (Size % 4U == 0) ? 0U : 4U - (Size % 4U);

  for (cnt = 0; cnt < Size; cnt += 4)
  {
    val = *(uint32_t *)start_addr;
    if (missalignement_address)
    {
      switch (missalignement_address)
      {
      case 1:
        init_val += (uint8_t)(val >> 8U & 0xFFU);
        init_val += (uint8_t)(val >> 16U & 0xFFU);
        init_val += (uint8_t)(val >> 24U & 0xFFU);
        missalignement_address -= 1;
        break;
      case 2:
        init_val += (uint8_t)(val >> 16U & 0xFFU);
        init_val += (uint8_t)(val >> 24U & 0xFFU);
        missalignement_address -= 2;
        break;
      case 3:
        init_val += (uint8_t)(val >> 24U & 0xFFU);
        missalignement_address -= 3;
        break;
      }
    }
    else if ((Size - missalignement_size) % 4 && (Size - cnt) <= 4U)
    {
      switch (Size - missalignement_size)
      {
      case 1U:
        init_val += (uint8_t)val;
        init_val += (uint8_t)(val >> 8U & 0xFFU);
        init_val += (uint8_t)(val >> 16U & 0xFFU);
        missalignement_size -= 1;
        break;
      case 2U:
        init_val += (uint8_t)val;
        init_val += (uint8_t)(val >> 8U & 0xFFU);
        missalignement_size -= 2;
        break;
      case 3U:
        init_val += (uint8_t)val;
        missalignement_size -= 3U;
        break;
      }
    }
    else
    {
      init_val += (uint8_t)val;
      init_val += (uint8_t)(val >> 8U & 0xFFU);
      init_val += (uint8_t)(val >> 16U & 0xFFU);
      init_val += (uint8_t)(val >> 24U & 0xFFU);
    }
    start_addr += 4U;
  }

  return (init_val);
}

#endif /* STM32_EXTMEMLOADER_STM32CUBETARGET */
