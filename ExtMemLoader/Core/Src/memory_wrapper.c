#include "extmemloader_init.h"
#include "stm32_extmemloader_conf.h"
#include "memory_wrapper.h"

__weak MEM_STATUS memory_init(void)
{
  return MEM_OK;
}

__weak MEM_STATUS memory_write(uint32_t Address, uint32_t Size, uint8_t* buffer)
{
  MEM_STATUS retr = MEM_OK;
  uint32_t addr = Address & 0x0FFFFFFFUL;

  if (EXTMEM_Write(STM32EXTLOADER_DEVICE_MEMORY_ID, addr, buffer, Size) != EXTMEM_OK)
  {
    retr = MEM_FAIL;
  }

  return retr;
}

__weak MEM_STATUS memory_masserase(void)
{
  MEM_STATUS retr = MEM_OK;
  if (EXTMEM_EraseAll(STM32EXTLOADER_DEVICE_MEMORY_ID) != EXTMEM_OK)
  {
    retr = MEM_FAIL;
  }

  return retr;
}

__weak MEM_STATUS memory_sectorerase(uint32_t EraseStartAddress, uint32_t EraseEndAddress, uint32_t SectorSize)
{
  uint32_t start_addr = EraseStartAddress & 0x0FFFFFFFUL;
  uint32_t end_addr = (EraseEndAddress & 0x0FFFFFFFUL)+SectorSize;
  MEM_STATUS retr = MEM_OK;

  do
  {
    if (EXTMEM_EraseSector(STM32EXTLOADER_DEVICE_MEMORY_ID, start_addr, SectorSize) != EXTMEM_OK)
    {
      retr = MEM_FAIL;
    }
    start_addr = start_addr + SectorSize;
  } while ((end_addr > start_addr) && (retr == MEM_OK));

  return retr;
}

__weak MEM_STATUS memory_mapmode(MEM_MAPSTAT State)
{
  MEM_STATUS retr = MEM_OK;
  if (EXTMEM_MemoryMappedMode(STM32EXTLOADER_DEVICE_MEMORY_ID, State == MEM_MAPENABLE ? EXTMEM_ENABLE: EXTMEM_DISABLE) != EXTMEM_OK)
  {
    retr = MEM_FAIL;
  }
  return retr;
}
