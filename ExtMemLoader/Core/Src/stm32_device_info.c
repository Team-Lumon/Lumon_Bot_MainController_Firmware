#define STM32_EXTMEMLOADER_STM32CUBETARGET
#if defined(STM32_EXTMEMLOADER_STM32CUBETARGET)
#include "stm32_device_info.h"

#if defined(__ICCARM__)
__root sStorageInfo const StorageInfo =
#else
__attribute__((used)) sStorageInfo const StorageInfo =
#endif                                     /* __ICCARM__*/
{
    "W25Q64JV_Lumon",                      // Device Name
    NOR_FLASH,                             // Device Type
    0x90000000,                            // Device Start Address
    0x800000,                              // Device Size in Bytes
    0x100,                                 // Programming Page Size in Bytes
    0xFF,                                  // Initial Content of Erased Memory
    {
      {2048, 0x1000},                      // Specify Number of sectors and size of each Sector
      {0x00000000, 0x00000000}
    },
};

#endif /* STM32_EXTMEMLOADER_CUBEPROGRAMMERTARGET */
