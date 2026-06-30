#ifndef MEMORY_WRAPPER_H_
#define MEMORY_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Public include ------------------------------------------------------------*/
#include "stm32_extmemloader_conf.h"

#if defined(__ICCARM__)
#define KeepInCompilation __root
#else
#define KeepInCompilation __attribute__((used))
#endif /* __ICCARM__ */

typedef enum {
  MEM_MAPDISABLE = 0, /*!< map mode disabled      */
  MEM_MAPENABLE       /*!< map mode enabled       */
} MEM_MAPSTAT;

typedef enum {
  MEM_OK,         /*!< mem function return status OK      */
  MEM_FAIL        /*!< mem function return status FAIL    */
} MEM_STATUS;

MEM_STATUS memory_init(void);
MEM_STATUS memory_write(uint32_t Address, uint32_t Size, uint8_t* buffer);
MEM_STATUS memory_masserase(void);
MEM_STATUS memory_sectorerase(uint32_t EraseStartAddress, uint32_t EraseEndAddress, uint32_t SectorSize);
MEM_STATUS memory_mapmode(MEM_MAPSTAT State);

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_WRAPPER_H_ */
