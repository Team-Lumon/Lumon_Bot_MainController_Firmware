#ifndef STM32_LOADER_API_H
#define STM32_LOADER_API_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_extmemloader_conf.h"
#include "memory_wrapper.h"

KeepInCompilation uint32_t Init (void);
KeepInCompilation uint32_t Write (uint32_t Address, uint32_t Size, uint8_t* buffer);
KeepInCompilation uint32_t SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress);
KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement);
KeepInCompilation uint32_t MassErase (uint32_t Parallelism );

#ifdef __cplusplus
}
#endif

#endif /* STM32_LOADER_API_H */
