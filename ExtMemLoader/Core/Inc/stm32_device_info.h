#ifndef STM32_LOADER_INFO_H
#define STM32_LOADER_INFO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_extmemloader_conf.h"

/**
 * @brief different type of supported memory
 */
#define MCU_FLASH 1
#define NAND_FLASH 2
#define NOR_FLASH 3
#define SRAM 4
#define PSRAM 5
#define PC_CARD 6
#define SPI_FLASH 7
#define I2C_FLASH 8
#define SDRAM 9
#define I2C_EEPROM 10

/**
 * @brief Maximum Number of Sectors
 */
#define SECTOR_NUM    10

/**
 * @brief Maximum length for the device name
 */
#define DEVICENAME_MAXLENGHT 100

typedef struct
{
  uint32_t SectorNumber;  /*!< Number of Sectors    */
  uint32_t SectorSize;    /*!< Sector Size in Bytes */
} DeviceSectors;

typedef struct
{
   uint8_t  DeviceName[DEVICENAME_MAXLENGHT]; /*!< Device Name and Description              */
   uint16_t DeviceType;    	              /*!< Device Type: NAND_FLASH, NOR_FLASH, ...  */
   uint32_t DeviceStartAddress;	              /*!< Default Device Start Address             */
   uint32_t DeviceSize;    	              /*!< Total Size of Device                     */
   uint32_t PageSize;  	                      /*!< Programming Page Size                    */
   uint8_t  EraseValue;    	              /*!< Content of Erased Memory                 */
   DeviceSectors Sectors[SECTOR_NUM];         /*!< sector descriptor                        */
} sStorageInfo;

#ifdef __cplusplus
}
#endif

#endif /* STM32_LOADER_INFO_H */
