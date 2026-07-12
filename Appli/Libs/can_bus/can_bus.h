#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "main.h"
#include <stdint.h>

#define CAN_BROADCAST_ID 0x00FU

typedef enum {
  CAN_ID_EMERGENCY = 0x0U,
  CAN_ID_HEARTBEAT = 0x1U,
  CAN_ID_STATUS = 0x2U,
  CAN_ID_COMMAND = 0x3U,
  CAN_ID_SYNC = 0x4U,
  CAN_ID_ADC_REPORT = 0x5U,
  CAN_ID_DEBUG = 0x6U,
  CAN_MessageId_Invalid = 0xFU
} CAN_BusMessageId_t;

typedef enum {
  CAN_Priority_VERY_HIGH = 0x0U,
  CAN_Priority_HIGH = 0x1U,
  CAN_Priority_MEDIUM = 0x2U,
  CAN_Priority_LOW = 0x3U,
  CAN_Priority_VERY_LOW = 0x4U
} CAN_BusPriority_t;

typedef struct {
    uint8_t deviceId;
    CAN_BusMessageId_t messageId;
    CAN_BusPriority_t priority;
    uint8_t dlc;
    uint8_t data[8];
} CAN_BusMessage_t;


HAL_StatusTypeDef CAN_Bus_Init(FDCAN_HandleTypeDef *hfdcan, uint8_t  deviceId);
HAL_StatusTypeDef CAN_Bus_SendU8(FDCAN_HandleTypeDef *hfdcan,uint8_t deviceId, CAN_BusMessageId_t messageId, CAN_BusPriority_t priority, uint8_t value);
HAL_StatusTypeDef CAN_Bus_SendU16(FDCAN_HandleTypeDef *hfdcan, uint8_t deviceId, CAN_BusMessageId_t messageId, CAN_BusPriority_t priority, uint16_t value);
HAL_StatusTypeDef CAN_Bus_SendU32(FDCAN_HandleTypeDef *hfdcan, uint8_t deviceId, CAN_BusMessageId_t messageId, CAN_BusPriority_t priority, uint32_t value);
HAL_StatusTypeDef CAN_Bus_Receive(FDCAN_HandleTypeDef *hfdcan, CAN_BusMessage_t *message);

uint8_t CAN_Bus_ReadU8(const CAN_BusMessage_t *message);
uint16_t CAN_Bus_ReadU16(const CAN_BusMessage_t *message);
uint32_t CAN_Bus_ReadU32(const CAN_BusMessage_t *message);

CAN_BusPriority_t CAN_Bus_GetPriority(const CAN_BusMessage_t *message);
CAN_BusMessageId_t CAN_Bus_GetMessageId(const CAN_BusMessage_t *message);

uint16_t CAN_Bus_makeID(uint8_t deviceId, CAN_BusMessageId_t messageId, CAN_BusPriority_t priority);

#endif
