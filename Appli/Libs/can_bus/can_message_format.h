#ifndef CAN_MESSAGE_FORMAT_H
#define CAN_MESSAGE_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_ID_PRIORITY_MASK      0x700U
#define CAN_ID_MESSAGE_TYPE_MASK  0x0F0U
#define CAN_ID_DESTINATION_MASK   0x00FU

#define CAN_ID_PRIORITY_SHIFT     8U
#define CAN_ID_MESSAGE_TYPE_SHIFT 4U
#define CAN_ID_DESTINATION_SHIFT  0U

#define CAN_DEST_BROADCAST_ID     0xFU

typedef enum
{
    CAN_Priority_VERY_HIGH = 0x0U,
    CAN_Priority_HIGH      = 0x1U,
    CAN_Priority_MEDIUM    = 0x2U,
    CAN_Priority_LOW       = 0x3U,
    CAN_Priority_VERY_LOW  = 0x4U
} CAN_Priority_t;

typedef enum
{
    CAN_ID_EMERGENCY   = 0x0U,
    CAN_ID_HEARTBEAT   = 0x1U,
    CAN_ID_STATUS      = 0x2U,
    CAN_ID_COMMAND     = 0x3U,
    CAN_ID_SYNC        = 0x4U,
    CAN_ID_ADC_REPORT  = 0x5U,
    CAN_ID_DEBUG       = 0x6U,
    CAN_MessageId_Invalid = 0xFU
} CAN_MessageType_t;

#define CAN_BUILD_ID(priority, message_type, destination_id) \
    ((((uint16_t)(priority)       & 0x7U) << CAN_ID_PRIORITY_SHIFT) | \
     (((uint16_t)(message_type)   & 0xFU) << CAN_ID_MESSAGE_TYPE_SHIFT) | \
     (((uint16_t)(destination_id) & 0xFU) << CAN_ID_DESTINATION_SHIFT))

#define CAN_GET_PRIORITY(can_id) \
    (((uint16_t)(can_id) & CAN_ID_PRIORITY_MASK) >> CAN_ID_PRIORITY_SHIFT)

#define CAN_GET_MESSAGE_TYPE(can_id) \
    (((uint16_t)(can_id) & CAN_ID_MESSAGE_TYPE_MASK) >> CAN_ID_MESSAGE_TYPE_SHIFT)

#define CAN_GET_DESTINATION(can_id) \
    (((uint16_t)(can_id) & CAN_ID_DESTINATION_MASK) >> CAN_ID_DESTINATION_SHIFT)

#define CAN_IS_BROADCAST(can_id) \
    (CAN_GET_DESTINATION(can_id) == CAN_DEST_BROADCAST_ID)

#endif // CAN_MESSAGE_FORMAT_H
