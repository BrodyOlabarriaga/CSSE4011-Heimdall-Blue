/**
 * @file tpacket.h
 * @author Gian Barta-Dougall ()
 * @brief Comms protocol
 * @version 0.1
 * @date 2023-03-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef TPACKET_H
#define TPACKET_H

/* C Library Incldues */
#include <stdint.h>
#include <stddef.h>

/* Personal Includes */
#include "utils.h"

/* Public Macros */
#define TP_START_BYTE 0xAA

#define TP_MAX_NUM_DATA_BYTES   14
#define TP_BUFFER_MAX_NUM_BYTES 17

#define TP_READ_OFFSET           (2)
#define TP_READ_START_BYTE       (TP_READ_OFFSET + 0)
#define TP_READ_INSTRUCTION_BYTE (TP_READ_OFFSET + 1)
#define TP_READ_TYPE_BYTE        (TP_READ_OFFSET + 2)
#define TP_READ_DATA_BYTE        (TP_READ_OFFSET + 3)

enum tp_instruction_e {
    TP_INSTRUCTION_REQUEST,
    TP_INSTRUCTION_RESPONSE,
};

enum tp_type_e {
    TP_TYPE_GET_TEMPERATURE,
    TP_TYPE_GET_BAROMETER,
    TP_TYPE_GET_ACC_X,
    TP_TYPE_GET_ACC_Y,
    TP_TYPE_GET_ACC_Z,
    TP_TYPE_GET_GYR_X,
    TP_TYPE_GET_GYR_Y,
    TP_TYPE_GET_GYR_Z,
    TP_TYPE_GET_MAG_X,
    TP_TYPE_GET_MAG_Y,
    TP_TYPE_GET_MAG_Z,
    TP_TYPE_GET_ALTITUDE,
    TP_TYPE_GET_ULTRASONIC,
    TP_TYPE_GET_PITCH,
    TP_TYPE_GET_ROLL,
    TP_TYPE_SET_SAMPLE_RATE,
    TP_TYPE_SET_SAMPLE_ALL_START,
    TP_TYPE_SET_SAMPLE_ALL_STOP,
    TP_TYPE_GET_BUTTON_STATE,
    TP_TYPE_GET_RSSI,
    TP_TYPE_ADD_NODE,
    TP_TYPE_REM_NODE,
    TP_TYPE_PRINT_NODES,
    TP_TYPE_PRINT_NODE,
};

/* Public Structs */
typedef struct tp_instruction_t {
    uint8_t val;
} tp_instruction_t;

typedef struct tp_type_t {
    uint8_t val;
} tp_type_t;

typedef struct tp_data_t {
    uint8_t length;
    uint8_t bytes[TP_MAX_NUM_DATA_BYTES];
} tp_data_t;

typedef struct tp_packet_t {
    tp_instruction_t Instruction;
    tp_type_t Type;
    tp_data_t Data;
} tp_packet_t;

typedef struct tp_buffer_t {
    uint16_t length;
    uint8_t bytes[TP_BUFFER_MAX_NUM_BYTES];
} tp_buffer_t;

/* Public Structs Creation */
#define TP_INSTRUCTION_START 1
extern tp_instruction_t TP_Instruction_Request;
extern tp_instruction_t TP_Instruction_Response;

#define TP_TYPE_START     3
#define TP_TYPE_MAX_VALUE (TP_TYPE_START + 29)

extern tp_type_t TP_Type_Get_Temperature;
extern tp_type_t TP_Type_Get_Barometer;
extern tp_type_t TP_Type_Get_Acc_X;
extern tp_type_t TP_Type_Get_Acc_Y;
extern tp_type_t TP_Type_Get_Acc_Z;
extern tp_type_t TP_Type_Get_Gyr_X;
extern tp_type_t TP_Type_Get_Gyr_Y;
extern tp_type_t TP_Type_Get_Gyr_Z;
extern tp_type_t TP_Type_Get_Mag_X;
extern tp_type_t TP_Type_Get_Mag_Y;
extern tp_type_t TP_Type_Get_Mag_Z;
extern tp_type_t TP_Type_Get_Altitude;
extern tp_type_t TP_Type_Get_Ultrasonic;
extern tp_type_t TP_Type_Get_Pitch;
extern tp_type_t TP_Type_Get_Roll;
extern tp_type_t TP_Type_Set_Sample_Rate;
extern tp_type_t TP_Type_Set_Sample_All_Start;
extern tp_type_t TP_Type_Set_Sample_All_Stop;
extern tp_type_t TP_Type_Get_Button_State;
extern tp_type_t TP_Type_Get_RSSI;
extern tp_type_t TP_Type_Add_Node;
extern tp_type_t TP_Type_Rem_Node;
extern tp_type_t TP_Type_Print_Nodes;
extern tp_type_t TP_Type_Print_Node;

extern tp_data_t TP_DATA_NONE;

/* Assertion Statements */

#define TP_ASSERT_VALID_DATA_LENGTH(Data)             \
    do {                                              \
        if ((Data)->length > TP_MAX_NUM_DATA_BYTES) { \
            return FALSE;                             \
        }                                             \
    } while (0)

#define TP_ASSERT_VALID_BUFFER(Buffer)                    \
    do {                                                  \
        if ((Buffer)->length > TP_BUFFER_MAX_NUM_BYTES) { \
            return FALSE;                                 \
        }                                                 \
    } while (0)

#define TP_ASSERT_VALID_PACKET(Packet)                       \
    do {                                                     \
        TP_ASSERT_VALID_INSTRUCTION(&(Packet->Instruction)); \
        TP_ASSERT_VALID_TYPE(&(Packet->Type));               \
        TP_ASSERT_VALID_DATA_LENGTH(&(Packet->Data));        \
    } while (0)

/**
 * @brief Creates a tpacket
 *
 * @param Packet packet
 * @param Instruction instruction
 * @param Type type
 * @param Data data
 * @return uint8_t true if creation was succesful else false
 */
uint8_t tp_create_packet(tp_packet_t* Packet, tp_instruction_t* Instruction, tp_type_t* Type, tp_data_t* Data);

/**
 * @brief Converts packet to buffer
 *
 * @param Packet packet
 * @param Pbuffer buffer
 * @return uint8_t success code
 */
uint8_t tp_packet_to_buffer(tp_packet_t* Packet, tp_buffer_t* Pbuffer);

/**
 * @brief Converts uint8_t array to a buffer
 *
 * @param Packet Packet
 * @param bytes uint8_t array of bytes
 * @return uint8_t success code
 */
uint8_t tp_bytes_to_packet(tp_packet_t* Packet, uint8_t* bytes);

/**
 * @brief Converts buffer to packet
 *
 * @param Pbuffer buffer
 * @param Packet packet
 * @return uint8_t success code
 */
uint8_t tp_buffer_to_packet(tp_buffer_t* Pbuffer, tp_packet_t* Packet);

/**
 * @brief Sets all values of packet to 0
 *
 * @param Packet packet to reset
 */
void tp_reset_packet(tp_packet_t* Packet);

/**
 * @brief Sets all values of data struct to 0 including the length
 *
 * @param Packet packet to reset
 */
void tp_reset_data(tp_data_t* Data);

/**
 * @brief Sets the instruction of the packet
 *
 * @param Packet TRUE if the instruction was valid else FALSE
 */
uint8_t tp_set_instruction(tp_packet_t* Packet, uint8_t instruction);

/**
 * @brief Sets the type of the packet
 *
 * @param Packet TRUE if the type was valid else FALSE
 */
uint8_t tp_set_type(tp_packet_t* Packet, uint8_t type);

#endif // TPACKET_H