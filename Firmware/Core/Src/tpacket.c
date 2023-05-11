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
/* Personal Includes */
#include "tpacket.h"

/* Private Macros */

/* Private Variable Declarations */
tp_type_t TP_Type_Get_Temperature  = {.val = TP_TYPE_GET_TEMPERATURE};
tp_type_t TP_Type_Get_Barometer    = {.val = TP_TYPE_GET_BAROMETER};
tp_type_t TP_Type_Get_Acc_X        = {.val = TP_TYPE_GET_ACC_X};
tp_type_t TP_Type_Get_Acc_Y        = {.val = TP_TYPE_GET_ACC_Y};
tp_type_t TP_Type_Get_Acc_Z        = {.val = TP_TYPE_GET_ACC_Z};
tp_type_t TP_Type_Get_Gyr_X        = {.val = TP_TYPE_GET_GYR_X};
tp_type_t TP_Type_Get_Gyr_Y        = {.val = TP_TYPE_GET_GYR_Y};
tp_type_t TP_Type_Get_Gyr_Z        = {.val = TP_TYPE_GET_GYR_Z};
tp_type_t TP_Type_Get_Mag_X        = {.val = TP_TYPE_GET_MAG_X};
tp_type_t TP_Type_Get_Mag_Y        = {.val = TP_TYPE_GET_MAG_Y};
tp_type_t TP_Type_Get_Mag_Z        = {.val = TP_TYPE_GET_MAG_Z};
tp_type_t TP_Type_Get_Altitude     = {.val = TP_TYPE_GET_ALTITUDE};
tp_type_t TP_Type_Get_Ultrasonic   = {.val = TP_TYPE_GET_ULTRASONIC};
tp_type_t TP_Type_Get_Pitch        = {.val = TP_TYPE_GET_PITCH};
tp_type_t TP_Type_Get_Roll         = {.val = TP_TYPE_GET_ROLL};
tp_type_t TP_Type_Get_RSSI         = {.val = TP_TYPE_GET_RSSI};
tp_type_t TP_Type_Add_Node         = {.val = TP_TYPE_ADD_NODE};
tp_type_t TP_Type_Rem_Node         = {.val = TP_TYPE_REM_NODE};
tp_type_t TP_Type_Print_Nodes      = {.val = TP_TYPE_PRINT_NODES};
tp_type_t TP_Type_Print_Node       = {.val = TP_TYPE_PRINT_NODE};

tp_type_t TP_Type_Set_Sample_Rate      = {.val = TP_TYPE_SET_SAMPLE_RATE};
tp_type_t TP_Type_Set_Sample_All_Start = {.val = TP_TYPE_SET_SAMPLE_ALL_START};
tp_type_t TP_Type_Set_Sample_All_Stop  = {.val = TP_TYPE_SET_SAMPLE_ALL_STOP};
tp_type_t TP_Type_Get_Button_State     = {.val = TP_TYPE_GET_BUTTON_STATE};

tp_data_t TP_DATA_NONE = {.length = 0, .bytes = {0}};

tp_instruction_t TP_Instruction_Response = {.val = TP_INSTRUCTION_RESPONSE};
tp_instruction_t TP_Instruction_Request = {.val = TP_INSTRUCTION_REQUEST};

/* Function Prototyes */
void tp_fill_packet_data(tp_packet_t* Packet, uint8_t length, uint8_t* data);
void tp_reset_buffer(tp_buffer_t* Buffer);

uint8_t tp_create_packet(tp_packet_t* Packet, tp_instruction_t* Instruction, tp_type_t* Type, tp_data_t* Data) {

    tp_reset_packet(Packet);

    if (tp_set_instruction(Packet, Instruction->val) != TRUE) {
        return FALSE;
    }

    if (tp_set_type(Packet, Type->val) != TRUE) {
        return FALSE;
    }

    TP_ASSERT_VALID_DATA_LENGTH(Data);
    tp_fill_packet_data(Packet, Data->length, Data->bytes);

    return TRUE;
}

uint8_t tp_packet_to_buffer(tp_packet_t* Packet, tp_buffer_t* Buffer) {

    tp_reset_buffer(Buffer);

    Buffer->bytes[0] = TP_START_BYTE;
    Buffer->bytes[1] = (Packet->Instruction.val << 4) | (Packet->Data.length);
    Buffer->bytes[2] = Packet->Type.val;

    for (int i = 0; i < Packet->Data.length; i++) {
        Buffer->bytes[i + 3] = Packet->Data.bytes[i];
    }

    // Add 3 for start byte, instruction/length byte and type byte
    Buffer->length = Packet->Data.length + 3;

    return TRUE;
}

uint8_t tp_buffer_to_packet(tp_buffer_t* Buffer, tp_packet_t* Packet) {

    tp_reset_packet(Packet);
    TP_ASSERT_VALID_BUFFER(Buffer);

    Packet->Instruction.val = (Buffer->bytes[1] >> 4);
    Packet->Type.val        = Buffer->bytes[2];
    Packet->Data.length     = (Buffer->bytes[0] & 0x0F);
    for (int i = 0; i < Packet->Data.length; i++) {
        Packet->Data.bytes[i] = Buffer->bytes[i + 3];
    }

    return TRUE;
}

/**
 * @brief Converts a byte array to a tpacket
 * 
 * @param Packet packet
 * @param bytes byte array
 * @return uint8_t success code
 */
uint8_t tp_bytes_to_packet(tp_packet_t* Packet, uint8_t* bytes) {

    /* Assert bytes follow the structure of a tp buffer */
    if ((bytes == NULL) || (bytes[0] != TP_START_BYTE)) {
        return FALSE;
    }

    if (tp_set_instruction(Packet, bytes[1] >> 4) != TRUE) {
        return FALSE;
    }

    if (tp_set_type(Packet, bytes[2]) != TRUE) {
        return FALSE;
    }

    tp_reset_data(&Packet->Data);
    Packet->Data.length = (bytes[1] & 0x0F);
    TP_ASSERT_VALID_DATA_LENGTH(&Packet->Data);

    for (int i = 0; i < Packet->Data.length; i++) {
        Packet->Data.bytes[i] = bytes[i + 3];
    }

    return TRUE;
}

uint8_t tp_set_instruction(tp_packet_t* Packet, uint8_t instruction) {

    switch (instruction) {
        
        case TP_INSTRUCTION_REQUEST:
        case TP_INSTRUCTION_RESPONSE:
            Packet->Instruction.val = instruction;
            return TRUE;

        default:
            return FALSE;
    }

}

uint8_t tp_set_type(tp_packet_t* Packet, uint8_t type) {

    switch (type) {
        case TP_TYPE_GET_TEMPERATURE:
        case TP_TYPE_GET_BAROMETER:
        case TP_TYPE_GET_ACC_X:
        case TP_TYPE_GET_ACC_Y:
        case TP_TYPE_GET_ACC_Z:
        case TP_TYPE_GET_GYR_X:
        case TP_TYPE_GET_GYR_Y:
        case TP_TYPE_GET_GYR_Z:
        case TP_TYPE_GET_MAG_X:
        case TP_TYPE_GET_MAG_Y:
        case TP_TYPE_GET_MAG_Z:
        case TP_TYPE_GET_ALTITUDE:
        case TP_TYPE_GET_ULTRASONIC:
        case TP_TYPE_GET_PITCH:
        case TP_TYPE_GET_ROLL:
        case TP_TYPE_SET_SAMPLE_RATE:
        case TP_TYPE_SET_SAMPLE_ALL_START:
        case TP_TYPE_SET_SAMPLE_ALL_STOP:
        case TP_TYPE_GET_BUTTON_STATE:
        case TP_TYPE_GET_RSSI:
        case TP_TYPE_REM_NODE:
        case TP_TYPE_ADD_NODE:
        case TP_TYPE_PRINT_NODES:
            Packet->Type.val = type;
            return TRUE;
        
        default:
            return FALSE;
    }

}


/**
 * @brief Files packet with data
 *
 * @param Packet packet
 * @param length num bytes
 * @param data data
 */
void tp_fill_packet_data(tp_packet_t* Packet, uint8_t length, uint8_t* data) {
    Packet->Data.length = length;
    for (int i = 0; i < Packet->Data.length; i++) {
        Packet->Data.bytes[i] = data[i];
    }
}

void tp_reset_packet(tp_packet_t* Packet) {
    Packet->Instruction.val = 0;
    Packet->Type.val        = 0;
    Packet->Data.length     = 0;
    for (int i = 0; i < TP_MAX_NUM_DATA_BYTES; i++) {
        Packet->Data.bytes[i] = 0;
    }
}

/**
 * @brief Resets a buffer to all 0
 *
 * @param Buffer buffer to reset
 */
void tp_reset_buffer(tp_buffer_t* Buffer) {
    Buffer->length = 0;
    for (int i = 0; i < TP_BUFFER_MAX_NUM_BYTES; i++) {
        Buffer->bytes[i] = 0;
    }
}

/**
 * @brief Resets a buffer to all 0
 *
 * @param Buffer buffer to reset
 */
void tp_reset_data(tp_data_t* Data) {
    Data->length = 0;
    for (int i = 0; i < TP_MAX_NUM_DATA_BYTES; i++) {
        Data->bytes[i] = 0;
    }
}