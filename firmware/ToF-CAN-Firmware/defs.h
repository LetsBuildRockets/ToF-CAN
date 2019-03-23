/*
 * defs.h
 *
 * Created: 3/22/2019 1:59:01 PM
 *  Author: erics
 */ 


#ifndef DEFS_H_
#define DEFS_H_

// Control Bytes
#define CTRL_SEND_ERROR 1
#define CTRL_SEND_DISTANCE 2
#define CTRL_SET_NEW_ADDR 3
#define CTRL_GET_FIRMWARE_VERSION 4
#define CTRL_SEND_FIRMWARE_VERSION 5

// Error codes
#define ERROR_NONE 0
#define ERROR_OUT_OF_RANGE 1
#define ERROR_WRITING_TO_CAN 2
#define ERROR_INIT_CAN 3
#define ERROR_INIT_VL53L0X 4
#define ERROR_BAD_CTRL_BYTE 5
#define ERROR_NOT_ENOUGH_DATA_BYTES 6

#endif /* DEFS_H_ */