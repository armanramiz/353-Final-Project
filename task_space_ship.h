/*
 * task_space_ship.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Joe Krachey
 */

#ifndef TASK_SPACE_SHIP_H_
#define TASK_SPACE_SHIP_H_

#include <main.h>

// Standard includes for ece353 firmware.
#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// RTOS header files.
#include <FreeRTOS.h>
#include <task.h>

// Application header files
#include "config.h"

#include <space_ship.h>
#include <task_button.h>
#include <timer32.h>
#include <lcd.h>


typedef enum {
    SHIP_CMD_LEFT,
    SHIP_CMD_RIGHT,
    SHIP_CMD_UP,
    SHIP_CMD_DOWN,
    SHIP_CMD_CENTER,
    SHIP_CMD_SPEED
} SHIP_CMD_t;



typedef struct {
    SHIP_CMD_t cmd;
    uint8_t     value;
}SHIP_MSG_t;


extern TaskHandle_t Task_Space_Ship_Handle;
extern QueueHandle_t Queue_Space_Ship;



/******************************************************************************
 * This function will initialize Queue_Space_Ship
 ******************************************************************************/
void Task_Space_Ship_Init(void);



/******************************************************************************
 * This task manages the movement of the space ship. The joystick task or the
 * console task can send messages to the
 ******************************************************************************/
void Task_Space_Ship(void *pvParameters);



#endif /* TASK_SPACE_SHIP_H_ */
