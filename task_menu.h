/*
 * task_menu.h
 *
 *  Created on: Apr 22, 2021
 *      Author: John Eslinger
 */

#ifndef TASK_MENU_H_
#define TASK_MENU_H_

// Standard includes for ece353 firmware.
#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// RTOS header files.
#include <FreeRTOS.h>
#include <task.h>

// Application header files.
#include "config.h"
#include "task_button.h"
#include "task_buzzer.h"
#include "task_accel.h"
#include "task_space_ship.h"
#include "task_joystick.h"

// Function handle and declaration for Menu Task.
extern TaskHandle_t Task_Menu_Handle;
void task_menu(void *pvParameters);

#endif /* TASK_MENU_H_ */
