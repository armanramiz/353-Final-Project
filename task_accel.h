/*
 * task_accel.h
 *
 *  Created on: Apr 23, 2021
 *      Author: John Eslinger
 */

#ifndef TASK_ACCEL_H_
#define TASK_ACCEL_H_

// Standard includes for ece353 firmware.
#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// RTOS header files.
#include <FreeRTOS.h>
#include <task.h>
#include <task_space_ship.h>

// Application header files.
#include "config.h"

// Enum to define the possible states of the accelerometer.
typedef enum AccelStates {
    LEFT,
    CENTER,
    RIGHT,
} dir_t;



/*
 * Initialization function used to initialize hardware resources to use the
 * accelerometer x-axis sensor on the MKII booster pack.
 */
void initTaskAccel(void);

/*
 * Getter function to determine the direction the MKII booster pack is
 * being tilted (with regards to the x-axis accelerometer facing the
 * negative y-axis direction).
 *
 * Returns direction MKII booster pack is being tilted
 */
dir_t accelDir(void);

// Function handle and declaration for Accelerometer (Starting) Task.
extern TaskHandle_t Task_Accel_Handle;
void task_accel(void *pvParameters);

#endif /* TASK_ACCEL_H_ */
