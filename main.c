/*
 * main.c
 *
 *  Created on: Apr 22nd, 2021
 *      Author: John Eslinger
 */

/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"

/*
 * Main application code. Initializes hardware resources, creates main
 * task for application code, and starts RTOS. Should not ever return.
 */
int main(void)
{
    // Stop Watchog Timer.
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initialize hardware resources.
    initTaskButton();
    initTaskBuzzer();
    initTaskAccel();
   // Crystalfontz128x128_Init();
    //Task_Joystick_Init();
    Task_Space_Ship_Init();

    // Create main menu task.
    xTaskCreate
    (   task_menu,
        "Task Menu",
        configMINIMAL_STACK_SIZE,
        NULL,
        MENU_TASK_PRIORITY,
        &Task_Menu_Handle
    );


    // Start the FreeRTOS scheduler.
    vTaskStartScheduler();

    // Endless loop.
    while(1) {};
    return (0); // Should never be reached.
}

/*
 * Application defined malloc failed hook.
 */
void vApplicationMallocFailedHook()
{
    // Handle Memory Allocation Errors.
    while(1) {};
}

/*
 * Application defined stack overflow hook.
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    // Handle FreeRTOS Stack Overflow
    while(1) {};
}
