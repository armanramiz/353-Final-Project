/*
 * task_accel.c
 *
 *  Created on: Apr 23, 2021
 *      Author: John Eslinger
 */

#include "task_accel.h"

// Handle for Accelerometer Task.
TaskHandle_t Task_Accel_Handle = NULL;

// Handle for the Accelerometer Task's bottom half sub-task.
TaskHandle_t Task_Accel_Bottom_Half_Handle = NULL;

// Overall state of the accelerometer on the MKII booster pack.
static volatile dir_t ACCEL_DIR = CENTER;

// Allow task_accel to see task_accel_botom_hald.
void task_accel_bottom_half(void *pvParameters);

/*
 * Initialization function used to initialize hardware resources to use the
 * accelerometer x-axis sensor on the MKII booster pack.
 */
void initTaskAccel(void) {
    // Configure pin to x-axis accelerometer as analog input.
    P6->DIR &= ~BIT1;
    P6->SEL0 |= BIT1; // select = 2'b11
    P6->SEL1 |= BIT1;

    // Configure CTL0 to sample 16-times in pulsed sample mode.
    ADC14->CTL0 = ADC14_CTL0_SHP |     // timer source
                  ADC14_CTL0_SHT0_2;   // 16 samples

    // Configure ADC to return 12-bit values
    ADC14->CTL1 = ADC14_CTL1_RES_2;

    // Associate the x-axis accelerometer signal with MEM[0]
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_14;

    // Enable interrupts AFTER a value is written into MEM[0].
    ADC14->IER0 |= ADC14_IER0_IE0;

    // Enable ADC Interrupt with priority 2.
    NVIC_EnableIRQ(ADC14_IRQn);
    NVIC_SetPriority(ADC14_IRQn, 2);

    // Turn ADC on.
    ADC14->CTL0 |= ADC14_CTL0_ON;
}

/*
 * Getter function to determine the direction the MKII booster pack is
 * being tilted (with regards to the x-axis accelerometer facing the
 * negative y-axis direction).
 *
 * Returns direction MKII booster pack is being tilted
 */
dir_t accelDir(void) {
    return ACCEL_DIR;
}

/*
 * Function for Accelerometer Task.
 * JohnEsl-TODO
 */
void task_accel(void *pvParameters) {
    // Begin bottom half sub-task.
    xTaskCreate
    (   task_accel_bottom_half,
        "Task Accel Bottom Half",
        configMINIMAL_STACK_SIZE,
        NULL,
        ACCEL_TASK_PRIORITY,
        &Task_Accel_Bottom_Half_Handle
    );

    // Endless Task Loop.
    while (1) {
        // Start ADC conversion.
        ADC14->CTL0 |= ADC14_CTL0_SC | ADC14_CTL0_ENC;

        // Delay before starting next conversion.
        vTaskDelay(pdMS_TO_TICKS(ACCEL_PERIOD_DELAY));
    }
}

/*
 * Function for Accelerometer Task bottom half sub-task.
 * JohnEsl-TODO
 */
void task_accel_bottom_half(void *pvParameters) {
    // Var to hold the current x-axis value.
    uint32_t xaxisValue;

    BaseType_t status;
    //SHIP_MSG_t msg;
    //SHIP_CMD_t cmd;

    // Endless Task Loop.
    while (1) {
        // Get/Wait for task notification with x-axis value.
        if (xTaskNotifyWait(0, 0, &xaxisValue, portMAX_DELAY) != pdPASS) {
            continue;
        }

        // Update the recorded state of the accelerometer.
        ACCEL_DIR = CENTER;
        if (xaxisValue < (ACCEL_CENTER_SAMPLE - ACCEL_CENTER_THRESH)) {
            //cmd = LEFT;
            ACCEL_DIR = LEFT;
        }
        else if (xaxisValue > (ACCEL_CENTER_SAMPLE + ACCEL_CENTER_THRESH)) {
           // cmd = RIGHT;
            ACCEL_DIR = RIGHT;
        }

        //status = xQueueSend(Queue_Space_Ship, &msg, portMAX_DELAY);

    }
}

/*
 * Interrupt Handler for ADC14. Sends collected data from the accelerometer
 * to the Accelerometer Task's bottom half task.
 */
void ADC14_IRQHandler(void) {
    // Get the converted value of the x-axis accelerometer (clears interrupt).
    uint32_t xaxisValue = ADC14->MEM[0];

    // Send data to bottom half task.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(Task_Accel_Bottom_Half_Handle,
                       xaxisValue,
                       eSetValueWithOverwrite,
                       &xHigherPriorityTaskWoken);
}
