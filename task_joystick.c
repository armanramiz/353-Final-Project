/*
 * task_led.c
 *
 *  Created on: Oct 19, 2020
 *      Author: Joe Krachey
 */

#include <task_joystick.h>


 TaskHandle_t Task_Joystick_Handle;
 TaskHandle_t Task_Joystick_Timer_Handle;

 volatile uint32_t JOYSTICK_X_DIR = 0;
 volatile uint32_t JOYSTICK_Y_DIR = 0;

/******************************************************************************
* This function will run the same configuration code that you would have
* written for HW02.
******************************************************************************/
 void Task_Joystick_Init(void)
 {
     // Configure the X direction as an analog input pin.
     P6->SEL0 |= BIT0;
     P6->SEL1 |= BIT0;

     // Configure the Y direction as an analog input pin.
     P4->SEL0 |= BIT4;
     P4->SEL1 |= BIT4;

     // Configure CTL0 to sample 16-times in pulsed sample mode.
     // NEW -- Indicate that this is a sequence of samples.
     ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_1;

     // Configure CTL1 to return 12-bit values
     ADC14->CTL1 = ADC14_CTL1_RES_2;         // Use sampling timer, 12-bit conversion results

     // Associate the X direction analog signal with MEM[0]
     ADC14->MCTL[0] |= ADC14_MCTLN_INCH_15;

     // Associate the Y direction analog signal with MEM[1]
     // NEW -- Make sure to indicate this is the end of a sequence.
     ADC14->MCTL[1] |= ADC14_MCTLN_INCH_9 | ADC14_MCTLN_EOS;

     // Enable interrupts AFTER a value is written into MEM[1].
     ADC14->IER0 |= ADC14_IER0_IE1 ;

     // Enable ADC Interrupt
     NVIC_EnableIRQ(ADC14_IRQn);

     // Interrupt priorities must NOT be set to 0.  It is preferred if they are set to a value that is > than 1 too.
     NVIC_SetPriority(ADC14_IRQn,2);

     // Turn ADC ON
     ADC14->CTL0 |= ADC14_CTL0_ON;

 }


/******************************************************************************
* Bottom Half Task.  Examines the ADC data from the joy stick on the MKII
******************************************************************************/
void Task_Joystick_Bottom_Half(void *pvParameters)
{

    SHIP_CMD_t prev_cmd = SHIP_CMD_CENTER;
    uint32_t ulEventsToProcess;
    BaseType_t status;
    SHIP_MSG_t msg;

    SHIP_CMD_t cmd;

    while(1)
    {
        /* ADD CODE
         * Wait until we get a task notification from the ADC14 ISR
         */

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * Set the dir variable to one of the following values based
         * on the values of JOYSTICK_X_DIR and JOYSTIC_Y_DIR
         */
        if(JOYSTICK_X_DIR < VOLT_0P85)
        {
           cmd = SHIP_CMD_LEFT;
        }
        else if(JOYSTICK_X_DIR > VOLT_2P50)
        {
            cmd = SHIP_CMD_RIGHT;
        }
        else if(JOYSTICK_Y_DIR < VOLT_0P85)
        {
            cmd = SHIP_CMD_DOWN;
        }
        else if(JOYSTICK_Y_DIR > VOLT_2P50)
        {
            cmd = SHIP_CMD_UP;
        }
        else
        {
            cmd = SHIP_CMD_CENTER;
       }

        SHIP_MSG_t msg[] = {cmd, 1};

        /* ADD CODE
         * Send dir to Queue_Console if the the current direction
         * of the joystick does not match the previous direction of the joystick
         */

       //if(prev_cmd != cmd){
           status = xQueueSend(Queue_Space_Ship, &msg, portMAX_DELAY);

             //  }

        /* ADD CODE
         * Update the prev_dir of the joystick
         */
       prev_cmd = cmd;
    }

}

void task_joystick(void *pvParameters) {
    // Begin bottom half sub-task.
    xTaskCreate
    (   Task_Joystick_Bottom_Half,
        "Task_Joystick_Bottom_Half",
        configMINIMAL_STACK_SIZE,
        NULL,
        ACCEL_TASK_PRIORITY,
        &Task_Joystick_Handle
    );

    // Endless Task Loop.
    while (1) {
        // Start ADC conversion.
        ADC14->CTL0 |= ADC14_CTL0_SC | ADC14_CTL0_ENC;

        // Delay before starting next conversion.
        vTaskDelay(pdMS_TO_TICKS(ACCEL_PERIOD_DELAY));
    }
}





