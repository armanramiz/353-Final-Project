/*
 * task_menu.c
 *
 *  Created on: Apr 22, 2021
 *      Author: John Eslinger
 */

#include "task_menu.h"

// Handle for Menu Task.
TaskHandle_t Task_Menu_Handle = NULL;

/*
 * Function for Menu Task.
 * JohnEsl-TODO
 */
void task_menu(void *pvParameters) {
    // Create Button Task (temporarily with same priority).
    xTaskCreate
    (   task_button,
        "Task Button",
        configMINIMAL_STACK_SIZE,
        NULL,
        BUTTON_TASK_PRIORITY,
        &Task_Button_Handle
    );

    // Create Buzzer Task (temporarily with same priority).
    xTaskCreate
    (   task_buzzer,
        "Task Buzzer",
        configMINIMAL_STACK_SIZE,
        NULL,
        BUZZER_TASK_PRIORITY,
        &Task_Buzzer_Handle
    );

    // Create Accel Task (temporarily with same priority).
    xTaskCreate
    (   task_accel,
        "Task Accel",
        configMINIMAL_STACK_SIZE,
        NULL,
        ACCEL_TASK_PRIORITY,
        &Task_Accel_Handle
    );
    xTaskCreate
     (   Task_Space_Ship,
         "Task_Space_Ship",
         configMINIMAL_STACK_SIZE,
         NULL,
         3,
         &Task_Space_Ship_Handle
     );

    while(1) {};

   // static uint8_t x = 64;
   // static uint8_t y = 110;
   // lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);

    // Endless Task Loop.
   //while(1) {

      /* // SHIP_MSG_t msg;
       // BaseType_t status;
       // SHIP_CMD_t cmd;

        //status = xQueueReceive(Queue_Space_Ship, &msg, portMAX_DELAY);

        // setup blue LED
        P2->DIR |= BIT2;
        P2->OUT &= ~BIT2;

        switch(accelDir())
               {
                case  LEFT://MOVE LEFT
                       {
                           if(x > 25){
                             x= x-1;
                             lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);
                             break;
                             }
                           else{
                               break;
                           }
                       }
                   case RIGHT: //MOVE RIGHT
                       {
                           if(x < 105){
                               x= x+1;
                              lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);
                              break;
                           }
                           else{
                             break;
                        }
                       }
                   case CENTER:
                        {

                         break;
                        }

                   default:
                   {
                       break;
                   }
               }


    }*/
}


