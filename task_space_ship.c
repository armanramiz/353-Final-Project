/*
 * task_space_ship.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Joe Krachey
 */

#include <task_space_ship.h>

#define SHIP_QUEUE_LEN  2

TaskHandle_t Task_Space_Ship_Handle;
QueueHandle_t Queue_Space_Ship;

/******************************************************************************
 * This function will initialize Queue_Space_Ship and initialize the LCD
 ******************************************************************************/
void Task_Space_Ship_Init(void)
{
   // Queue_Space_Ship = xQueueCreate(SHIP_QUEUE_LEN,sizeof(SHIP_MSG_t));//Current location of space ship on LCD
    //msg indicates which direction to move the spaceship

    // ADD CODE to initialize the LCD
    Crystalfontz128x128_Init();

}


/******************************************************************************
 * This task manages the movement of the space ship. The joystick task or the
 * console task can send messages to SHIP_QUEUE_LEN
 ******************************************************************************/
void Task_Space_Ship(void *pvParameters)
{
    uint8_t x = 64;
     uint8_t y = 110;
    lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);

    // Endless Task Loop.
    while(1) {

       // SHIP_MSG_t msg;
       // BaseType_t status;
       // SHIP_CMD_t cmd;
        //lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);
        //status = xQueueReceive(Queue_Space_Ship, &msg, portMAX_DELAY);

        // setup blue LED
       // P2->DIR |= BIT2;
       // P2->OUT &= ~BIT2;

        ece353_T32_2_Interrupt_Ms(10);

        P2->DIR |= BIT2;
                P2->OUT &= ~BIT2;
         int height=90;

       if(LCD_UPDATE){
        if(isButtonPressed()){
         while(height > 10){

                  height = height -5;
                  lcd_draw_image(x, height, space_lazerWidth, space_lazerHeight, Bitmaps_Lazer, BLUE_5, BG_COLOR);
                LCD_UPDATE= false;

            }
           // lcd_draw_image(x, height, space_lazerWidth, space_lazerHeight, Bitmaps_Lazer, BLUE_5, BG_COLOR);
           // lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);

           }
            //lcd_draw_image(x, (i), space_lazerWidth, space_lazerHeight, Bitmaps_Lazer, BG_COLOR, BG_COLOR);
        }

        switch(accelDir())
               {
                case  LEFT://MOVE LEFT
                       {
                          // lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, BG_COLOR, BG_COLOR);
                           if(x > 25){
                           // lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, BG_COLOR, BG_COLOR);

                             x= x-1;
                            // lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, BG_COLOR, BG_COLOR);
                             lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);
                             break;
                             }
                           else{
                               break;
                           }
                       }
                   case RIGHT: //MOVE RIGHT
                       {
                           if(x < 105){
                               x=x+1;
                              lcd_draw_image(x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);

                             // lcd_draw_image( x, y,space_shipWidthPixels, space_shipHeightPixels, space_shipBitmaps, FG_COLOR, BG_COLOR);
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


    }
}


