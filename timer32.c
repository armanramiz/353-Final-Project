/*
 * timer32.c
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe Krachey
 */
#include <main.h>

/*****************************************************
 *Configures Timer32_2 to generate a periodic interrupts.
 *This will be used to generate interrupts every 100ms
 * Parameters:
 *      ticks : number of milliseconds per interrupt
 * Returns
 *      None
 *****************************************************/
void ece353_T32_2_Interrupt_Ms(uint16_t ms){

    uint32_t tick = ((SystemCoreClock * ms) / 1000) -1;

    //stop the timer
    TIMER32_2 ->CONTROL = 0;
    //set the load register with ticks value
    TIMER32_2->LOAD = tick;
    //Enable the timer32 interrupt in NVIC : Enable the NVIC to allow interrupts from timer32 peripheral
    __enable_irq(); //turns on NVIC
    NVIC_EnableIRQ(T32_INT2_IRQn);// turns NVIC for that particular interrupt;
    NVIC_SetPriority(T32_INT2_IRQn, 2); // to give timer32_2 the highest priority (for its interrupt)

    //Start timer32 and enable interrupt
    TIMER32_2 -> CONTROL = TIMER32_CONTROL_ENABLE | TIMER32_CONTROL_MODE | TIMER32_CONTROL_SIZE | TIMER32_CONTROL_IE;
                        // TURN TIMER ON,           PERIODIC MODE            32 BIT TIMER         ENABLE INTERRUPTS
}


volatile bool LCD_UPDATE;// global variable to indicate that LCD can be updated
/*****************************************************************************
 * Timer32_2 ISR (Should be generated every 100ms in our hw3)
 *
Set a global variable that will be used to indicate that the LCD screen can be updated.
*****************************************************************************/

void T32_INT2_IRQHandler(void)
{
    //Set a global variable that will be used to indicate that the LCD screen can be updated.
    LCD_UPDATE = true;
    // Clear the timer interrupt
    TIMER32_2->INTCLR = BIT0; // Write any value into this timer to clear the interrupt
}




