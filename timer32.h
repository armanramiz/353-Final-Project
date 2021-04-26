/*
 * interrupts.h
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe Krachey
 */

#ifndef TIMER32_H_
#define TIMER32_H_

#include <main.h>


void ece353_T32_2_Interrupt_Ms(uint16_t ms);
extern bool volatile LCD_UPDATE;
/*****************************************************************************
 * Timer32_2 ISR (Should be generated every 100ms in our hw3)
 *
Set a global variable that will be used to indicate that the LCD screen can be updated.
*****************************************************************************/
void T32_INT2_IRQHandler();
#endif /* TIMER32_H_ */
