/*
 * config.h
 *
 *  Created on: Apr 22, 2021
 *      Author: John Eslinger
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Menu Task Configs
#define MENU_DEALLOC_DELAY      50 // Time (ms) for deallocating deleted tasks
#define MENU_TASK_PRIORITY      3  // Priority of Menu Task

// Button Task Configs
#define BUTTON_DEBOUNCE_VALUE   10 // # of consecutive pressed samples for "pressed" state
#define BUTTON_PERIOD_DELAY     5  // Time (ms) between button sampling
#define BUTTON_TASK_PRIORITY    3  // Priority of Button Task

// Buzzer Task Configs
#define BUZZER_BPM              80 // Beats per minute to play melodies at
#define BUZZER_QUEUE_SIZE       10 // Size of queue for pending melodies
#define BUZZER_TASK_PRIORITY    3  // Priority of Buzzer Task

// Accelerometer Task Configs
#define ACCEL_PERIOD_DELAY      5        // Time (ms) between ADC conversion starts
#define ACCEL_CENTER_SAMPLE     2060     // Average x-axis value when held flat (experimentally found)
#define ACCEL_CENTER_THRESH    (830 / 3) // Radius of "center" region (830 being experimentally found limit)
#define ACCEL_TASK_PRIORITY     3        // Priority of Accelerometer Task


// Buzzer Task Configs
//#define BUZZER_BPM              80 // Beats per minute to play melodies at
#define LCD_QUEUE_SIZE       10 // Size of queue for pending melodies
#define SHIP_TASK_PRIORITY    3  // Priority of Buzzer Task


#endif /* CONFIG_H_ */
