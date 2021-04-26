// Copyright (c) 2015-18, Joe Krachey
// All rights reserved.
//
// Redistribution and use in source or binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in source form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef __SPACE_SHIP_H__
#define __SPACE_SHIP_H__

#include <stdint.h>
#include <main.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <task_space_ship.h>
#include <lcd.h>



#define RED_5   0x1F
#define GREEN_6 0x0F
#define BLUE_5  0x0F
#define BLACK 0x0000

#define FG_COLOR (RED_5 << 11)
#define BG_COLOR BLACK

#define X_COORD 63
#define Y_COOR  63
#define X_LEN   20
#define Y_LEN   20


/* Bitmap info for space_ship */
extern const uint8_t space_shipBitmaps[];
extern const uint8_t space_shipWidthPages;
extern const uint8_t space_shipWidthPixels;
extern const uint8_t space_shipHeightPixels;

extern const uint8_t Bitmaps_Lazer[];
extern const uint8_t space_lazerWidth;
extern const uint8_t space_lazerHeight;

#endif
