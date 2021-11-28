/*
 * VFD_driver.h
 *
 *  Created on: 18 нояб. 2021 г.
 *      Author: maxsp
 */

#ifndef VFD_VFD_DRIVER_H_
#define VFD_VFD_DRIVER_H_

#include <stdint.h>

void Clear_Display(void);
void Draw_Pixel(uint8_t x, uint8_t y, uint8_t state);
void Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set);
void Draw_Rectangle(uint8_t	x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set);
void Draw_Circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t set);
void Show_String(uint8_t X, uint8_t Y,  uint8_t Size, uint8_t *Str);


#endif /* VFD_VFD_DRIVER_H_ */
