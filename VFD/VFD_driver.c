/*
 * VFD_driver.c
 *
 *  Created on: 18 нояб. 2021 г.
 *      Author: maxsp
 */

#include "VFD_driver.h"

extern const unsigned char font_asc2_1608[95][16];
extern const unsigned char font_asc2_1206[95][12];

volatile uint8_t VideoBuffer[512]; // Видеобуфер 512 байт для дисплея 128х32
volatile uint8_t Brightness = 3;


void Clear_Display(void)
{
	uint16_t i;
	for(i = 0; i < 1408; i++)
			VideoBuffer[i] = 0x00;

}


void Draw_Pixel(uint8_t x, uint8_t y, uint8_t state)
{
	uint8_t *Video_Addr =  VideoBuffer;
	uint8_t x1, pattern;


	x &= 0x7F;		// Коррекция координат в случае выхода за пределы
	y &= 0x1F;
	y = 31 - y;		// ЧТобы нулевая точка была в левом верхнем углу

	if(x == 0 || x == 127)
		x1 = 0;
	else
		x1 = ((x - 1) >> 1) + 1;

	Video_Addr += x1 * 8 + y / 4;

	if(x & 0x01)
		pattern = 0x80;
	else
		pattern = 0x40;

	pattern = pattern >> ((y & 0x03) << 1);

	if(state)
		*Video_Addr  |= pattern;
	else
		*Video_Addr  &= pattern ^ 0xFF;
}


void Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set)
{
  int16_t dx = x2 - x1; // calculate delta and step's sign
  int16_t dy = y2 - y1;
  int16_t step_x, step_y;

  if (dy < 0)
  {
    dy = -dy;
    step_y = -1;
  }
  else
    step_y = 1;

  if (dx < 0)
  {
    dx = -dx;
    step_x = -1;
  }
  else
    step_x = 1;

  dy <<= 1; dx <<= 1;

  if (dx > dy)
  {
    int16_t fract = dy -(dx >> 1);
    while(x1 != x2)
    {
      if(fract >= 0)
      {
        y1 += step_y;
        fract -= dx;
      }
      x1 += step_x;
      fract += dy;
      Draw_Pixel(x1, y1, set);
    }
  }
  else
  {
    int16_t fract = dx -(dy >> 1);
    while(y1 != y2)
    {
      if(fract >= 0)
      {
        x1 += step_x;
        fract -= dy;
      }
      y1 += step_y;
      fract += dx;
      Draw_Pixel(x1, y1, set);
    }
  }
}




void Draw_Rectangle(uint8_t	x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set)
{
  Draw_Line(x1, y1, x2, y1, set);
  Draw_Line(x2, y1, x2, y2, set);
  Draw_Line(x2, y2, x1, y2, set);
  Draw_Line(x1, y2, x1, y1, set);
}

void Draw_Circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t set)
{
  int16_t f = 1 - radius;
  int16_t ddF_x = 0;
  int16_t ddF_y = -2 * radius;
  int16_t x0 = 0;
  int16_t y0 = radius;

  Draw_Pixel(x,y + radius, set);
  Draw_Pixel(x,y - radius, set);
  Draw_Pixel(x + radius, y, set);
  Draw_Pixel(x - radius, y, set);

  while (x0 < y0)
  {
    if(f >= 0)
    {
      y0--;
      ddF_y += 2;
      f += ddF_y;
    }
    x0++;
    ddF_x += 2;
    f += ddF_x + 1;

    Draw_Pixel(x + x0, y + y0, set);
    Draw_Pixel(x - x0, y + y0, set);
    Draw_Pixel(x + x0, y - y0 , set);
    Draw_Pixel(x - x0, y - y0, set);
    Draw_Pixel(x + y0, y + x0, set);
    Draw_Pixel(x - y0, y + x0, set);
    Draw_Pixel(x + y0, y - x0 , set);
    Draw_Pixel(x - y0, y - x0, set);

  }
}


void Show_Char(uint8_t X, uint8_t Y, uint8_t Chr, uint8_t Size, uint8_t Mode)
{
	uint8_t Temp, t, t1;
	uint8_t y0 = Y;

	Chr = Chr - ' ';
    for(t = 0; t < Size; t ++)
    {
			if(Size == 12)
			{
				Temp = font_asc2_1206[Chr][t];
			}
			else
			{
				Temp = font_asc2_1608[Chr][t];
			}
      for(t1 = 0; t1 < 8; t1 ++)
			{
				if(Temp & 0x80)
					Draw_Pixel(X, Y, Mode);
				else
					Draw_Pixel(X, Y, !Mode);
				Temp <<= 1;
				Y ++;
				if((Y - y0) == Size)
				{
					Y = y0;
					X ++;
					break;
				}
			}
    }
}


void Show_String(uint8_t X, uint8_t Y,  uint8_t Size, uint8_t *Str)
{
#define MAX_CHAR_POSX 119
#define MAX_CHAR_POSY 20

    while(*Str != '\0')
    {
        if(X > MAX_CHAR_POSX)
		{
			X = 0;
			Y += Size;
		}
        if(Y > MAX_CHAR_POSY)
		{
			Y = X = 0;
		//	Clear_Display();
		}
        Show_Char(X, Y, *Str, Size, 1);
				if(Size == 16)
					X += 8;
				else
					X += 6;
        Str ++;
    }
}

