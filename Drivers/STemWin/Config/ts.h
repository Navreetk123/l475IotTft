/*
 * ts.h
 *
 *  Created on: Apr 21, 2019
 *      Author: mdundas
 */

#ifndef STEMWIN_CONFIG_TS_H_
#define STEMWIN_CONFIG_TS_H_

#include "GUI.h"
#include <stdint.h>

//mel
 void pfSendCmd(unsigned char Data);
 unsigned short int pfGetResult();
 char pfGetBusy();
 void pfSetCS(char OnOff);
 char pfGetPENIRQ();

GUI_PID_STATE pstate; //for stemwin






typedef struct
{
	unsigned short int X0;
	unsigned short int Y0;
	uint32_t X;
	unsigned short int Y;
	unsigned char  Key_Sta;
	unsigned char  Key_LSta;
    unsigned char  noise;
    unsigned short inttime;
	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;

// Touchscreen connection:

#define Y1_Pin LCD_CS_Pin //A3 	need two analog inputs
#define X1_Pin LCD_RS_Pin //A2
#define Y2_Pin LCD_D1_Pin //9
#define X2_Pin LCD_D0_Pin //8

#define Y1_PORT LCD_CS_GPIO_Port
#define X1_PORT LCD_RS_GPIO_Port
#define Y2_PORT LCD_D1_GPIO_Port
#define X2_PORT LCD_D0_GPIO_Port


extern Pen_Holder Pen_Point;



void TPReadXY(uint32_t *X, uint32_t *Y);
void GUI_TOUCH_X_MeasureXY(uint32_t *sumx, uint32_t *sumy);

void EXTI9_5_IRQHandler(void);
void NVIC_TOUCHConfiguration(void);
void touch_init(void);

void Convert_Pos(void);
void Touch_Adjust(void);


#define Key_Down 0x01
#define Key_Up   0x00

#ifdef __cplusplus
}
#endif



#endif /* STEMWIN_CONFIG_TS_H_ */
