#include "ts.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

Pen_Holder Pen_Point;

unsigned char flag=0;


int GUI_TOUCH_X_MeasureX()
{
	  unsigned short i;
	  unsigned short sum=0;
	  for(i=0;i<8;i++)
	   	sum+=TPReadX();
	  return sum>>3;

}

int GUI_TOUCH_X_MeasureY()
{
	  unsigned short i;
	  unsigned short sum=0;
	  for(i=0;i<8;i++)
	   	sum+=TPReadY();
	  return sum>>3;
}


void pfSendCmd(unsigned char Data)
{
//	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
////	SPI_I2S_SendData(SPI2,Data);			//mel need to get ADS7486 Datasheet
//	HAL_SPI_Transmit(&hspi2, &Data, sizeof(Data), 100U);
//	SPI_RW_Byte(Data);						//not quite matching up to XPT2046??
}

unsigned short int pfGetResult()
{
	unsigned short int x=0;

	return x;
}

char pfGetBusy()
{
//	if(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))

		return 0;
}


void pfSetCS(char OnOff)
{
	if(OnOff)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

char pfGetPENIRQ()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t touch=0;

	//test for touch
	HAL_GPIO_WritePin(X1_PORT,X1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X2_PORT,X2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Y1_PORT,Y1_Pin, GPIO_PIN_SET);

	//Configure GPIO pins :Y2_Pin digital input
	GPIO_InitStruct.Pin = Y2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Y2_PORT, &GPIO_InitStruct);


	touch = HAL_GPIO_ReadPin(Y2_PORT, Y2_Pin);


	//Configure GPIO pins :Y2_Pin digital output
	GPIO_InitStruct.Pin = Y2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Y2_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(X1_PORT,X1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(X2_PORT,X2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y1_PORT,Y1_Pin, GPIO_PIN_RESET);

	if(!touch) //active low
		return 1;
	else
		return 0;

}




unsigned short int TPReadY(void)
{
	unsigned short int y=0;


	return y;
}


uint32_t TPReadX(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint32_t x=0;

	//setup for touch
	HAL_GPIO_WritePin(X1_PORT,X1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X2_PORT,X2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Y2_PORT,Y2_Pin, GPIO_PIN_SET);

	  //Configure GPIO pins : Y2 input with pullups
	  GPIO_InitStruct.Pin = Y2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(Y2_PORT, &GPIO_InitStruct);

	  //Configure GPIO pins : Y1_Pin analog
	  GPIO_InitStruct.Pin = Y1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(Y1_PORT, &GPIO_InitStruct);


	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  x = HAL_ADC_GetValue(&hadc1);


	//Configure GPIO pins : LCD_RS_Pin and LCD_CS digital output
  GPIO_InitStruct.Pin = Y1_Pin | Y2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(X1_PORT,X1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(X2_PORT,X2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y2_PORT,Y2_Pin, GPIO_PIN_RESET);


	return x;
}



#define TSLEFT 137
#define TSRIGHT 1909
#define TSTOP 161
#define TSBOTTOM 1850

//hard code these so not to recalculate every time
//#define XDIVISOR (TSRIGHT-TSLEFT)/320
//#define YDIVISOR (TSBOTTOM-TSTOP)/240

#ifdef ILI9481

#define XDIVISOR 3.69  	//TSRIGHT-TSLEFT)/480
#define YDIVISOR 5.28	//TSBOTTOM-TSTOP)/320

#elif SSD1963

#define XDIVISOR 2.82  	//TSBOTTOM-TSTOP)/600
#define YDIVISOR 2.22	//TSRIGHT-TSLEFT)/800

#else

#define XDIVISOR 5.54	//(TSRIGHT-TSLEFT)/320
#define YDIVISOR 7.04	//(TSBOTTOM-TSTOP)/240

#endif

void Convert_Pos(void)
{
	Pen_Point.X=GUI_TOUCH_X_MeasureX();
	Pen_Point.Y=GUI_TOUCH_X_MeasureY();

	Pen_Point.X0=(int)((Pen_Point.X-TSLEFT)/XDIVISOR); //widescreen xy, 00 upper left
	Pen_Point.Y0=(int)((Pen_Point.Y-TSTOP)/YDIVISOR);

	pstate.x = Pen_Point.X0;
	pstate.y = Pen_Point.Y0;

	if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
		pstate.Pressed = 0;		//pressed active lo
	else
		pstate.Pressed = 1;

	GUI_PID_StoreState(&pstate);
}

