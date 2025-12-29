#include "SPI_Init.h"

void EPD_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(EPD_SCL_GPIO_CLK|EPD_SDA_GPIO_CLK|EPD_RES_GPIO_CLK
												 |EPD_DC_GPIO_CLK|EPD_CS_GPIO_CLK|EPD_BUSY_GPIO_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=EPD_SCL_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(EPD_SCL_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=EPD_SDA_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(EPD_SDA_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=EPD_RES_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(EPD_RES_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=EPD_DC_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(EPD_DC_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=EPD_CS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(EPD_CS_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=EPD_BUSY_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;
	GPIO_Init(EPD_BUSY_GPIO_PORT,&GPIO_InitStructure);
	
}


void EPD_WR_Bus(u8 dat)
{
	u8 i;
	EPD_CS_Clr();
	for(i=0;i<8;i++)
	{
		EPD_SCL_Clr();
		if(dat&0x80)
		{
			EPD_SDA_Set();
		}
		else
		{
			EPD_SDA_Clr();
		}
		EPD_SCL_Set();
		dat<<=1;
	}
	EPD_CS_Set();	
}

void EPD_WR_REG(u8 reg)
{
	EPD_DC_Clr();
	EPD_WR_Bus(reg);
	EPD_DC_Set();
}
void EPD_WR_DATA8(u8 dat)
{
	EPD_DC_Set();
	EPD_WR_Bus(dat);
	EPD_DC_Set();
}





