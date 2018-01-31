/*
*********************************************************************************************************
*
*	模块名称 : DAC8562/8563 驱动模块(单通道带16位DAC)
*	文件名称 : bsp_dac8562.c
*	版    本 : V1.0
*	说    明 : DAC8562/8563模块和CPU之间采用SPI接口。本驱动程序支持硬件SPI接口和软件SPI接口。
*			  通过宏切换。
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2014-01-17  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

//#include "bsp.h"
#include "dac8562.h"
#include "spi.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

//#define SOFT_SPI		/* 定义此行表示使用GPIO模拟SPI接口 */
//#define HARD_SPI		/* 定义此行表示使用CPU的硬件SPI接口 */

///*
//	DAC8501模块可以直接插到STM32-V5开发板CN19排母(2*4P 2.54mm)接口上

//    DAC8562/8563模块    STM32F407开发板
//	  GND   ------  GND    
//	  VCC   ------  3.3V
//	  
//	  LDAC  ------  PA4/NRF905_TX_EN/NRF24L01_CE/DAC1_OUT
//      SYNC  ------  PF7/NRF24L01_CSN
//      	  
//      SCLK  ------  PB3/SPI3_SCK
//      DIN   ------  PB5/SPI3_MOSI

//			------  PB4/SPI3_MISO
//	  CLR   ------  PH7/NRF24L01_IRQ
//*/

///*
//	DAC8562基本特性:
//	1、供电2.7 - 5V;  【本例使用3.3V】
//	4、参考电压2.5V，使用内部参考

//	对SPI的时钟速度要求: 高达50MHz， 速度很快.
//	SCLK下降沿读取数据, 每次传送24bit数据， 高位先传
//*/

//#if !defined(SOFT_SPI) && !defined(HARD_SPI)
// 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
//#endif

//#ifdef SOFT_SPI		/* 软件SPI */
//	/* 定义GPIO端口 */
//	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
//	#define PORT_SCLK	GPIOB
//	#define PIN_SCLK	GPIO_Pin_3
//	
//	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
//	#define PORT_DIN 	GPIOB
//	#define PIN_DIN 	GPIO_Pin_5
//	
//	/* 片选 */
//	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
//	#define PORT_SYNC	GPIOF
//	#define PIN_SYNC	GPIO_Pin_7

//	/* LDAC, 可以不用 */
//	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
//	#define PORT_LDAC	GPIOA
//	#define PIN_LDAC	GPIO_Pin_4
//	
//	/* CLR, 可以不用 */
//	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
//	#define PORT_CLR	GPIOF
//	#define PIN_CLR 	GPIO_Pin_7

//	/* 定义口线置0和置1的宏 */
//	#define SYNC_0()	PORT_SYNC->BSRRH = PIN_SYNC
//	#define SYNC_1()	PORT_SYNC->BSRRL = PIN_SYNC

//	#define SCLK_0()	PORT_SCLK->BSRRH = PIN_SCLK
//	#define SCLK_1()	PORT_SCLK->BSRRL = PIN_SCLK

//	#define DIN_0()		PORT_DIN->BSRRH = PIN_DIN
//	#define DIN_1()		PORT_DIN->BSRRL = PIN_DIN

//	#define LDAC_0()	PORT_LDAC->BSRRH = PIN_LDAC
//	#define LDAC_1()	PORT_LDAC->BSRRL = PIN_LDAC

//	#define CLR_0()		PORT_CLR->BSRRH = PIN_CLR
//	#define CLR_1()		PORT_CLR->BSRRL = PIN_CLR	
//#endif

//#ifdef HARD_SPI		/* 硬件SPI (未做) */
//	;
//#endif



void setDacSync(uint8_t bit)
{
	if(bit)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void setDin(uint8_t bit)
{
	if(bit)
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	else
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
}


void setCLK(uint8_t bit)
{
	
	if(bit)
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
	else
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
}

/*
*********************************************************************************************************
*	函 数 名: InitDAC8562
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 DAC8562
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void InitDAC8562(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;

//#ifdef SOFT_SPI
	setDacSync(1);	/* SYNC = 1 */
	//HAL_Delay(1);

//	/* 打开GPIO时钟 */
//	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

//	/* 配置几个推完输出IO */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

//	GPIO_InitStructure.GPIO_Pin = PIN_SCLK;
//	GPIO_Init(PORT_SCLK, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
//	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = PIN_SYNC;
//	GPIO_Init(PORT_SYNC, &GPIO_InitStructure);
//#endif

	/* Power up DAC-A and DAC-B */
	DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	
	/* LDAC pin inactive for DAC-B and DAC-A  不使用LDAC引脚更新数据 */
	DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));

	/* 复位2个DAC到中间值, 输出2.5V */
	DAC8562_SetData(0, 32767);
	DAC8562_SetData(1, 32767);

	/* 选择内部参考并复位2个DAC的增益=2 （复位时，内部参考是禁止的) */
	DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd
*	功能说明: 向SPI总线发送24个bit数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_WriteCmd(uint32_t _cmd)
{
//	uint8_t i;
//	
	setDacSync(0);
	//HAL_Delay(1);
	
	/*　DAC8562 SCLK时钟高达50M，因此可以不延迟 */
	
//	for(uint8_t i=0;i<3;i++)
//	{
//		uint8_t data=(_cmd&(0xff<<((2-i)*8)))>>((2-i)*8);
//		HAL_SPI_Transmit(&hspi1,&data,1,10);
//	}
	
	for(uint8_t i = 0; i < 24; i++)
	{
		if (_cmd & 0x800000)
		{
			setDin(1);
		}
		else
		{
			setDin(0);
		}
		setCLK(1);
		_cmd <<= 1;
		setCLK(0);
	}
		//HAL_Delay(1);
	setDacSync(1);
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_SetData
*	功能说明: 设置DAC输出，并立即更新。
*	形    参: _ch, 通道, 0 , 1
*		     _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_SetData(uint8_t _ch, uint16_t _dac)
{
	if (_ch == 0)
	{
		/* Write to DAC-A input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (0 << 16) | (_dac << 0));
	}
	else if (_ch == 1)
	{
		/* Write to DAC-B input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (1 << 16) | (_dac << 0));		
	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
