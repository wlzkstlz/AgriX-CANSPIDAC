/*
*********************************************************************************************************
*
*	ģ������ : DAC8562/8563 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*	��    �� : V1.0
*	˵    �� : DAC8562/8563ģ���CPU֮�����SPI�ӿڡ�����������֧��Ӳ��SPI�ӿں����SPI�ӿڡ�
*			  ͨ�����л���
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-17  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

//#include "bsp.h"
#include "dac8562.h"
#include "spi.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

//#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
//#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */

///*
//	DAC8501ģ�����ֱ�Ӳ嵽STM32-V5������CN19��ĸ(2*4P 2.54mm)�ӿ���

//    DAC8562/8563ģ��    STM32F407������
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
//	DAC8562��������:
//	1������2.7 - 5V;  ������ʹ��3.3V��
//	4���ο���ѹ2.5V��ʹ���ڲ��ο�

//	��SPI��ʱ���ٶ�Ҫ��: �ߴ�50MHz�� �ٶȺܿ�.
//	SCLK�½��ض�ȡ����, ÿ�δ���24bit���ݣ� ��λ�ȴ�
//*/

//#if !defined(SOFT_SPI) && !defined(HARD_SPI)
// 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
//#endif

//#ifdef SOFT_SPI		/* ���SPI */
//	/* ����GPIO�˿� */
//	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
//	#define PORT_SCLK	GPIOB
//	#define PIN_SCLK	GPIO_Pin_3
//	
//	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
//	#define PORT_DIN 	GPIOB
//	#define PIN_DIN 	GPIO_Pin_5
//	
//	/* Ƭѡ */
//	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
//	#define PORT_SYNC	GPIOF
//	#define PIN_SYNC	GPIO_Pin_7

//	/* LDAC, ���Բ��� */
//	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
//	#define PORT_LDAC	GPIOA
//	#define PIN_LDAC	GPIO_Pin_4
//	
//	/* CLR, ���Բ��� */
//	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
//	#define PORT_CLR	GPIOF
//	#define PIN_CLR 	GPIO_Pin_7

//	/* ���������0����1�ĺ� */
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

//#ifdef HARD_SPI		/* Ӳ��SPI (δ��) */
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
*	�� �� ��: InitDAC8562
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� DAC8562
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitDAC8562(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;

//#ifdef SOFT_SPI
	setDacSync(1);	/* SYNC = 1 */
	//HAL_Delay(1);

//	/* ��GPIOʱ�� */
//	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

//	/* ���ü����������IO */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

//	GPIO_InitStructure.GPIO_Pin = PIN_SCLK;
//	GPIO_Init(PORT_SCLK, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
//	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = PIN_SYNC;
//	GPIO_Init(PORT_SYNC, &GPIO_InitStructure);
//#endif

	/* Power up DAC-A and DAC-B */
	DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	
	/* LDAC pin inactive for DAC-B and DAC-A  ��ʹ��LDAC���Ÿ������� */
	DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));

	/* ��λ2��DAC���м�ֵ, ���2.5V */
	DAC8562_SetData(0, 32767);
	DAC8562_SetData(1, 32767);

	/* ѡ���ڲ��ο�����λ2��DAC������=2 ����λʱ���ڲ��ο��ǽ�ֹ��) */
	DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd
*	����˵��: ��SPI���߷���24��bit���ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_WriteCmd(uint32_t _cmd)
{
//	uint8_t i;
//	
	setDacSync(0);
	//HAL_Delay(1);
	
	/*��DAC8562 SCLKʱ�Ӹߴ�50M����˿��Բ��ӳ� */
	
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
*	�� �� ��: DAC8562_SetData
*	����˵��: ����DAC��������������¡�
*	��    ��: _ch, ͨ��, 0 , 1
*		     _data : ����
*	�� �� ֵ: ��
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
