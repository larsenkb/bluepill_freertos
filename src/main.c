/*
 * This is freertos release 9.0.0 configured for a stm32f103vct6 board,
 * named Hy-MiniSTM32V. It should run on any stm32f103 with a few changes.
 * 
 * There are two tasks running. One is blinking two LEDs, the other 
 * is sending stuff via USART1.
 * 
 * Author: Nils Stec, stecdose@gmail.com
 * Date 2016-07-01
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f10x.h"
#include "usart.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"


// LED1=PC13
#if 0
#define LED1_ON()	GPIO_SetBits(GPIOC , GPIO_Pin_13)
#define LED1_OFF()	GPIO_ResetBits(GPIOC , GPIO_Pin_13)
#define LED1_ON()	{GPIO_WriteBit(GPIOC , GPIO_Pin_13, Bit_SET);}
#define LED1_OFF()	{GPIO_WriteBit(GPIOC , GPIO_Pin_13, Bit_RESET);}
#endif
#define GPIOC_BSRR	(*(volatile unsigned long*)(GPIOC_BASE + 0x10))
#define GPIOC_BRR	(*(volatile unsigned long*)(GPIOC_BASE + 0x14))
#define LED1_ON()	GPIOC_BSRR = 1<<13
#define LED1_OFF()	GPIOC_BRR  = 1<<13
//#define LED2_ON()	GPIO_SetBits(GPIOB , GPIO_Pin_1)
//#define LED2_OFF()	GPIO_ResetBits(GPIOB , GPIO_Pin_1)
void init_leds();

void vT_usart(void *p);
void vT_led(void *p);

void vT_usart(void *p) {
	// Block for 500ms.
	const portTickType xDelay = 500 / portTICK_RATE_MS;
	
	for(;;) {
		usart1_puts("FreeRTOS V9.0.0 demo on STM32F103VCT6 STM32F103VC\r\n");
		usart1_puts(" 128k flash, 20k ram ..........\r\n");
		vTaskDelay(xDelay);		
	}
}

void vT_led(void *p) {
	// Block for 100ms.
	const portTickType xDelay = 100 / portTICK_RATE_MS;
	
	LED1_OFF();
//	LED2_OFF();
	for(;;) {
		LED1_ON();
//		LED2_OFF();
		
		vTaskDelay(xDelay*10);
		
		LED1_OFF();
//		LED2_ON();
		
		vTaskDelay(xDelay);
	}
}

int main(void)
{
//	SystemInit();
//	usart1_init();

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );  //Needed for STM32F10x

	init_leds();
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

//	xTaskCreate(vT_usart, (const char*) "USART Task", 128, NULL, 1, NULL);
	xTaskCreate(vT_led, (const char*) "LED Task", 128, NULL, 1, NULL);

	// Start RTOS scheduler

	vTaskStartScheduler();


	for(;;) {
		
	}
		
	return 0;
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
// 	printf("ERROR: vApplicationStackOverflowHook(): Task \"%s\" overflowed its stack\n", pcTaskName);
// 	fflush(stdout);
// 	assert(false);
}

void init_leds() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);

	// LED1=PC13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; // | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	if (SysTick_Config(SystemCoreClock / 1000))
//		while (1);

	return;
}
