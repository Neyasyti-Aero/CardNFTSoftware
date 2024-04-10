/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f415_wk_config.h"
#include "display.h"
#include "st7789.h"
#include "images.h"
#include "gif.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */
#define DISPLAY_RES1 240
#define DISPLAY_RES2 320
/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
volatile uint32_t millis = 0;
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* nvic config. */
  wk_nvic_config();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify parameters memory_base_addr and buffer_size */
  wk_dma_channel_config(DMA1_CHANNEL1, (uint32_t)&SPI2->dt, 0, 0);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init spi2 function. */
  wk_spi2_init();
	
	/* init exint function. */
  wk_exint_config();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init gpio function. */
  wk_gpio_config();

  /* add user code begin 2 */
	delay_init();
	gpio_bits_set(GPIOA, GPIO_PINS_9); // screen power supply
	delay_ms(100);
	//gpio_bits_set(GPIOA, GPIO_PINS_10); // screen led
	gpio_bits_set(GPIOA, GPIO_PINS_7); // board led
	
	
	// Here we start display initialization
	LCD_Handler lcd1;
	LCD_BackLight_data bl_dat = { 0, 0, GPIOA, GPIO_PINS_10, 100 };
	LCD_DMA_TypeDef dma_tx = {DMA1, FLEX_CHANNEL1};
	LCD_SPI_Connected_data spi_dat = {SPI2, dma_tx, LCD_RES_GPIO_Port, LCD_RES_Pin, LCD_DC_GPIO_Port, LCD_DC_Pin, LCD_CS_GPIO_Port, LCD_CS_Pin }; // in main.h
	LCD = LCD_DisplayAdd(LCD, &lcd1, DISPLAY_RES1, DISPLAY_RES2, ST7789_CONTROLLER_WIDTH, ST7789_CONTROLLER_HEIGHT, 0, 0, PAGE_ORIENTATION_PORTRAIT_MIRROR, ST7789_Init, ST7789_SetWindow, ST7789_SleepIn, ST7789_SleepOut, &spi_dat, LCD_DATA_16BIT_BUS, bl_dat );
	
	LCD_Handler *lcd = LCD;
	LCD_Init(lcd);
	
	// Here we start buttons state check
	exint_interrupt_enable(EXINT_LINE_3, TRUE);
	exint_interrupt_enable(EXINT_LINE_4, TRUE);
	
	g_bGIFStartRequest = 1;
	
  while(1)
  {
		if (g_bGIFStartRequest)
			PlayGif();
		delay_ms(10);
		
		if (g_bShowImageRequest)
			ShowImage(g_iCurrentImage);
		delay_ms(10);
  }
}
