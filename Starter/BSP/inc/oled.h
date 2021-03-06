#ifndef _HAL_OLED_H
#define _HAL_OLED_H

#include <stdio.h>
#include <stm32f10x.h>
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"

#define OLED_CMD    0
#define OLED_DAT    1


/*************************************************************
*1 > VCC    -----  3.3V
*2 > GBD    -----  GND
*3 > NC     -----  NC
*4 > DIN    -----  PA7
*5 > SCLK   -----  PA5
*7 > D/C    -----  PA6
*8 > RES    -----  PA4
***************************************************************/
#define OLED_SPIx                		SPI1
#define OLED_SPIx_CLK          			RCC_APB2Periph_SPI1
#define OLED_SPIx_CLK_Cmd		       	RCC_APB2PeriphClockCmd

#define OLED_SPIx_MOSI_PIN         GPIO_Pin_7
#define OLED_SPIx_MOSI_PORT        GPIOA
#define OLED_SPIx_MOSI_CLK         RCC_APB2Periph_GPIOA

#define OLED_SPIx_SCK_PIN          GPIO_Pin_5
#define OLED_SPIx_SCK_PORT         GPIOA
#define OLED_SPIx_SCK_CLK          RCC_APB2Periph_GPIOA

#define OLED_DC_PORT              GPIOA
#define OLED_DC_CLK          			RCC_APB2Periph_GPIOA
#define OLED_DC_PIN               GPIO_Pin_6

#define OLED_RES_PORT             GPIOA
#define OLED_RES_CLK          		RCC_APB2Periph_GPIOA
#define OLED_RES_PIN              GPIO_Pin_4


#define OLED_DC_SET()     				GPIO_WriteBit(OLED_DC_PORT, OLED_DC_PIN, Bit_SET)
#define OLED_DC_RESET()						GPIO_WriteBit(OLED_DC_PORT, OLED_DC_PIN, Bit_RESET)

#define OLED_RES_SET()     				GPIO_WriteBit(OLED_RES_PORT, OLED_RES_PIN, Bit_SET)
#define OLED_RES_RESET()					GPIO_WriteBit(OLED_RES_PORT, OLED_RES_PIN, Bit_RESET)


void OLED_Init(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_Refresh_Gram(void);
void LCD_Clear(uint8_t Color);
void OLED_DrawPoint(uint8_t Xpos,uint8_t Ypos,uint8_t Fill);
void OLED_ShowChar(uint8_t X, uint8_t Y, uint8_t Chr, uint8_t Size, uint8_t Mode);
void OLED_ShowString(uint8_t X, uint8_t Y, const char *Str, uint8_t fontsize);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,const uint8_t BMP[]);
void OLED_ShowCN(uint8_t x,uint8_t y,uint8_t no);
#endif /*_HAL_OLED_H*/

