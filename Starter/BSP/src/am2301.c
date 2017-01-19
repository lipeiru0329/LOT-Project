/*
 * am2301.c
 *
 *  Created on: May 22, 2016
 *      Author: Chen Si
 */

#include "am2301.h"
#include "stm32f10x.h"
#include "bsp.h"
#include "sys.h"
/*
 uint8_t cnt = 0;

 unsigned char hum_h, hum_l, temp_h, temp_l, check;

 void Dat_in(void) {
 GPIO_InitTypeDef GPIO_InitStructure;
 //pull-up input
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 }

 void Dat_out(void) {
 GPIO_InitTypeDef GPIO_InitStructure;
 //push-pull output
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 }

 void AM2301_Start(void) {
 Dat_out();
 DAT_VAL_OUT = 0;
 delay_1ms();
 DAT_VAL_OUT = 1;
 delay_nus(20);
 Dat_in();

 while ((!DAT_VAL_IN) && (cnt < 8)) {
 cnt++;
 delay_nus(10);
 }
 cnt = 0;

 while (DAT_VAL_IN && (cnt < 8)) {
 cnt++;
 delay_nus(10);
 }
 cnt = 0;

 }

 uint8_t read_byte(void) {
 uint8_t n, byte = 0, dat;
 for (n = 0; n < 8; n++) {
 while (!DAT_VAL_IN && (cnt < 6)) {
 cnt++;
 delay_nus(10);
 }
 cnt = 0;
 delay_nus(40);
 dat = 0;
 if (DAT_VAL_IN)
 dat = 1;
 while (DAT_VAL_IN && (cnt < 4)) {
 cnt++;
 delay_nus(10);
 }
 cnt = 0;
 byte = (byte << 1) | dat;
 }
 return byte;
 }

 void read_hum_temp(uint8_t* hum, uint8_t* temp) {
 uint8_t a;
 AM2301_Start();
 hum_h = read_byte();
 hum_l = read_byte();
 temp_h = read_byte();
 temp_l = read_byte();
 check = read_byte();

 while ((!DAT_VAL_IN) && (cnt < 6)) {
 cnt++;
 delay_nus(10);
 }
 cnt = 0;
 Dat_out();
 DAT_VAL_OUT = 1;
 a = hum_h + hum_l + temp_h + temp_l;
 if (a == check) {
 *hum = *temp = 0;
 *hum = ((*hum | hum_h) << 8) | hum_l;
 *temp = ((*temp | temp_h) << 8) | temp_l;

 } else {
 *hum = *temp = 11;
 }
 }*/

void Dat_in(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	//pull-up input //HJ changes to GPIOB
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//HJ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Dat_out(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	//push-pull output //HJ changes to GPIOB
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//HJ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Private functions */
TM_AM2301_t TM_AM2301_INT_Read(TM_AM2301_Data_t* data);

TM_AM2301_t TM_AM2301_Init(void) {
	Dat_in();
	/* Return OK */
	return TM_AM2301_OK;
}

TM_AM2301_t TM_AM2301_Read(TM_AM2301_Data_t* data) {
	/* Read data */
	return TM_AM2301_INT_Read(data);
}

/* Internal function */
TM_AM2301_t TM_AM2301_INT_Read(TM_AM2301_Data_t* data) {
	volatile uint32_t time;
	uint8_t i, j, d[5];
	/* Pin output */
	Dat_out();
	/* Set pin low for ~800-1000 us */
	DAT_VAL_OUT = 0;
	Delay_nus(1000);
	/* Set pin high to ~30 us */
	DAT_VAL_OUT = 1;
	Delay_nus(30);

	/* Read mode */
	Dat_in();
	;

	time = 0;
	/* Wait 20us for acknowledge, low signal */
	while (DAT_VAL_IN) {
		if (time > 20) {
			return TM_AM2301_CONNECTION_ERROR;
		}
		/* Increase time */
		time++;
		/* Wait 1 us */
		Delay_nus(1);
	}

	time = 0;
	/* Wait high signal, about 80-85us long (measured with logic analyzer) */
	while (!DAT_VAL_IN) {
		if (time > 85) {
			return TM_AM2301_WAITHIGH_ERROR;
		}
		/* Increase time */
		time++;
		/* Wait 1 us */
		Delay_nus(1);
	}

	time = 0;
	/* Wait low signal, about 80-85us long (measured with logic analyzer) */
	while (DAT_VAL_IN) {
		if (time > 85) {
			return TM_AM2301_WAITLOW_ERROR;
		}
		/* Increase time */
		time++;
		/* Wait 1 us */
		Delay_nus(1);
	}

	/* Read 5 bytes */
	for (j = 0; j < 5; j++) {
		d[j] = 0;
		for (i = 8; i > 0; i--) {
			/* We are in low signal now, wait for high signal and measure time */
			time = 0;
			/* Wait high signal, about 57-63us long (measured with logic analyzer) */
			while (!DAT_VAL_IN) {
				if (time > 75) {
					return TM_AM2301_WAITHIGH_LOOP_ERROR;
				}
				/* Increase time */
				time++;
				/* Wait 1 us */
				Delay_nus(1);
			}
			/* High signal detected, start measure high signal, it can be 26us for 0 or 70us for 1 */
			time = 0;
			/* Wait low signal, between 26 and 70us long (measured with logic analyzer) */
			while (DAT_VAL_IN) {
				if (time > 90) {
					return TM_AM2301_WAITLOW_LOOP_ERROR;
				}
				/* Increase time */
				time++;
				/* Wait 1 us */
				Delay_nus(1);
			}

			if (time > 10 && time < 37) {
				/* We read 0 */
			} else {
				/* We read 1 */
				d[j] |= 1 << (i - 1);
			}
		}
	}

	/* Check for parity */
	if (((d[0] + d[1] + d[2] + d[3]) & 0xFF) != d[4]) {
		/* Parity error, data not valid */
		data->Hum = d[0] << 8 | d[1];
		/* Negative temperature */
		if (d[2] & 0x80) {
			data->Temp = -((d[2] & 0x7F) << 8 | d[3]);
		} else {
			data->Temp = (d[2]) << 8 | d[3];
		}
		return TM_AM2301_PARITY_ERROR;
	}

	/* Set humidity */
	data->Hum = d[0] << 8 | d[1];
	/* Negative temperature */
	if (d[2] & 0x80) {
		data->Temp = -((d[2] & 0x7F) << 8 | d[3]);
	} else {
		data->Temp = (d[2]) << 8 | d[3];
	}

	/* Data OK */
	return TM_AM2301_OK;
}
