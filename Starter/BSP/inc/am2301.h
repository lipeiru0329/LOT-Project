/*
 * am2301.h
 *
 *  Created on: May 22, 2016
 *      Author: Chen Si
 */

#ifndef AM2301_H_
#define AM2301_H_

#include "stm32f10x.h"

/*
#define DAT_VAL_IN PAin(7)
#define DAT_VAL_OUT PAout(7)
*/
//#define DAT_VAL_IN PCin(6)
//#define DAT_VAL_OUT PCout(6)
//AM2301 for usable pin //HJ
#define DAT_VAL_IN PBin(5)
#define DAT_VAL_OUT PBout(5)

/*void Dat_in(void);
void Dat_out(void);
void AM2301_Start(void);
unsigned char read_byte(void);
void read_hum_temp(uint8_t* hum, uint8_t* temp);*/


/**
 * Enumerations
 *
 * There are several different possible results.
 * If TM_AM2301_OK is returned from read function then you have valid data.
 */
typedef enum {
	TM_AM2301_OK,						/*!< Data OK */
	TM_AM2301_ERROR,					/*!< An error occurred */
	TM_AM2301_CONNECTION_ERROR,			/*!< Device is not connected */
	TM_AM2301_WAITHIGH_ERROR,           /*!< Wait high pulse timeout */
	TM_AM2301_WAITLOW_ERROR,            /*!< Wait low pulse timeout */
	TM_AM2301_WAITHIGH_LOOP_ERROR,      /*!< Loop error for high pulse */
 	TM_AM2301_WAITLOW_LOOP_ERROR,       /*!< Loop error for low pulse */
	TM_AM2301_PARITY_ERROR				/*!< Data read fail */
} TM_AM2301_t;

/**
 * @brief  AM2301 main data structure
 */
typedef struct {
	int16_t Temp; /*!< Temperature in tenths of degrees.
	                   If real temperature is 27.3°C, this variable's value is 273 */
	uint16_t Hum; /*!< Humidity in tenths of percent.
	                   If real humidity is 55.5%, this variable's value is 555 */
} TM_AM2301_Data_t;

/**
 * @}
 */

/**
 * @defgroup TM_AM2301_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes AM2301 sensor
 * @param  None
 * @retval TM_AM2301_OK
 */
TM_AM2301_t TM_AM2301_Init(void);

/**
 * @brief  Reads data from sensor
 * @param  *data: Pointer to @ref TM_AM2301_Data_t data structure to store data into
 * @retval Data valid:
 *            - TM_AM2301_OK: Data valid
 *            - Else: Data not valid
 */
TM_AM2301_t TM_AM2301_Read(TM_AM2301_Data_t* data);

#endif /* AM2301_H_ */
