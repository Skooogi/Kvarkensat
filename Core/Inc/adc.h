/*
 * adc.h
 *
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_


#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "liquid.h"


#define ADC_RX_PRIORITY				( tskIDLE_PRIORITY + 1 )	// ADC receive task priority
#define ADC_RX_BUF_SIZE  			0x200						// Size of the ADC receive buffer
#define ADC_TEMPERATURE_BUF_SIZE	0x20						// TODO: Just a preliminary guess
#define ADC_SAMPLERATE 				288000.0f					// Sample rate of the ADC



/****** Only for prerecorded test data START ******/
#define TEST_LENGTH_SAMPLES ((uint32_t) 2048)
/****** Only for prerecorded test data END ******/


/* Structs for storing the states of the RF receiving ADCs */
typedef struct rfadc {

	uint8_t converting;													// Flag to tell whether the ADC is in the middle of conversions or not
	ALIGN_32BYTES ( uint32_t rx_buf[ADC_RX_BUF_SIZE] );					// This buffer is only needed for ADC1 (in multimode ADC operations, converted data of both ADC1 and ADC2 are stored in the result register of ADC1)
	ALIGN_32BYTES ( int32_t data[ADC_RX_BUF_SIZE] );

} *rfadc_t;

/* Declare the extern structs adcIQ and adcQ. */
extern struct rfadc adcIQ;

/* Struct for storing the state of the MCU temperature measuring ADC */
typedef struct tempadc {

	uint8_t converting;														// Flag to tell whether the ADC is in the middle of conversions or not
	ALIGN_32BYTES (uint16_t temperature_buf[ADC_TEMPERATURE_BUF_SIZE]);		// TODO: Is the alignment necessary?

} *tempadc_t;

/* Declare the extern struct adcT */
extern struct tempadc adcT;



/************* Publicly callable functions *************/
void prvADCTask( void *pvParameters );							// Task acquiring latest ADC data
void prvADCInit();												// Initialize the state of the ADC
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);		// ADC conversion half complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);			// ADC conversion complete callback

#endif /* INC_ADC_H_ */
