/*
 * dsp.h
 *
 */

#ifndef CORE_INC_DSP_H_
#define CORE_INC_DSP_H_

#include <stdint.h>
#include "math.h"
#include "complex.h"
#include "adc.h"
#include "arm_math.h"
#include "arm_const_structs.h"


typedef struct dsp {

	uint8_t processing_request_flag;
	uint8_t dbuf_false_processing_request_error;	// Error flag to signal if data processing has been requested without ready data (debugging)
	uint32_t batch_sn;
	complex complex_data;
	complex prev_complex;
	complex temp_complex;
	int16_t meanI;
	int16_t meanQ;
	int16_t demodulated_IQ[ADC_RX_BUF_SIZE];
	float64_t temp_I;
	float64_t temp_Q;
	int32_t downmix_freq;

	q31_t fft_max_mag;
	uint32_t fft_max_mag_idx;
	uint32_t mix_freq;
	float32_t radians;
	float32_t sine_value;

	q31_t fft_buf[FFT_SIZE*2];
	q31_t fft_mag_buf[FFT_SIZE*2];
	uint8_t	ifft_flag;					// Perform IFFT? (Regular FFT if 0)
	uint8_t bit_reverse_flag;

	uint32_t num_blocks;
	float32_t snr;





	uint32_t prim;

	// TODO: Explain all variables. Decoder heavily depends on the filter implementation that can be found in filter.c and filter.h

} *dsp_t;

/* Declare the extern struct decoder. */
extern struct dsp dsp;


/************* Publicly callable functions *************/
void prvDSPInit();
void prvDMSKDemodulate();
void prvDSPTask( void *pvParameters );		// Task for digital signal processing of the RF signals


#endif /* CORE_INC_DSP_H_ */

