/*
 * dsp.h
 *
 */

#ifndef CORE_INC_DSP_H_
#define CORE_INC_DSP_H_


#include "adc.h"
#include "decoder.h"
#include "complex.h"
#include "liquid.h"


#define FFT_SIZE 						ADC_RX_BUF_SIZE/2			// Number of samples for FFT
#define SYMBOLRATE						9600.0f						// AIS data baudrate
#define SAMPLES_PER_SYMBOL				ADC_SAMPLERATE/SYMBOLRATE	// How many samples per symbol for raw ADC data
#define DECIMATION_RATIO				0.1f						// Only one out of 10 samples are not discarded in decimation (by the resampler)
#define ANTIALIAS_CUTOFF				DECIMATION_RATIO*1			// The cutoff frequency of the anti-aliasing filter before resampling (0 <= fc <= 0.5 in normalized frequencies) TODO: Tweak (increase) value if cutoff should give more slack


/* A struct to store FFT (fft) object options. */
struct fft
{
	unsigned int size;  											// == 256, input data size
	int flags;        												// FFT flags (typically ignored)
	complex float fft_buf[FFT_SIZE*2];								// Buffer to store data for FFT calculation
	float mag_buf[FFT_SIZE*2];										// Buffer to store the magnitude of the FFT
	float max_mag;													// The value of the maximum magnitude of the FFT
	uint32_t max_mag_idx;											// The index of the maximum magnitude of the FFT
	fftplan fft;													// FFT plan object
};

/* A struct to store filter (firfilt) object options. */
struct filter
{
    firfilt_crcf filter;											// Filter object
    float fc;        												// Filter cutoff frequency
    float ft;         												// Filter transition
    float attenuation;      										// Stop-band attenuation [dB]
    float mu;         												// Fractional timing offset
    uint32_t num_taps;												// Estimated filter length
    float *taps;													// Filter coefficients
	complex float *input, *output;									// Input, output of filter (probably not needed)
};

/* A struct to store resampler (resamp) object options (for decimation). */
struct resamp
{
    resamp_crcf resampler;											// Resample object
    uint32_t filter_delay;		    								// Filter semi-length (filter delay)
    float rate;               										// Resampling rate (output/input)
    float fc;              											// Resampling filter cutoff frequency
    float attenuation;          									// Resampling filter sidelobe suppression level
    uint32_t npfb;       											// Number of filters in bank (timing resolution)
    uint32_t input_length; 											// Number of input samples
    uint32_t output_length;											// Number of output samples
    unsigned int num_written;   									// Number of values written to buffer
};

/* A struct to store symbol synchronizer (symsync) object options . */
struct symsync
{
	symsync_crcf symsyncer;											// Symsync object
    uint32_t sampersym;    											// Samples/symbol
    uint32_t filter_delay;  	 									// Filter delay (symbols)
    float beta;  													// Filter excess bandwidth factor
    uint32_t npfb;    												// Number of polyphase filters in bank
    int32_t ftype; 													// Filter type
    unsigned int num_written;										// Number of values written to buffer
};

/* A struct to store GMSK demodulator (gmskdem) object options . */
struct demod
{
    gmskdem demod;													// GMSK demodulator object
    uint32_t sampersym;    											// Filter samples/symbol
    uint32_t filter_delay;    										// Filter delay (symbols)
    float BT;    													// Bandwidth-time product
    uint32_t output_length;											// Length of output array of 1s and 0s
};

/* Struct for storing all DSP related options, variables and data. */
typedef struct dsp
{

	/* General variables */
	uint32_t batch_counter;											// Count how many times the DSP pipeline has been invoked
	uint32_t message_counter;										// Counts number of successful decoded messages
	float downmix_freq;												// Downmixing frequency determined by the FFT

	/* Arrays to store intermediate results. */
	complex float raw_IQ[ADC_RX_BUF_SIZE];							// Store the interleaved raw ADC data
	complex float filtered_IQ[ADC_RX_BUF_SIZE];						// Store the data after anti-alias filtering
	complex float *resampled_IQ;									// Store the resampler output data
	complex float *synced_IQ;										// Store the symbol synchronizer output data
    unsigned int *demodulated_data;									// Store the demodulator output data

    /* Structs for storing options of the liquid-dsp objects used. */
    struct fft fft;													// FFT object options
    struct filter fr;												// Filter object options
    struct resamp rs;												// Resampler object options
    struct symsync ss;												// Symbol synchronizer object options
    struct demod dm;												// GMSK demodulator object options

    /* Struct for storing options of the decoder object. */
    struct decoder dr;

} *dsp_t;

/* Declare the extern struct dsp. */
extern struct dsp dsp;

/* Declare DSP Task handle */
extern TaskHandle_t DSPTaskHandle;


/************* Publicly callable functions *************/
void prvDSPInit();
void prvDSPPipeline();
void prvDSPTask( void *pvParameters );		// Task for digital signal processing of the RF signals


#endif /* CORE_INC_DSP_H_ */

