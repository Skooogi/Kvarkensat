/*
 * dsp.c
 *
 * This file contains the digital signal processing functionality of the payload.
 *
 * To Kasper and Topi: I'm sorry. Have a nice 5 weeks :D
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <liquid.h>  //Added this, but why wasn't it here? -Topi
#include "dsp.h"
#include "buffer.h"
#include "saved_signal.h"
#include "SEGGER_RTT.h"
#include "debugRTT.h"


struct dsp dsp;																															// Define the dsp struct


void prvDSPInit()
{

	/*
    // options
    unsigned int n=16;  // input data size
    int flags=0;        // FFT flags (typically ignored)

    // allocated memory arrays
    float complex * x = (float complex*) malloc(n * sizeof(float complex));
    float complex * y = (float complex*) malloc(2*n * sizeof(float complex));

    // create FFT plan
    //fftplan q = fft_create_plan(n, x, y, LIQUID_FFT_FORWARD, flags);

    // initialize input
	unsigned int i;
	for (i = 0; i < n; i++) {
		x[i] = (float)i - _Complex_I*(float)i;
	}
	// create fft plans
	fftplan pf = fft_create_plan(n, x, y, LIQUID_FFT_FORWARD,  flags);

	// print fft plans
	fft_print_plan(pf);
    // execute FFT (repeat as necessary)
    fft_execute(pf);

    // destroy FFT plan and free memory arrays
    fft_destroy_plan(pf);
    free(x);
    free(y);
    */

	/* General init */
	dsp.batch_counter = 0;																												// Reset the batch counter (how many times DSP has been requested)
	dsp.message_counter = 0;																											// Reset successful message counter
	dsp.downmix_freq = 0;																												// Reset the downmix frequency

	/* fft init */
	dsp.fft.size = 0x100;  																												// == 256, input data size
	dsp.fft.flags = 0;        																											// FFT flags (typically ignored)
	dsp.fft.fft = fft_create_plan(dsp.fft.size, dsp.raw_IQ, dsp.fft.fft_buf, LIQUID_FFT_FORWARD, dsp.fft.flags);						// Create FFT plan

    /* filter init TODO: Tweak the values to get good enough filtering with the shortest possible filter */
    dsp.fr.fc = DECIMATION_RATIO;		         																						// Filter cutoff frequency TODO: Verify the value makes sense for our decimation factor. Should be 0 <= fc <= 0.5 (normalized frequencies).
    dsp.fr.ft = 0.05f;         																											// Filter transition band width	TODO: Wider bandwidth decreases filter length
    dsp.fr.attenuation = 60.0f;   																										// Stop-band attenuation [dB] TODO: Less attenuation decreases filter length
    dsp.fr.mu = 0.0f;          																											// Fractional timing offset TODO: Don't know what this does
    dsp.fr.num_taps = estimate_req_filter_len( dsp.fr.ft, dsp.fr.attenuation );															// Estimate filter length
    dsp.fr.taps = malloc( dsp.fr.num_taps * sizeof(float) );																			// Allocate memory for the filter coefficients based on estimated length
    //liquid_firdes_kaiser( dsp.fr.num_taps, dsp.fr.fc, dsp.fr.attenuation, dsp.fr.mu, dsp.fr.taps );										// Create filter taps and store them and their length
    dsp.fr.filter = firfilt_crcf_create( dsp.fr.taps, dsp.fr.num_taps );																// Create filter object

    /* resamp init */
    dsp.rs.filter_delay = 13;    																										// Filter semi-length (filter delay) TODO: Value just from example
    dsp.rs.rate = DECIMATION_RATIO;               																						// Resampling rate (output/input). We want to decimate from 288 kHz to 28.8 kHz.
    dsp.rs.fc = 0.2f;              																										// Resampling filter cutoff frequency TODO: Just a preliminary value
    dsp.rs.attenuation = -60.0f;          																								// Resampling filter sidelobe suppression level
    dsp.rs.npfb = 32;       																											// Number of filters in bank (more filters == better timing resolution) TODO: Value just from example
    dsp.rs.input_length = ADC_RX_BUF_SIZE + dsp.rs.filter_delay;																		// Total length of resampler input
    dsp.rs.output_length = ceilf( dsp.rs.rate * dsp.rs.input_length );																	// Resampler output length ("zero padded" to incorporate filter delay). According to docs and examples, the output length should never exceed this.
    dsp.resampled_IQ = malloc( dsp.rs.output_length * sizeof(complex float) );															// Allocate memory for the resampled data based on estimated length
    dsp.rs.num_written = 0;   																											// Number of values written by the resampler to it output
    dsp.rs.resampler = resamp_crcf_create( dsp.rs.rate, dsp.rs.filter_delay, dsp.rs.fc, dsp.rs.attenuation, dsp.rs.npfb );				// Create resamp object

	/* symsync init */
    dsp.ss.sampersym = SAMPLES_PER_SYMBOL*dsp.rs.rate;     																				// Samples/symbol. We have decimated our datarate to 28.8 kHz, which is 3 times the AIS baudrate 9600. Symsync decimates this to 1.
    dsp.ss.filter_delay = 3;     																										// Filter delay (symbols) TODO: Ensure this value makes sense
    dsp.ss.beta = 0.3f;  																												// Filter excess bandwidth factor TODO: What does it do?
    dsp.ss.npfb  = 32;    																												// Number of polyphase filters in bank (more filters == better timing resolution) TODO: Value just from example
    dsp.ss.ftype = LIQUID_FIRFILT_GMSKRX; 																								// Filter type (Chose GMSKRX type filter since it would seem to make sense in our application)
    dsp.synced_IQ = malloc( ceilf( dsp.rs.output_length*sizeof(complex float)/dsp.ss.sampersym ) );										// Same length as dsp.resampled_IQ as the length of
    dsp.ss.num_written = 0;   																											// Number of values written to buffer
    dsp.ss.symsyncer = symsync_crcf_create_rnyquist( dsp.ss.ftype, dsp.ss.sampersym, dsp.ss.filter_delay, dsp.ss.beta, dsp.ss.npfb );	// Create symbol synchronizer

    /* demod init */
    dsp.dm.sampersym = 1;    																											// Samples/symbol after symsync is 1.
    dsp.dm.filter_delay = dsp.ss.filter_delay;    																						// Filter delay (symbols)
    dsp.dm.BT = 0.25f;    																												// Bandwidth-time product
    dsp.dm.output_length = ceil(dsp.rs.output_length/(float)SAMPLES_PER_SYMBOL) + AIS_MAX_PAYLOAD_BITS;									// The output length is
    dsp.demodulated_data = malloc( dsp.dm.output_length * sizeof(unsigned int) );														// Demodulation output is stored with offset of maximum AIS message lenght
    dsp.dm.demod = gmskdem_create( dsp.dm.sampersym, dsp.dm.filter_delay, dsp.dm.BT );													// Create demod object

    /* decoder init (decoder is not a liquid-dsp object) */
    prvDecoderInit();																													// Initialize the decoder

    /* resultbuf init (resultbuf is not a liquid-dsp object) */
    prvBufferInit();

}


/* Function to remove the mean value of the elements of an array from each of its elements */
static void prvSubtractMean()
{
	float real_mean;
	float imag_mean;
	for (uint32_t i=0; i<ADC_RX_BUF_SIZE; i++)
	{
		real_mean = crealf(dsp.raw_IQ[i]);								// mean of real parts
		imag_mean = cimagf(dsp.raw_IQ[i]);								// mean of imaginary parts
	}
	for(uint32_t i=0; i< ADC_RX_BUF_SIZE; i++) {
		dsp.raw_IQ[i] -= (real_mean + imag_mean*I);						// subtract complex mean from data
	}
}


/* Function taking care of the complete DSP pipeline from raw data to a decoded signal
 *
 * The pipeline has a following outline:
 *
 * 1. 	Interleave I & Q data		--------- I.e. combine the I and Q signals to one single complex IQ array.
 * 2.	Remove DC offset			--------- Subtract the mean from the signal.
 * 3.	Detect peak frequency		--------- Detect whether data contains signal @ +25 kHz or -25 kHz.
 * 4.	Downmix						--------- +25 kHz or -25 kHz to 0 Hz.
 * 5.	Lowpass filter				--------- Use firfilt object (liquid-dsp) for anti-aliasing, i.e. remove out-of-band interference.
 * 6.	Decimation					--------- Use resamp object (liquid-dsp) to decimate excessive samples. Resamp does some filtering, but per docs: "For synchronization of digital receivers, it is always good practice to precede the resampler with an anti-aliasing filter to remove out-of-band interference."
 * 7. 	Matched filter				----,
 * 8. 	Coarse freq sync				|---- Use symsync object (liquid-dsp). It removes the frequency and timing (phase) offsets of the signal and decimates to one sample/symbol
 * 9. 	Time sync						|---- Nice visualization of what is does in the docs: https://liquidsdr.org/doc/symsync/
 * 10. 	Fine freq sync				----'
 * 11. 	Demodulation				--------- Use gmksdem object (liquid-dsp). Converts the complex signal to a real signal of 0s and 1s.
 * 12. 	Frame detect				--------- Detect preamble, start flag, and end flag. Extract data packet from between the start and end flags.
 * 13.	Extract/decode packet		--------- Extract and destuff packet. Pack array of "bits" to actual binary (bytes).
 * 14. 	--> Output					--------- Push result to output buffer.
 */
void prvDSPPipeline()
{

	/* Interleave the I and Q signals to one single complex IQ array.
	for(uint32_t i=0; i<ADC_RX_BUF_SIZE; i++)
	{*/
		/* ADC dualmode sampling stores both I and Q samples in same buffer as single 32 bit value (2*16 bits) so they have to be separated
		// TODO: Check the endiannes correct bytes are taken
		int16_t I_value = adcIQ.data[i] & 0x00FF;
		int16_t Q_value = (adcIQ.data[i] >> 16) & 0x00FF;
		dsp.raw_IQ[i] = I_value + Q_value*I;
	}*/

	/* Remove DC spike from the data. */
	prvSubtractMean( dsp.raw_IQ, ADC_RX_BUF_SIZE );

	/* Channel detection. Try to detect data on either A or B channel only if no promising message is currently being decoded. If decoding is in progress, don't change the downmix frequency to ensure staying on the same channel for the whole message. */
	if ( !dsp.dr.decoding_in_progress )
	{
		fft_execute( dsp.fft.fft );																					// Start with FFT
		for ( uint32_t i=0; i<FFT_SIZE*2; i++ )
		{
			dsp.fft.mag_buf[i] = sqrt( dsp.fft.fft_buf[i] * conj(dsp.fft.fft_buf[i]) ); 							// Compute the complex magnitudes of the FFT
		}
		dsp.fft.max_mag = 0;
		dsp.fft.max_mag_idx = 0;
		for ( uint32_t i=0; i<FFT_SIZE*2; i++ )
		{
			if ( dsp.fft.mag_buf[i] > dsp.fft.max_mag )																// Check if the current magnitude is larger than the previously largest
			{
				dsp.fft.max_mag = dsp.fft.mag_buf[i];																// Update largest magnitude
				dsp.fft.max_mag_idx = i;																			// Update largest magnitude index
			}
		}
		dsp.downmix_freq = (dsp.fft.max_mag_idx - FFT_SIZE) * ADC_SAMPLERATE / FFT_SIZE;							// Calculate peak frequency (source: https://stackoverflow.com/questions/4364823/how-do-i-obtain-the-frequencies-of-each-value-in-an-fftd) TODO: Check sign

		// Peak expected around 25 kHz or -25 kHz, check if downmix_freq has valid value
		if( (dsp.downmix_freq > 30000 && dsp.downmix_freq < 20000) || (dsp.downmix_freq < -30000 && dsp.downmix_freq > -20000) )	// If true, mix frequency far off --> no AIS data
		{
			return;																									// Return from function to wait for new ADC data
		}
	}

	/* Downmix from +25 kHz or -25 kHz to 0 Hz. */
	for( uint16_t i = 0; i<ADC_RX_BUF_SIZE; i++ )
	{
		float t = (1.0*i) / (1.0*ADC_SAMPLERATE);													// Time of sample
		dsp.raw_IQ[i] = (complex float) (dsp.raw_IQ[i] * cexp(2*I*M_PI*dsp.downmix_freq*t));		// Downmix to 0 Hz by multiplying the time domain sample with a complex exponential (sine wave) that has the mix frequency.
	}

	/* Lowpass filter for anti-aliasing */
    for ( uint32_t i=0; i<ADC_RX_BUF_SIZE; i++ )
    {
        firfilt_crcf_push( dsp.fr.filter, dsp.raw_IQ[i] );    										// Push filter input sample to the internal buffer of the filter
        firfilt_crcf_execute( dsp.fr.filter, &(dsp.filtered_IQ[i]) ); 								// Compute output
    }

    /* Resample (decimate) (Docs: "For synchronization of digital receivers, it is always good practice to precede the resampler with an anti-aliasing filter to remove out-of-band interference.") */
    resamp_crcf_execute_block( dsp.rs.resampler, dsp.filtered_IQ, dsp.rs.input_length, dsp.resampled_IQ, &(dsp.rs.num_written) );

    /* Time and frequency synchronization */
    symsync_crcf_execute( dsp.ss.symsyncer, dsp.resampled_IQ, dsp.rs.output_length, dsp.synced_IQ, &(dsp.ss.num_written) );

    /* Demodulation */
    uint32_t total_samples = dsp.dm.sampersym * dsp.rs.output_length;
    for ( uint32_t i; i<total_samples; i++ )
    {
    	gmskdem_demodulate( dsp.dm.demod, dsp.synced_IQ, &(dsp.demodulated_data[i + AIS_MAX_PACKAGE_LENGTH*dsp.dm.sampersym]) );	// Cannot perform as "block", only for a sample at a time
    }

    /* Frame detection: Look for preamble, start flag, and end flag. Determine payload length and destuff payload data. */
	for ( uint32_t i=0; i<dsp.dm.output_length; i++ )
	{
		if ( !dsp.dr.preamble_found )														// If preamble has not been found, look for that
		{
			prvDetectPreamble( dsp.demodulated_data[i] );
		}
		else if ( !dsp.dr.startflag_found )													// If preamble has been found but the start flag has not been found, look for that
		{
			prvDetectStartFlag( dsp.demodulated_data[i] );
		}
		else if ( !dsp.dr.endflag_found )													// If preamble and start flag have been found but the end flag has not been found, look for that and extract the payload and corresponding CRC if end flag is found
		{
			prvDetectEndFlag( dsp.demodulated_data[i] );
		}
		else if ( dsp.dr.endflag_found )													// If all three flags are found, decode the extracted payload
		{
			prvPayloadAndCRCDecode();														// Custom decoder for the payload and its CRC
			prvPayloadTo8Bit();																// Bitshift the 1s and 0s in the payload array to 8 bit ASCII
			prvPayloadTo6Bit();																// Bitshift the 1s and 0s in the payload array to stuff the 6 bit chars into bytes without padding
			prvExtractCRC();																// Bitshift the 1s and 0s in the CRC-16 array to bytes

			if ( prvCheckPayloadCRC() )														// Check the CRC of the payload to make sure the payload has not been corrupted
			{
				prvBufferPushN( dsp.dr.ascii_message, dsp.dr.ascii_message_length );		// Push the message to the result buffer
				dsp.message_counter++;														// Successful message counter goes brrrr
				prvBufferPushN( &(resultbuf.eom), 1 );										// Push the end of message character to the result buffer to separate this message from the next ones
			}
			prvDecoderReset();																// Reset the state of the decoder to start looking for new messages
		}
	}
}


/* DSP Task responsible for the complete DSP pipeline */
void prvDSPTask( void *pvParameters )
{
	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	/*
	 * RTT debugger brick
	 */
	int samples = ADC_RX_BUF_SIZE/2;  	// How many samples are received and sent back. Size of alloc'd buffers in int16 s.
	int bytesPsamp = sizeof(int16_t); 	// bytes per SENT data sample (2 for int16_t), (4 for float), (8 for complex float)
	int sleeptime = 1000;

	//initRTTbuffers(samples, bytesPsamp, sleeptime);
	//void * allocArrayI = calloc(samples+1, bytesPsamp);				// Allocate memory for RTT buffer
	//void * allocArrayQ = calloc(samples+1, bytesPsamp);
	int16_t allocArrayI[samples+1];								// Allocate memory for RTT buffer
	int16_t allocArrayQ[samples+1];								// Allocate memory for RTT buffer
	array2RTTbuffer(1, 1, allocArrayI, sizeof(allocArrayI));	// Configure RTT up-buffer '1'='DataOutI'
	array2RTTbuffer(1, 2, allocArrayQ, sizeof(allocArrayQ));	// Configure RTT up-buffer '2'='DataOutQ
	int16_t allocDownArrayI[samples+1];								// Allocate memory for RTT buffer
	int16_t allocDownArrayQ[samples+1];								// Allocate memory for RTT buffer
	array2RTTbuffer(-1, 1, allocDownArrayI, sizeof(allocDownArrayI));	// Configure RTT down-buffer '1'='DataInI'
	array2RTTbuffer(-1, 2, allocDownArrayQ, sizeof(allocDownArrayQ));	// Configure RTT down-buffer '2'='DataInQ'

	// Send communication specs over RTT to python scripts
	int16_t allocArraySpecs[6];									// Allocate memory for RTT buffer
	array2RTTbuffer(1, 3, allocArraySpecs, sizeof(allocArraySpecs));// Configure RTT up-buffer '3'='DataOut'
	int16_t specs[6] = {samples, bytesPsamp, sleeptime, samples, bytesPsamp, sleeptime};
	SEGGER_RTT_Write(3, &specs[0], sizeof(specs));

	// Allocate buffers for read data
	int16_t readDataI[samples];
	int16_t readDataQ[samples];
	uint32_t numBytesI, numBytesQ;
	dsp.batch_counter = 0;

	printf("--- Starting DSP loop (data from Python) ---\n\n");

	for( ;; ){
		/*
		 * DATA IN part
		 */
		numBytesI = SEGGER_RTT_Read(1, &readDataI[0], sizeof(readDataI));
		printf("Read %d bytes from down-buff 1 = 'I':\n", (int)numBytesI);
		numBytesQ = SEGGER_RTT_Read(2, &readDataQ[0], sizeof(readDataQ));

		// CHANGE to just 'if (numBytes)' when comfortable and no prints are needed
		if (numBytesI > 9) {
			// Print out a small amount of readings
			printf(" [");
			for (int k = 0; k < 10; k++){
				printf("%d ", readDataI[k]);
			}
			printf("... %d]\n", readDataI[numBytesI/2 - 1]);
			printf("Read %d bytes from down-buff 2 = 'Q':\n [", (int)numBytesQ);
			for (int k = 0; k < 10; k++){
				printf("%d ", readDataQ[k]);
			}
			printf("... %d]\n", readDataQ[numBytesQ/2 - 1]);

			//TEST DATA FROM SAVED SIGNAL
			//for(int i = 0; i < ADC_RX_BUF_SIZE/2; ++i) {
					// adcIQ.data[i] = ((uint32_t)readDataQ[i] << 16) | readDataI[i];
			//		dsp.raw_IQ[i] = (float)readDataI[i] + I*(float)readDataQ[i];
			//}

			/*
			 * PROCESS DATA
			 */
			/* Do DSP */
			//prvDSPPipeline();
			dsp.batch_counter++;			// Another brick in the wall

			/*
			 * DATA OUT part
			 */
			// WRITE DATA to up-buffers

			// MODIFY TO SEND REAL AND IMAG PARTS or SOMETHING
			//numBytesI = SEGGER_RTT_Write(1, &dsp.raw_IQ[0], samples*bytesPsamp);
			numBytesI = SEGGER_RTT_Write(1, &readDataI[0], numBytesI);	// Write I data to up-buffer '1' = I
			numBytesQ = SEGGER_RTT_Write(2, &readDataQ[0], numBytesQ);	// Write Q data to up-buffer '2' = Q
			//if (numBytesI > 0) printf("Send IQ data back (%d bytes) \n", numBytesI);
			if (numBytesI > 0) printf("Send I data back (%d bytes), ", (int)numBytesI);
			if (numBytesQ > 0) printf("Send Q data back (%d bytes) \n", (int)numBytesQ);
			printf("Batch num: %d ", (int)dsp.batch_counter);

		}
		else {
			dsp.batch_counter = 0;
			printf("Nothing read; ");
		}

		// WAIT A LITTLE FOR NEXT LOOP (some secs)
		printf("Waiting %d secs\n\n", (sleeptime/1000));
		vTaskDelay(sleeptime);
	}
	/*
	 * ORIGINAL TASK LOOP
	 */
	//for( ;; )
	//{
		/* The ADCTask gives a notification when data requires processing. This task remains blocked until
		 * a notification is received. Upon receiving the notification, clear the notification and start DSP. */
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


		/* Do DSP */
		//prvDSPPipeline();
		//dsp.batch_counter++;			// Another brick in the wall

		/* TODO: Make sure this task goes back to blocked state after it has performed the DSP.
		 * Should ulTaskNotifyTake(pdTRUE, portMAX_DELAY) actually be called here instead of before DSP? */
	//}
}



