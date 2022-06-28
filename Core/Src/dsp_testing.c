/*
 * dsp_testing.c
 *
 *  Created on: 21.6.2022
 *      Author: topir
 */
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "debugRTT.h"
#include "saved_signal.h"


void prvDSPTestingTask( void *pvParameters ) {

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;


	int samples = 5000;  // How many samples are received and sent back. Size of alloc'd buffers in int16 s.
	int bytesPsamp = 2;  // 2 bytes per data sample (int16_t)
	int16_t specs[] = {(int16_t)samples, (int16_t)bytesPsamp};

	printf("Begin DSP testing task\n");


	int16_t allocArrayI[samples];								// Allocate memory for RTT buffer
	int16_t allocArrayQ[samples];								// Allocate memory for RTT buffer
	array2RTTbuffer(1, 1, allocArrayI, sizeof(allocArrayI));	// Configure RTT up-buffer '1'='DataOutI'
	array2RTTbuffer(1, 2, allocArrayQ, sizeof(allocArrayQ));	// Configure RTT up-buffer '2'='DataOutQ'
	int16_t allocDownArrayI[samples];								// Allocate memory for RTT buffer
	int16_t allocDownArrayQ[samples];								// Allocate memory for RTT buffer
	array2RTTbuffer(-1, 1, allocDownArrayI, sizeof(allocDownArrayI));	// Configure RTT down-buffer '1'='DataInI'
	array2RTTbuffer(-1, 2, allocDownArrayQ, sizeof(allocDownArrayQ));	// Configure RTT down-buffer '2'='DataInQ'
	printf("Alloc'd all buffers\n");


	//int16_t allocArraySpecs[2];										// Allocate memory for RTT buffer
	//array2RTTbuffer(1, 1, allocArraySpecs, sizeof(allocArraySpecs));// Configure RTT up-buffer '3'='DataOut'
	SEGGER_RTT_Write(1, &specs[0], sizeof(specs));					// Send data specs to Python script
	printf("Sent specs: %d samples per loop & %d bytes per sample\n", samples, bytesPsamp);

	/*
	 * DATA IN part

	// Allocate and configure RTT down-buffers
	int16_t allocDownArrayI[samples];								// Allocate memory for RTT buffer
	int16_t allocDownArrayQ[samples];								// Allocate memory for RTT buffer
	array2RTTbuffer(-1, 1, allocDownArrayI, sizeof(allocDownArrayI));	// Configure RTT down-buffer '1'='DataInI'
	array2RTTbuffer(-1, 2, allocDownArrayQ, sizeof(allocDownArrayQ));	// Configure RTT down-buffer '2'='DataInQ'
	*/
	// READ THE DOWN BUFFER
	int16_t readDataI[samples];
	int16_t readDataQ[samples];
	unsigned NumBytes;
	NumBytes = SEGGER_RTT_Read(1, &readDataI[0], sizeof(readDataI));
	printf("Read %d bytes from down-buff 1 = 'I':\n [", NumBytes);
	NumBytes = SEGGER_RTT_Read(2, &readDataQ[0], sizeof(readDataQ));

	if (NumBytes > 9) {

		for (int k = 0; k < 10; k++){		// Print out the individual bytes
			printf("%d ", readDataI[k]);
		}
		printf("... %d]\n", readDataI[samples-1]);
		printf("Read %d bytes from down-buff 2 = 'Q':\n [", NumBytes);
		for (int k = 0; k < 10; k++){		// Print out the individual bytes
			printf("%d ", readDataQ[k]);
		}
		printf("... %d]\n", readDataQ[samples-1]);
	}

	/*
	 * PROCESS DATA
	 */
	for (int k = 0; k < sizeof(readDataI)/bytesPsamp; k++){
		readDataI[k] = 2 * readDataI[k];
		readDataQ[k] = 4 * readDataQ[k];
	}


	/*
	 * DATA OUT part

	int16_t allocArrayI[samples];								// Allocate memory for RTT buffer
	int16_t allocArrayQ[samples];								// Allocate memory for RTT buffer
	array2RTTbuffer(1, 1, allocArrayI, sizeof(allocArrayI));	// Configure RTT up-buffer '1'='DataOutI'
	array2RTTbuffer(1, 2, allocArrayQ, sizeof(allocArrayQ));	// Configure RTT up-buffer '2'='DataOutQ'
	*/
	/*
	 * // Add 10k _SAVED_ signal points to test array ; OBSOLETE as data samples come through RTT now
	int16_t testArr[samples];
	// Save test data to I up-buffer '1'
	for (int k = 0; k<10000; k++){
		testArr[k] = savedIdata[k];
	}
	for (int k = 0; k<10000; k++){
		testArr[k] = savedQdata[k];
	}
	*/

	// WRITE DATA to up-buffers
	SEGGER_RTT_Write(1, &readDataI[0], sizeof(readDataI));	// Write I data to up-buffer '1' = I
	SEGGER_RTT_Write(2, &readDataQ[0], sizeof(readDataQ));	// Write Q data to up-buffer '2' = Q


	for( ;; ){
		printf("Did you read it yet?\n");
		vTaskDelay(1000);
	}

}

