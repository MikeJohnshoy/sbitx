#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <linux/types.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <stdbool.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <ctype.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "sound.h"
#include "modem_ft8.h"
#include "modem_cw.h"

typedef float float32_t;

//when running
#define QSO_STATE_ZOMBIE 0

#define QSO_STATE_CQ_CALLING 6
#define QSO_STATE_CQ_REPORT 2
#define QSO_STATE_CQ_RRR 4

//when search & pounding
#define QSO_STATE_REPLY_GRID 1
#define QSO_STATE_REPLY_REPORT 3

#define QSO_STATE_73 5   //we are done anyway

int qso_state = QSO_STATE_ZOMBIE;

/*
	This file dispatches to the modems sbitx implements itself: FT8 and FT4
	(modem_ft8.c) and CW (modem_cw.c).

	There are four functions called to implement almost all the digital modes
	1. There is the modem_init() that is used to initialize all the different modems.
	2. The modem_poll() is called about 10 times a second to check if any transmit/receiver
		 changeover is needed, etc.
	3. On receive, each time a block of samples is received, modem_rx() is called and
		 it despatches the block of samples to the currently selected modem.
		 The demodulators call write_console() to call the routines to display the decoded text.
	4. During transmit, modem_next_sample() is repeatedly called by the sdr to accumulate
		 samples. In turn the sample generation routines call get_tx_data_byte() to read the next
		 text/ascii byte to encode.

	Other digital modes are run by external programs, in DIGI mode, through
	the audio loopback or TCI.
*/


static int current_mode = -1;

void modem_rx(int mode, int32_t *samples, int count){
	switch(mode){
	case MODE_FT4:
	case MODE_FT8:
		ft8_rx(samples, count);
		break;
	case MODE_CW:
	case MODE_CWR:
		cw_rx(samples, count);
		break;
	}
}

void modem_init(){
	cw_init();
	ft8_init();
}


//this called routinely to check if we should start/stop the transmitting
//each mode has its peculiarities, like the ft8 will start only on 15th second boundary

void modem_poll(int mode){
	int tx_is_on = is_in_tx();

	int bytes_available = get_tx_data_length();

	if (current_mode != mode){
		current_mode = mode;

		//clear the text buffer
		abort_tx();

		if (current_mode == MODE_CW || current_mode == MODE_CWR)
			cw_init();
	}

	switch(mode){
	case MODE_FT4:
	case MODE_FT8:
		ft8_poll(tx_is_on);
	break;
	case MODE_CW:
	case MODE_CWR:
		cw_poll(bytes_available, tx_is_on);
	break;
	}
}

float modem_next_sample(int mode){
	float sample=0;

	switch(mode){
		// the ft8 samples are generated at 12ksps, we need to feed the
		// sdr with 96 ksps (eight times as much)
	case MODE_FT4:
	case MODE_FT8:
		sample = ft8_next_sample();
		break;
	case MODE_CW:
	case MODE_CWR:
		sample = cw_tx_get_sample();
		break;
	}
	return sample;
}


void modem_abort(bool terminate_qso){
	char c;

	//flush the buffer
	while(get_tx_data_byte(&c))
		NULL;

	switch(current_mode){
	case MODE_FT4:
	case MODE_FT8:
		ft8_abort(terminate_qso);
		break;
	case MODE_CW:
	case MODE_CWR:
		cw_abort();
		break;
	}
}
