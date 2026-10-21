void cw_rx(int *samples, int count);
float cw_tx_get_sample();
void cw_init();
void cw_abort();
void cw_tx(char *message, int freq);
void cw_poll(int bytes_available, int tx_is_on);

// In the text the keyer sends (the TEXT field): the speed codes, each
// changing the speed of what follows by CW_SPEED_STEP WPM, within
// CW_WPM_MIN to CW_WPM_MAX.
#define CW_SPEED_DOWN '['
#define CW_SPEED_UP ']'
#define CW_SPEED_STEP 5
#define CW_WPM_MIN 1
#define CW_WPM_MAX 60

// The GTK thread: stops the text being sent where it is - the element in
// progress completes - and goes back to WPM. The caller clears the TEXT
// field (cw_text_clear(), sbitx_gtk.c).
void cw_stop_text(void);

// How many times the key or paddle has closed, as cw_poll() sees it.
unsigned cw_key_press_count(void);
float cw_next_sample();

// added to support zerobeat display of cw decoder status
int cw_get_max_bin_highlight_index(void);

#define N_BINS 128
#define INIT_TONE 600
#define SAMPLING_FREQ 12000
#define INIT_WPM 20 
