// CESSB (Controlled Envelope Single Sideband) Processing Implementation

#include <stdint.h>

// CESSB processing states
#define CESSB_DISABLED 0
#define CESSB_ENABLED 1

// CESSB Parameters
#define CESSB_CLIP_LEVEL 0.85f          // initial soft-clip threshold
#define CESSB_ENVELOPE_LIMIT 1.0f       // final envelope limit
#define CESSB_POST_LPF_CUTOFF 3400.0f   // post-limiter LPF cutoff (Hz)

// Hilbert transform filter length (must be odd)
#define HILBERT_TAPS 127

// structure to hold CESSB state
typedef struct {
  int enabled;
  float clip_level;
  float envelope_limit;

  // Hilbert transform delay line
  float hilbert_delay[HILBERT_TAPS];
  int hilbert_index;

  // delay line for group delay compensation
  float delay_line[HILBERT_TAPS/2 + 1];
  int delay_index;

  // Post-processing LPF state
  float post_lpf_state;

  // statistics for monitoring
  float peak_input;
  float peak_output;
  float average_power_in;
  float average_power_out;
} cessb_state_t;

extern int cessb_enabled;
extern cessb_state_t cessb_processor;

// function prototypes
void cessb_init(cessb_state_t *state);
void cessb_set_enabled(cessb_state_t *state, int enabled);
void cessb_set_clip_level(cessb_state_t *state, float level);
void cessb_set_envelope_limit(cessb_state_t *state, float limit);
int cessb_is_enabled(cessb_state_t *state);

void cessb_process(cessb_state_t *state, float *samples, int num_samples, float sample_rate);
void cessb_process_int32(cessb_state_t *state, int32_t *samples, int num_samples, float sample_rate);

void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db, float *avg_power_gain_db);
void cessb_reset_stats(cessb_state_t *state);

