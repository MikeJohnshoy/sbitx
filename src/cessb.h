// CESSB (Controlled Envelope Single Sideband) Processing
// Based on W9GR (David Hershberger) CESSB algorithm

#include <stdint.h>

// add recording of input and output samples for test purpposes
#define REC_AUDIO 0                  // set to 0 in operational code
#define REC_AUDIO_SEGMENT_SECONDS 5  // per-file recording length

// CESSB processing states
#define CESSB_DISABLED 0
#define CESSB_ENABLED  1

// CESSB Parameters
#define CESSB_PRE_GAIN           30.0f   // working gain
#define CESSB_CLIP_LEVEL         0.85f   // hard-clip threshold (default)
#define CESSB_ENVELOPE_LIMIT     1.00f   // final limiter ceiling
#define CESSB_SAMPLE_RATE        96000.0f

// Hilbert transform filter length (must be odd)
#define HILBERT_TAPS             127
// Length of the Hilbert delay line used for get_delayed_sample
#define HILBERT_DELAY_LEN        ((HILBERT_TAPS / 2) + 1)

// Overshoot control filter - key CESSB innovation
#define OVERSHOOT_FILTER_TAPS    65
// Group delay for overshoot FIR (used in Stage 4 alignment)
#define OVERSHOOT_DELAY_LEN      ((OVERSHOOT_FILTER_TAPS - 1) / 2)

// Post-limiter LPF - multi-stage biquad for clean rolloff
#define POST_LPF_BIQUAD_STAGES   3

// Look-ahead limiter configuration
#define LOOKAHEAD_MAX_SAMPLES        1024
#define LOOKAHEAD_DEFAULT_SAMPLES   192
#define LOOKAHEAD_DEFAULT_ATTACK_MS 0.5f
#define LOOKAHEAD_DEFAULT_RELEASE_MS 50.0f

// Biquad filter section state
typedef struct {
    float x1, x2;   // input delay elements
    float y1, y2;   // output delay elements
} biquad_state_t;

// Vector (I/Q) look-ahead limiter state
typedef struct {
    float delay_i[LOOKAHEAD_MAX_SAMPLES];
    float delay_q[LOOKAHEAD_MAX_SAMPLES];
    float envelope[LOOKAHEAD_MAX_SAMPLES];
    int   write_index;
    int   lookahead_samples;
    float current_gain;
    float attack_coeff;
    float release_coeff;
    float peak_hold;
} lookahead_limiter_t;

// Main CESSB state structure
typedef struct {
    int enabled;
    float clip_level;
    float envelope_limit;
    float sample_rate;

    // STAGE 1: Hilbert transform delay lines (analytic pair)
    float hilbert_delay[HILBERT_TAPS];
    int   hilbert_index;
    float delay_line[HILBERT_DELAY_LEN];
    int   delay_index;

    // Overshoot control filter delay line (I and Q branches)
    float overshoot_i_delay[OVERSHOOT_FILTER_TAPS];
    float overshoot_q_delay[OVERSHOOT_FILTER_TAPS];
    int   overshoot_i_index;
    int   overshoot_q_index;

    // Second delay (for stage-4 envelope measurement) for analytic pair
    float delay2_i[OVERSHOOT_DELAY_LEN];
    float delay2_q[OVERSHOOT_DELAY_LEN];
    int   delay2_i_index;
    int   delay2_q_index;

    // Look-ahead limiter (vector)
    lookahead_limiter_t lookahead;

    // Post-limiter LPF state (cascade of biquads)
    biquad_state_t post_lpf_state[POST_LPF_BIQUAD_STAGES];

    // Statistics
    float peak_input;
    float peak_output;
    float peak_after_clip;
    float peak_after_overshoot;
    float average_power_in;
    float average_power_out;
    float min_limiter_gain;
    unsigned long sample_count;
} cessb_state_t;

// Global instance
extern int cessb_enabled;
extern cessb_state_t cessb_processor;

// Initialization and configuration
void cessb_init(cessb_state_t *state, float sample_rate);
void cessb_set_enabled(cessb_state_t *state, int enabled);
void cessb_set_clip_level(cessb_state_t *state, float level);
/* If you want a runtime pre-gain setter, implement it in cessb.c and then
   uncomment the prototype below. Otherwise remove this prototype:
   void cessb_set_pre_gain(cessb_state_t *state, float gain);
*/
void cessb_set_envelope_limit(cessb_state_t *state, float limit);
int cessb_is_enabled(cessb_state_t *state);

// Look-ahead limiter configuration
void cessb_set_lookahead_samples(cessb_state_t *state, int samples);
void cessb_set_lookahead_ms(cessb_state_t *state, float milliseconds);
void cessb_set_attack_ms(cessb_state_t *state, float attack_ms);
void cessb_set_release_ms(cessb_state_t *state, float release_ms);
int cessb_get_lookahead_samples(cessb_state_t *state);

// Main processing functions
void cessb_process(cessb_state_t *state, float *samples, int num_samples);
void cessb_process_int32(cessb_state_t *state, int32_t *samples, int num_samples);

// Statistics
void cessb_get_stats(cessb_state_t *state,
                     float *peak_reduction_db,
                     float *avg_power_gain_db,
                     float *talk_power_db);
void cessb_reset_stats(cessb_state_t *state);


