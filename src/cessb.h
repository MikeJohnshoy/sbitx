// CESSB (Controlled Envelope Single Sideband) Processing Implementation
// Based on W9GR (David Hershberger) CESSB algorithm

#include <stdint.h>

// CESSB processing states
#define CESSB_DISABLED 0
#define CESSB_ENABLED 1

// CESSB Parameters
#define CESSB_PRE_GAIN 20.0f            // allow float values to get up over 1.0f
                                        // (experimentally determined on sBitx)
#define CESSB_CLIP_LEVEL 1.05f          // hard-clip threshold
#define CESSB_ENVELOPE_LIMIT 0.90f      // final envelope limit
#define CESSB_AUDIO_LOW_CUTOFF 300.0f   // audio bandpass low cutoff (Hz)
#define CESSB_AUDIO_HIGH_CUTOFF 3000.0f // audio bandpass high cutoff (Hz)
#define CESSB_SAMPLE_RATE 96000.0f      // audio sample rate

// Hilbert transform filter length (must be odd)
#define HILBERT_TAPS 127

// Overshoot control filter - key CESSB innovation
#define OVERSHOOT_FILTER_TAPS 65

// Post-limiter LPF - multi-stage biquad for clean rolloff
#define POST_LPF_BIQUAD_STAGES 3

// Look-ahead limiter configuration
#define LOOKAHEAD_MAX_SAMPLES 1024      // maximum look-ahead buffer (< 1 block)
#define LOOKAHEAD_DEFAULT_SAMPLES 192   // default ~2ms at 96kHz
#define LOOKAHEAD_DEFAULT_ATTACK_MS 0.5f
#define LOOKAHEAD_DEFAULT_RELEASE_MS 50.0f

// Biquad filter section state
typedef struct {
    float x1, x2;   // input delay elements
    float y1, y2;   // output delay elements
} biquad_state_t;

// Look-ahead limiter state
typedef struct {
    float delay[LOOKAHEAD_MAX_SAMPLES];     // audio delay buffer
    float envelope[LOOKAHEAD_MAX_SAMPLES];  // envelope delay buffer
    int write_index;
    int lookahead_samples;
    
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

    // STAGE 1: Hilbert transform delay lines
    float hilbert_delay[HILBERT_TAPS];
    int hilbert_index;
    float delay_line[(HILBERT_TAPS / 2) + 1];
    int delay_index;

    // Overshoot control filter delay line
    float overshoot_delay[OVERSHOOT_FILTER_TAPS];
    int overshoot_index;

    // STAGE 2: Hilbert transform delay lines
    float hilbert2_delay[HILBERT_TAPS];
    int hilbert2_index;
    float delay2_line[(HILBERT_TAPS / 2) + 1];
    int delay2_index;

    // Look-ahead limiter
    lookahead_limiter_t lookahead;

    // Post-limiter LPF state
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
void cessb_set_pre_gain(cessb_state_t *state, float gain);
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
void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db, float *avg_power_gain_db);
void cessb_reset_stats(cessb_state_t *state);

















