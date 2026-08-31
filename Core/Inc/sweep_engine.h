#ifndef __SWEEP_ENGINE_H
#define __SWEEP_ENGINE_H

#include "main.h"

/* ---- Sweep configuration ---- */
#define SWEEP_NUM_POINTS    205          /* matches display X-axis pixels  */
#define SWEEP_FREQ_MIN      10.0f       /* Hz  – display left edge        */
#define SWEEP_FREQ_MAX      100000.0f   /* Hz  – display right edge       */

#define SINE_SAMPLES        64          /* DAC points/period (low-freq)   */
#define SINE_SAMPLES_HF     16          /* DAC points/period (high-freq)  */
#define FREQ_TIER_THRESHOLD 25000.0f    /* Hz – tier switch boundary      */

#define ADC_CAPTURE_PERIODS 4
#define ADC_BUF_SIZE        (SINE_SAMPLES * ADC_CAPTURE_PERIODS) /* 256    */

#define DAC_AMPLITUDE       1800        /* peak counts (0-4095 range)     */
#define DAC_OFFSET          2048        /* mid-scale = 1.65 V             */

#define SYSTEM_CLOCK_HZ     170000000UL

typedef struct {
    float freq_hz[SWEEP_NUM_POINTS];
    float gain_db[SWEEP_NUM_POINTS];
    float phase_deg[SWEEP_NUM_POINTS];
    int   num_points;

    /* Analysis results */
    float gain_1k_db;
    float phase_1k_deg;
    float passband_gain_db;
    float cutoff_hz;
    float q_value;
    float f0_hz;
    char  filter_type[16];
} SweepResult;

void Sweep_Init(void);
void Sweep_Run(void);

#endif
