#include "sweep_engine.h"
#include "hmi_driver.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------- peripheral handles (defined in main.c) ---------- */
extern ADC_HandleTypeDef  hadc1;
extern DAC_HandleTypeDef  hdac1;
extern TIM_HandleTypeDef  htim6;

/* ---------- DAC sine lookup tables ---------- */
static uint16_t dac_buf_64[SINE_SAMPLES];
static uint16_t dac_buf_16[SINE_SAMPLES_HF];

/* ---------- DFT reference tables ---------- */
static float sin_ref_64[SINE_SAMPLES];
static float cos_ref_64[SINE_SAMPLES];
static float sin_ref_16[SINE_SAMPLES_HF];
static float cos_ref_16[SINE_SAMPLES_HF];

/* ---------- ADC capture buffer ---------- */
static uint16_t adc_buf[ADC_BUF_SIZE];

/* ---------- DMA completion flag ---------- */
static volatile uint8_t adc_dma_done = 0;

/* ---------- Sweep results (static to avoid stack overflow) ---------- */
static SweepResult result;

/* ================================================================== */
/*                     Private helper functions                       */
/* ================================================================== */

/**
 * Configure TIM6 prescaler & auto-reload for desired sample rate.
 * Timer is stopped on entry and NOT restarted.
 */
static void ConfigureTimer(float freq_hz, int samples_per_period)
{
    float   target_rate = freq_hz * (float)samples_per_period;
    uint32_t total      = (uint32_t)((float)SYSTEM_CLOCK_HZ / target_rate + 0.5f);

    uint16_t psc, arr;
    if (total <= 65536U) {
        psc = 0U;
        arr = (uint16_t)(total - 1U);
    } else {
        psc = (uint16_t)(total / 65536U);
        arr = (uint16_t)(total / ((uint32_t)psc + 1U) - 1U);
    }

    HAL_TIM_Base_Stop(&htim6);

    htim6.Instance->PSC = psc;
    htim6.Instance->ARR = arr;
    htim6.Instance->CNT = 0;
    htim6.Instance->EGR = TIM_EGR_UG;                     /* load shadow regs  */
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);         /* clear spurious UIF */
}

/**
 * Measure gain (dB) and phase (deg) at a single frequency.
 *
 * DAC outputs sine → external level-shift → filter → level-shift → ADC.
 * Both level-shift circuits are purely DC offset (unity AC gain), so
 * gain = ADC_amplitude / DAC_AMPLITUDE directly in counts.
 */
static void MeasureAtFreq(float freq_hz, float *out_gain_db, float *out_phase_deg)
{
    /* ---- select tier ---- */
    int       spp;
    uint16_t *dac_buf;
    float    *s_ref, *c_ref;

    if (freq_hz <= FREQ_TIER_THRESHOLD) {
        spp     = SINE_SAMPLES;
        dac_buf = dac_buf_64;
        s_ref   = sin_ref_64;
        c_ref   = cos_ref_64;
    } else {
        spp     = SINE_SAMPLES_HF;
        dac_buf = dac_buf_16;
        s_ref   = sin_ref_16;
        c_ref   = cos_ref_16;
    }
    int capture_len = spp * ADC_CAPTURE_PERIODS;

    /* ---- configure timer for this frequency ---- */
    ConfigureTimer(freq_hz, spp);

    /* ---- start DAC DMA (circular) ---- */
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t *)dac_buf, (uint32_t)spp,
                      DAC_ALIGN_12B_R);

    /* ---- start timer ---- */
    HAL_TIM_Base_Start(&htim6);

    /* ---- wait for filter to settle (≥ 5 periods, ≥ 2 ms) ---- */
    float settle_ms = 1000.0f / freq_hz * 5.0f;
    if (settle_ms < 2.0f)   settle_ms = 2.0f;
    if (settle_ms > 500.0f) settle_ms = 500.0f;
    HAL_Delay((uint32_t)(settle_ms + 0.5f));

    /* ---- capture ADC via DMA (normal mode) ---- */
    adc_dma_done = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, (uint32_t)capture_len);

    /* wait with generous timeout */
    uint32_t timeout_ms = (uint32_t)(1000.0f / freq_hz * ADC_CAPTURE_PERIODS) + 200U;
    uint32_t t0 = HAL_GetTick();
    while (!adc_dma_done) {
        if ((HAL_GetTick() - t0) > timeout_ms) break;
    }
    HAL_ADC_Stop_DMA(&hadc1);

    /* ================================================================
     *  DFT at the fundamental frequency (k = 1)
     *
     *  X[1] = Σ x[n]·cos(2πn/N) − j·Σ x[n]·sin(2πn/N)
     *
     *  For x[n] = A·sin(2πn/N + φ)  over L = M·N samples:
     *      |X[1]| = A·L/2
     *      ∠X[1] = φ − π/2
     *
     *  DAC reference:  sin(2πn/N),  so ∠X_dac = −π/2
     *  Phase shift   = ∠X_adc − ∠X_dac = φ
     * ================================================================ */

    /* remove DC */
    float dc_sum = 0;
    for (int i = 0; i < capture_len; i++) dc_sum += (float)adc_buf[i];
    float dc = dc_sum / (float)capture_len;

    /* correlate with sin / cos at fundamental */
    float re = 0.0f, im = 0.0f;
    for (int i = 0; i < capture_len; i++) {
        float s  = (float)adc_buf[i] - dc;
        int   idx = i % spp;
        re +=  s * c_ref[idx];
        im += -s * s_ref[idx];          /* −sin for e^{−j…} */
    }

    float adc_amp   = 2.0f * sqrtf(re * re + im * im) / (float)capture_len;
    float adc_phase = atan2f(im, re);                     /* = φ − π/2        */
    float dac_phase = -(float)M_PI / 2.0f;                /* reference phase   */

    /* gain */
    float gain_lin = adc_amp / (float)DAC_AMPLITUDE;
    *out_gain_db   = 20.0f * log10f(gain_lin + 1e-10f);

    /* phase */
    float ph = adc_phase - dac_phase;                     /* = φ               */
    while (ph >  (float)M_PI) ph -= 2.0f * (float)M_PI;
    while (ph < -(float)M_PI) ph += 2.0f * (float)M_PI;
    *out_phase_deg = ph * (180.0f / (float)M_PI);

    /* for a low-pass filter phase ∈ [−180°, 0°] */
    if (*out_phase_deg > 10.0f) *out_phase_deg -= 360.0f;
}

/* ---------- linear interpolation helpers ---------- */

/** Interpolate y at x_target from monotonically increasing x[] */
static float InterpY(const float *x, const float *y, int n, float x_target)
{
    for (int i = 1; i < n; i++) {
        if (x[i] >= x_target) {
            float t = (x_target - x[i - 1]) / (x[i] - x[i - 1] + 1e-20f);
            return y[i - 1] + t * (y[i] - y[i - 1]);
        }
    }
    return y[n - 1];
}

/** Find x where y crosses target (y decreasing) */
static float InterpX(const float *x, const float *y, int n, float y_target)
{
    for (int i = 1; i < n; i++) {
        if (y[i] <= y_target && y[i - 1] > y_target) {
            float t = (y_target - y[i - 1]) / (y[i] - y[i - 1] + 1e-20f);
            return x[i - 1] + t * (x[i] - x[i - 1]);
        }
    }
    return x[n - 1];
}

/**
 * Post-sweep analysis: extract all metrics from the swept data.
 *
 *  - passband gain      = max(gain) over all points
 *  - cutoff frequency   = freq where gain = passband − 3 dB
 *  - characteristic f0  = freq where phase = −90°
 *  - Q                  = |H(f0)| / H_passband  (linear)
 *  - type               = Bessel / Butterworth / Chebyshev
 */
static void AnalyzeResults(void)
{
    int n = result.num_points;

    /* ---------- 1kHz gain & phase ---------- */
    result.gain_1k_db  = InterpY(result.freq_hz, result.gain_db,  n, 1000.0f);
    result.phase_1k_deg = InterpY(result.freq_hz, result.phase_deg, n, 1000.0f);

    /* ---------- passband gain (global max) ---------- */
    result.passband_gain_db = -200.0f;
    for (int i = 0; i < n; i++) {
        if (result.gain_db[i] > result.passband_gain_db)
            result.passband_gain_db = result.gain_db[i];
    }

    /* ---------- cutoff frequency (−3 dB from passband) ---------- */
    result.cutoff_hz = InterpX(result.freq_hz, result.gain_db,
                               n, result.passband_gain_db - 3.0f);

    /* ---------- characteristic frequency (phase = −90°) ---------- */
    result.f0_hz = InterpX(result.freq_hz, result.phase_deg, n, -90.0f);

    /* ---------- Q value ---------- */
    float gain_at_f0 = InterpY(result.freq_hz, result.gain_db, n, result.f0_hz);
    result.q_value = powf(10.0f, (gain_at_f0 - result.passband_gain_db) / 20.0f);

    /* ---------- filter type classification ----------
     *  Bessel       Q ≈ 0.577
     *  Butterworth  Q ≈ 0.707
     *  Chebyshev    Q > 0.8
     */
    if (result.q_value < 0.63f)
        strcpy(result.filter_type, "Bessel");
    else if (result.q_value < 0.80f)
        strcpy(result.filter_type, "Butterworth");
    else
        strcpy(result.filter_type, "Chebyshev");
}

/* ================================================================== */
/*                       Public API                                   */
/* ================================================================== */

void Sweep_Init(void)
{
    /* ---- generate 64-sample sine LUT & DFT references ---- */
    for (int i = 0; i < SINE_SAMPLES; i++) {
        float a = 2.0f * (float)M_PI * (float)i / (float)SINE_SAMPLES;
        dac_buf_64[i] = (uint16_t)(DAC_OFFSET + (int)(DAC_AMPLITUDE * sinf(a)));
        sin_ref_64[i] = sinf(a);
        cos_ref_64[i] = cosf(a);
    }

    /* ---- generate 16-sample sine LUT & DFT references ---- */
    for (int i = 0; i < SINE_SAMPLES_HF; i++) {
        float a = 2.0f * (float)M_PI * (float)i / (float)SINE_SAMPLES_HF;
        dac_buf_16[i] = (uint16_t)(DAC_OFFSET + (int)(DAC_AMPLITUDE * sinf(a)));
        sin_ref_16[i] = sinf(a);
        cos_ref_16[i] = cosf(a);
    }

    /* ---- ADC single-ended calibration ---- */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* ---- shorten ADC sampling time → max ~1.7 MS/s ---- */
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_1;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc1, &ch);

    memset(&result, 0, sizeof(result));
}

void Sweep_Run(void)
{
    /* ---- generate log-spaced frequency table ---- */
    result.num_points = SWEEP_NUM_POINTS;
    float log_min  = log10f(SWEEP_FREQ_MIN);
    float log_max  = log10f(SWEEP_FREQ_MAX);
    float log_step = (log_max - log_min) / (float)(SWEEP_NUM_POINTS - 1);
    for (int i = 0; i < SWEEP_NUM_POINTS; i++) {
        result.freq_hz[i] = powf(10.0f, log_min + log_step * (float)i);
    }



    /* ---- sweep every frequency ---- */
    for (int i = 0; i < SWEEP_NUM_POINTS; i++) {
        if (!hmi_is_measuring) goto cleanup;

        MeasureAtFreq(result.freq_hz[i],
                      &result.gain_db[i],
                      &result.phase_deg[i]);

        HMI_AddWavePoint(result.gain_db[i], result.phase_deg[i]);
    }

    if (!hmi_is_measuring) goto cleanup;

    /* ---- post-sweep analysis ---- */
    AnalyzeResults();

    /* ---- push results to display (also clears measuring flag) ---- */
    HMI_UpdateMetrics(result.gain_1k_db,
                      result.phase_1k_deg,
                      result.passband_gain_db,
                      result.cutoff_hz,
                      result.q_value,
                      result.f0_hz,
                      result.filter_type);

cleanup:
    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}

/* ---- ADC DMA transfer-complete callback ---- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_dma_done = 1;
    }
}
