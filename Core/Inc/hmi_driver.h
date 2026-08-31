#ifndef __HMI_DRIVER_H
#define __HMI_DRIVER_H

#include "main.h"

/* 串口屏控件映射 */
#define HMI_WAVE_ID         20
#define HMI_TEXT_STATUS     "t2"
#define HMI_TEXT_Q          "t5"
#define HMI_TEXT_F0         "t6"
#define HMI_TEXT_TYPE       "t8"
#define HMI_TEXT_GAIN_1K    "t12"
#define HMI_TEXT_PHASE_1K   "t13"
#define HMI_TEXT_CUTOFF     "t14"
#define HMI_TEXT_PASSBAND   "t15"

extern volatile uint8_t hmi_is_measuring;

void HMI_Init(void);
void HMI_SetMeasureState(uint8_t is_start);
void HMI_SetStatus(const char *text);
void HMI_UpdateMetrics(float gain_1k_db,
                       float phase_1k_deg,
                       float passband_gain_db,
                       float cutoff_hz,
                       float q_val,
                       float f0_hz,
                       const char *type_str);
void HMI_AddWavePoint(float gain_db, float phase_deg);

#endif
