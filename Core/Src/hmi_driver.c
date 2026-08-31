#include "hmi_driver.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

volatile uint8_t hmi_is_measuring = 0; // 测量状态标志位
uint8_t hmi_rx_buf;                    // 串口单字节接收缓存

static void HMI_SendCmd(const char *cmd)
{
    uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF};
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 50);
    HAL_UART_Transmit(&huart1, end_cmd, 3, 50);
}

static void HMI_UpdateTextInternal(const char *obj, const char *text)
{
    char buf[64];
    (void)snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, text);
    HMI_SendCmd(buf);
}

static void HMI_SetButtonText(const char *text)
{
    HMI_UpdateTextInternal("b2", text);
}

static void HMI_RestoreDefaultView(void)
{
    HMI_UpdateTextInternal(HMI_TEXT_STATUS, "待机中...");
    HMI_UpdateTextInternal(HMI_TEXT_Q, "--");
    HMI_UpdateTextInternal(HMI_TEXT_F0, "-- Hz");
    HMI_UpdateTextInternal(HMI_TEXT_TYPE, "等待测试");
    HMI_UpdateTextInternal(HMI_TEXT_GAIN_1K, "-- dB");
    HMI_UpdateTextInternal(HMI_TEXT_PHASE_1K, "-- deg");
    HMI_UpdateTextInternal(HMI_TEXT_CUTOFF, "-- Hz");
    HMI_UpdateTextInternal(HMI_TEXT_PASSBAND, "-- dB");
}

static void HMI_ClearWave(void)
{
    char buf[24];

    (void)snprintf(buf, sizeof(buf), "cle %d,0", HMI_WAVE_ID);
    HMI_SendCmd(buf);
    (void)snprintf(buf, sizeof(buf), "cle %d,1", HMI_WAVE_ID);
    HMI_SendCmd(buf);
}

static void HMI_FormatFrequency(char *buf, size_t buf_size, float freq_hz)
{
    if (freq_hz >= 1000.0f) {
        (void)snprintf(buf, buf_size, "%.2f kHz", freq_hz / 1000.0f);
    } else {
        (void)snprintf(buf, buf_size, "%.0f Hz", freq_hz);
    }
}

void HMI_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &hmi_rx_buf, 1);
    HMI_RestoreDefaultView();
}

void HMI_SetMeasureState(uint8_t is_start)
{
    if (is_start != 0U) {
        hmi_is_measuring = 1;
        HMI_RestoreDefaultView();
        HMI_SetButtonText("停止测量");
        HMI_SetStatus("扫频测试中...");
        HMI_ClearWave();
    } else {
        hmi_is_measuring = 0;
        HMI_RestoreDefaultView();
        HMI_SetButtonText("开始测量");
    }
}

void HMI_SetStatus(const char *text)
{
    HMI_UpdateTextInternal(HMI_TEXT_STATUS, text);
}

void HMI_UpdateMetrics(float gain_1k_db,
                       float phase_1k_deg,
                       float passband_gain_db,
                       float cutoff_hz,
                       float q_val,
                       float f0_hz,
                       const char *type_str)
{
    char buf[64];

    (void)snprintf(buf, sizeof(buf), "%.2f", q_val);
    HMI_UpdateTextInternal(HMI_TEXT_Q, buf);

    HMI_FormatFrequency(buf, sizeof(buf), f0_hz);
    HMI_UpdateTextInternal(HMI_TEXT_F0, buf);

    HMI_UpdateTextInternal(HMI_TEXT_TYPE, type_str);

    (void)snprintf(buf, sizeof(buf), "%.2f dB", gain_1k_db);
    HMI_UpdateTextInternal(HMI_TEXT_GAIN_1K, buf);

    (void)snprintf(buf, sizeof(buf), "%.1f deg", phase_1k_deg);
    HMI_UpdateTextInternal(HMI_TEXT_PHASE_1K, buf);

    HMI_FormatFrequency(buf, sizeof(buf), cutoff_hz);
    HMI_UpdateTextInternal(HMI_TEXT_CUTOFF, buf);

    (void)snprintf(buf, sizeof(buf), "%.2f dB", passband_gain_db);
    HMI_UpdateTextInternal(HMI_TEXT_PASSBAND, buf);

    HMI_SetStatus("测试完成");
    HMI_SetButtonText("开始测量");
    hmi_is_measuring = 0;
}

void HMI_AddWavePoint(float gain_db, float phase_deg)
{
    char buf[32];
    int y_gain, y_phase;

    y_gain = (int)((gain_db + 40.0f) * 3.0f + 0.5f); // +0.5f是为了四舍五入
    y_phase = (int)((phase_deg + 180.0f) * 1.0f + 0.5f);

    if (y_gain < 0) {
        y_gain = 0;
    } else if (y_gain > 180) {
        y_gain = 180;
    }

    if (y_phase < 0) {
        y_phase = 0;
    } else if (y_phase > 180) {
        y_phase = 180;
    }

    (void)snprintf(buf, sizeof(buf), "add %d,0,%d", HMI_WAVE_ID, y_gain);
    HMI_SendCmd(buf);

    (void)snprintf(buf, sizeof(buf), "add %d,1,%d", HMI_WAVE_ID, y_phase);
    HMI_SendCmd(buf);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if ((hmi_rx_buf == 0x01U) && (hmi_is_measuring == 0U)) {
            hmi_is_measuring = 1;
        } else if ((hmi_rx_buf == 0x00U) && (hmi_is_measuring == 1U)) {
            hmi_is_measuring = 0;
        }

        HAL_UART_Receive_IT(&huart1, &hmi_rx_buf, 1);
    }
}
