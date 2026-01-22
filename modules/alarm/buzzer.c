#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "string.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config->alarm_level > BUZZER_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->stream = config->stream;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void BuzzerTask()
{
    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            if(buzz->stream == USE_OCTAVE)
            {
                switch (buzz->octave)
                {
                case OCTAVE_1:
                    PWMSetPeriod(buzzer, (float)1 / DoFreq);
                    break;
                case OCTAVE_2:
                    PWMSetPeriod(buzzer, (float)1 / ReFreq);
                    break;
                case OCTAVE_3:
                    PWMSetPeriod(buzzer, (float)1 / MiFreq);
                    break;
                case OCTAVE_4:
                    PWMSetPeriod(buzzer, (float)1 / FaFreq);
                    break;
                case OCTAVE_5:
                    PWMSetPeriod(buzzer, (float)1 / SoFreq);
                    break;
                case OCTAVE_6:
                    PWMSetPeriod(buzzer, (float)1 / LaFreq);
                    break;
                case OCTAVE_7:
                    PWMSetPeriod(buzzer, (float)1 / SiFreq);
                    break;
                case OCTAVE_8:
                    PWMSetPeriod(buzzer, (float)1 / DoHighFreq);
                    break;
                default:
                    break;
                }
            }
            if(buzz->octave == USE_STREAM)
            {
                switch (buzz->stream)
                {
                case STREAM_INIT:
                    PWMSetPeriod(buzzer, (float)1 / DoFreq);
                    DWT_Delay(0.2);
                    PWMSetPeriod(buzzer, (float)1 / MiFreq);
                    DWT_Delay(0.2);
                    PWMSetPeriod(buzzer, (float)1 / SoFreq);
                    DWT_Delay(0.2);
                    AlarmSetStatus(buzz,ALARM_OFF);
                    break;
                case STREAM_ERROR:
                    PWMSetPeriod(buzzer, (float)1 / DoHighFreq);
                    DWT_Delay(0.1);
                    PWMSetPeriod(buzzer, (float)1 / SiFreq);
                    DWT_Delay(0.1);
                    PWMSetPeriod(buzzer, (float)1 / DoHighFreq);
                    DWT_Delay(0.1);
                    PWMSetPeriod(buzzer, (float)1 / SiFreq);
                    DWT_Delay(0.1);
                    // AlarmSetStatus(buzz,ALARM_OFF);
                    break;
                default:
                    break;
                }
            }
        }
    }
}
