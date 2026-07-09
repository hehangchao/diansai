/* main.c — RT-Thread (STM32F103C8T6)
 * ADC1 + DMA(环形) + TIM3 TRGO 外部触发，500 kSPS 连续采样
 * 批处理统计阈值命中，避免每样点加锁
 * ΔV(阈值增量)按“一个完整周期(6s+1.5s)后评估i<10000”逐周期自增，
 * 达标后才开始 i_yuan 标定和正常运行。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "i2c_gpio.h"
#include "i2c_ee.h"
#include "stm32f1xx_hal.h"

/* ----------------- 引脚/设备 ----------------- */
#define LED1_PIN   GET_PIN(B, 1)
#define LED5_PIN   GET_PIN(B, 5)

#define UART2_DEV_NAME  "uart2"
#define PWM4_DEV_NAME   "pwm4"
#define PWM4_CH         3
#define PWM3_DEV_NAME   "pwm3"
#define PWM3_CH         2

/* ----------------- 常量 ----------------- */
#define VREF              3.3f
#define ADC_MAX_VALUE     4095.0f

/* 自适应 ΔV 搜索策略（逐周期评估 i 是否 < 20000） */
static const float I_TARGET     = 60000.0f;    /* 一个完整周期 i 的目标上限 */
static const float DV_INIT      = 0.35f;       /* 初始 ΔV */
static const float DV_STEP_MIN  = 0.01f;       /* 最小递增步进（也用于向下调） */
static const float DV_STEP_MID  = 0.04f;
static const float DV_STEP_BIG  = 0.08f;
static const float DV_STEP_HUGE = 0.12f;
static const float DV_MARGIN    = 0.05f;       /* 与 VREF 留的最小裕量，避免阈值顶到头 */
static const float DV_MIN       = 0.01f;       /* ΔV 下界，防止过低 */

/* 连续 0 触发重新标定 ΔV（仅“正常运行”情况下） */
#define ZERO_RECAL_N  8

struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

/* ===== 采样率与缓冲长度 ===== */
#define ADC_SAMPLE_RATE_HZ   500000U
#define ADC_BUF_LEN          4096U
static __IO uint16_t s_adc_buf[ADC_BUF_LEN];

/* ----------------- HAL 句柄 ----------------- */
static ADC_HandleTypeDef     s_hadc1;
static DMA_HandleTypeDef     s_hdma_adc1;
//static TIM_HandleTypeDef     s_htim3;
static TIM_HandleTypeDef     s_htim2;

/* ----------------- 设备/同步 ----------------- */
static rt_device_t g_uart2 = RT_NULL;
static struct rt_device_pwm *g_pwm4 = RT_NULL;
static struct rt_device_pwm *g_pwm3 = RT_NULL;

static rt_mutex_t g_mutex = RT_NULL;
static rt_event_t g_adc_evt;
#define ADC_EVT_HALF   0x01
#define ADC_EVT_FULL   0x02

/* ----------------- 运行变量 ----------------- */
volatile float g_adc_v = 0.0f;
static float average_voltage = 0.0f;

/* 阈值增量（运行时可变）：调参期每周期评估一次，达标后固定 */
static float g_delta_v = DV_INIT;

static volatile float i = 0.0f;    /* 本周期命中计数 */
static float i1 = 0.0f;
static float i_yuan = 0.0f;
static int   i_flag = 0;           /* 你的原标定状态机 */
static int   flag = 0;
static int   biaoding_flag = 0;
static int   dianya_flag = 1;

static float temperature = 0.0f, pressure = 0.0f, altitude = 0.0f;

/* 运行模式：先调 ΔV -> 达标后做 i_yuan 标定 -> 正常运行 */
typedef enum { MODE_TUNE_DV = 0, MODE_CAL_IYUAN = 1, MODE_NORMAL = 2 } run_mode_t;
static run_mode_t g_mode = MODE_TUNE_DV;

/* 正常运行下的“连续 0”计数，用于触发 ΔV 重新标定 */
static int zero_streak = 0;

/* 新增：ΔV 连续达标次数（i 在 (0, I_TARGET) 内的连续轮数） */
static int dv_ok_streak = 0;

/* ----------------- 打印/JSON ----------------- */
static void print_float(const char *tag, float v, int frac_digits)
{
    int sign = (v < 0);
    if (sign) v = -v;
    int i_part = (int)v;
    int scale = 1;
    for (int k = 0; k < frac_digits; k++) scale *= 10;
    int f_part = (int)((v - i_part) * scale + 0.5f);
    if (f_part >= scale) { i_part += 1; f_part -= scale; }

    if (sign) rt_kprintf("%s=-%d.%0*d\n", tag, i_part, frac_digits, f_part);
    else      rt_kprintf("%s=%d.%0*d\n",  tag, i_part, frac_digits, f_part);
}

static void ftoa(char *out, int out_sz, float v, int frac)
{
    if (out_sz < 4) { if (out_sz > 0) out[0] = '\0'; return; }
    int neg = (v < 0.0f);
    if (neg) v = -v;

    int scale = 1;
    for (int i = 0; i < frac; i++) scale *= 10;

    int i_part = (int)v;
    int f_part = (int)((v - (float)i_part) * scale + 0.5f);
    if (f_part >= scale) { i_part += 1; f_part -= scale; }

    if (neg) rt_snprintf(out, out_sz, "-%d.%0*d", i_part, frac, f_part);
    else     rt_snprintf(out, out_sz,  "%d.%0*d", i_part, frac, f_part);
}

static void send_json_data(float reshi, float wendu, float i1_val, float yaqiang1)
{
    char buf[200];
    char s_reshi[24], s_wendu[24], s_i1[24], s_yaqiang1[24];

    ftoa(s_reshi,    sizeof(s_reshi),    reshi,    2);
    ftoa(s_wendu,    sizeof(s_wendu),    wendu,    2);
    ftoa(s_i1,       sizeof(s_i1),       i1_val,   2);
    ftoa(s_yaqiang1, sizeof(s_yaqiang1), yaqiang1, 2);

    rt_snprintf(buf, sizeof(buf),
        "{\"method\":\"thing.event.property.post\",\"id\":\"4\","
        "\"params\":{\"mokuai2\":%s,\"wendu2\":%s,\"i1\":%s,\"yaqiang2\":%s},\"version\":\"1.0\"}",
        s_reshi, s_wendu, s_i1, s_yaqiang1);

    if (g_uart2) rt_device_write(g_uart2, 0, buf, (rt_size_t)rt_strlen(buf));
}

/* ---- TIM2: 用 CC2 事件触发 ADC（频率 = sample_rate_hz） ---- */
static void TIM2_CC2_Init(uint32_t sample_rate_hz)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 72MHz/(71+1) = 1MHz 计数时钟 */
    const uint32_t psc = 71;
    const uint32_t cnt_clk = 72000000UL / (psc + 1UL);
    uint32_t arr = (cnt_clk + sample_rate_hz - 1U) / sample_rate_hz;
    if (arr < 2U) arr = 2U;
    arr -= 1U;

    s_htim2.Instance = TIM2;
    s_htim2.Init.Prescaler         = psc;
    s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period            = arr;                    // 定周期
    s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_htim2);

    TIM_ClockConfigTypeDef clk = {0};
    clk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&s_htim2, &clk);

    /* —— 关键：初始化 OC 并配置 CH2 —— */
    HAL_TIM_OC_Init(&s_htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_TOGGLE;     // 或 TIM_OCMODE_ACTIVE 也可以
    oc.Pulse        = 1;                     // CCR2=1，每周期命中一次 → 产生一次 CC2 事件
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_2);

    /* 不需要 Master TRGO 设置；ADC 用 CC2 事件 */
    HAL_TIM_Base_Start(&s_htim2);
    HAL_TIM_OC_Start(&s_htim2, TIM_CHANNEL_2);   // 必须启动 CH2
}

/* ----------------- ADC1 + DMA 环形 ----------------- */
static void ADC1_DMA_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_CC2;
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&s_hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_5;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    HAL_ADC_ConfigChannel(&s_hadc1, &ch);

    s_hdma_adc1.Instance                 = DMA1_Channel1;
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&s_hdma_adc1);
    __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_ADCEx_Calibration_Start(&s_hadc1);
}

static void ADC1_DMA_Start(void)
{
    HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_buf, ADC_BUF_LEN);
}

/* 仅作“最新电压”展示 */
static inline void update_latest_v(uint32_t offset_last)
{
    uint16_t raw = s_adc_buf[offset_last];
    g_adc_v = ((float)raw / ADC_MAX_VALUE) * VREF;
}

/* HAL 回调 -> 发事件，由 led0_task 批处理 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        update_latest_v((ADC_BUF_LEN/2) - 1);
        rt_event_send(g_adc_evt, ADC_EVT_HALF);
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        update_latest_v(ADC_BUF_LEN - 1);
        rt_event_send(g_adc_evt, ADC_EVT_FULL);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_adc1);
}

/* ----------------- led0：批处理阈值统计 ----------------- */
static void led0_task(void *parameter)
{
    while (1)
    {
        rt_uint32_t set = 0;
        if (rt_event_recv(g_adc_evt, ADC_EVT_HALF | ADC_EVT_FULL,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &set) != RT_EOK)
            continue;

        /* 电压阈值 -> ADC 码阈值（使用逐周期调整的 g_delta_v） */
        float thr_v = average_voltage + g_delta_v;
        if (thr_v < 0) thr_v = 0;
        if (thr_v > VREF) thr_v = VREF;
        const uint16_t thr_adc = (uint16_t)((thr_v / VREF) * ADC_MAX_VALUE + 0.5f);

        uint32_t total_hits = 0;

        if (set & ADC_EVT_HALF)
        {
            const uint32_t off = 0;
            const uint32_t len = ADC_BUF_LEN/2;
            for (uint32_t n = 0; n < len; n++)
                if (s_adc_buf[off + n] >= thr_adc) total_hits++;
        }
        if (set & ADC_EVT_FULL)
        {
            const uint32_t off = ADC_BUF_LEN/2;
            const uint32_t len = ADC_BUF_LEN/2;
            for (uint32_t n = 0; n < len; n++)
                if (s_adc_buf[off + n] >= thr_adc) total_hits++;
        }

        if (total_hits && g_mutex && rt_mutex_take(g_mutex, 10) == RT_EOK)
        {
            i += (float)total_hits;
            rt_mutex_release(g_mutex);
        }
    }
}

/* ----------------- led1：基线/HP203B/上报 + ΔV 逐周期评估 ----------------- */
static void led1_task(void *parameter)
{
    while (1)
    {
        while (flag < 9)
        {
            rt_tick_t start = rt_tick_get();
            rt_pin_write(LED5_PIN, PIN_HIGH);

            /* 6s 基线电压估计（用于阈值的“基线+ΔV”） */
            if (dianya_flag == 1)
            {
                float sum_v = 0.0f; int cnt = 0;
                while (rt_tick_get() - start < rt_tick_from_millisecond(6000))
                {
                    sum_v += g_adc_v; cnt++;
                    rt_thread_mdelay(10);
                }
                if (cnt > 0) average_voltage = sum_v / (float)cnt;
                dianya_flag = 0;
            }
            else
            {
                rt_thread_mdelay(6000);
            }

            rt_pin_write(LED5_PIN, PIN_LOW);

            /* 可选：气压计 */
            if (HP203B_ReadReg(0x0d) & 0x40)
            {
                (void)HP203B_ReadData(&temperature, &pressure, &altitude);
                HP203B_StartConv();
            }

            /* 1.5s 指示 */
            rt_pin_write(LED1_PIN, PIN_HIGH);
            rt_thread_mdelay(1500);
            rt_pin_write(LED1_PIN, PIN_LOW);

            /* —— 完成一个完整周期后，读取本周期命中数 i 并做“模式相关”的处理 —— */
            if (g_mutex && rt_mutex_take(g_mutex, 10) == RT_EOK)
            {
                float cur_i = i;     /* 这轮命中 */
                float delta  = cur_i - i_yuan;

                if (g_mode == MODE_TUNE_DV)
                {
                    /* 只调 ΔV，不做 i_yuan 标定，也不算 i1（保持原输出格式） */
                    print_float("i",        cur_i,  2);
                    print_float("i_yuan",   i_yuan, 2);
                    rt_kprintf("mode=TUNE\n");
                    print_float("dianya",   average_voltage, 3);
                    rt_kprintf("dV(g_delta_v)==");
                    print_float("", g_delta_v, 3);  /* 行末有换行 */

                    if (cur_i > 5000.0f && cur_i < I_TARGET)
                    {
                        /* 达标一次：计数+1，需连续两次达标才锁定 ΔV */
                        dv_ok_streak++;
                        rt_kprintf("[TUNE] ok (%d/2): i=%.0f < %.0f, delta_v=%.3fV\n",
                                   dv_ok_streak, cur_i, I_TARGET, g_delta_v);

                        if (dv_ok_streak >= 3)
                        {
                            rt_kprintf("[TUNE] done: two consecutive ok, lock delta_v=%.3fV\n", g_delta_v);
                            i_yuan = 0.0f;
                            i_flag = 0;
                            g_mode = MODE_CAL_IYUAN;
                            dv_ok_streak = 0;
                        }
                        /* 达标但未满两次时，不改 ΔV，进入下一周期继续验证 */
                    }
                    else if (cur_i >= I_TARGET)
                    {
                        /* i 太大 → 阈值偏低 → 增大 ΔV，且打断连续达标计数 */
                        dv_ok_streak = 0;

                        float ratio = cur_i / I_TARGET;
                        float step =
                            (ratio > 10.0f) ? DV_STEP_HUGE :
                            (ratio >  3.0f) ? DV_STEP_BIG  :
                            (ratio >  1.5f) ? DV_STEP_MID  : DV_STEP_MIN;

                        float dv_cap = VREF - average_voltage - DV_MARGIN;
                        if (dv_cap < 0.02f) dv_cap = 0.02f; /* 最低留一点空间 */

                        g_delta_v += step;
                        if (g_delta_v > dv_cap) g_delta_v = dv_cap;

                        rt_kprintf("[TUNE] i=%.0f >= %.0f, increase dV by %.3f -> %.3f V (cap=%.3f)\n",
                                   cur_i, I_TARGET, step, g_delta_v, dv_cap);
                    }
                    else /* cur_i == 0 → 阈值偏高 → 减小 ΔV，并打断连续达标计数 */
                    {
                        dv_ok_streak = 0;
                        g_delta_v -= DV_STEP_MIN;
                        if (g_delta_v < DV_MIN) g_delta_v = DV_MIN;
                        rt_kprintf("[TUNE] i=0, decrease dV by %.3f -> %.3f V\n",
                                   DV_STEP_MIN, g_delta_v);
                    }
                }
                else
                {
                    /* ==== 达标后才会来到这里：做 i_yuan 标定 / 正常运行 ==== */

                    if (g_mode == MODE_CAL_IYUAN)
                    {
                        /* 你的旧逻辑：第一次 i=0.1 且不参与；累计4次后求均值进入正常 */
                        while (i_flag <= 4)
                        {

                            i_yuan += cur_i;    /* 注意：第一次这 0.1 也会加上，后面会 /4 抹掉影响 */
                            i_flag++;
                            break;
                        }
                        if (i_yuan == 0.0f) i_flag = 0;
                        if (i_flag == 5) { i_yuan /= 5.0f; i_flag++; g_mode = MODE_NORMAL; }

                        /* 标定阶段不计算 i1，只输出观察量（保持原格式） */
                        print_float("i",        cur_i,  2);
                        print_float("i_yuan",   i_yuan, 2);
                        rt_kprintf("i_flag=%d\n", i_flag);
                        print_float("i1",       0.0f,   2);
                        print_float("dianya",   average_voltage, 3);
                        rt_kprintf("dV(g_delta_v)==");
                        print_float("", g_delta_v, 3);
                        rt_kprintf("\r\n");
                    }
                    else /* MODE_NORMAL */
                    {
                        /* 进入正常运行后，沿用你原先的 i1 计算 */
                        if (i_flag > 4)
                        {
                            if (delta >= 0 && delta <= 70000)      i1 =  delta / 10000.0f;
                            else if (delta > 70000)                 i1 =  7.0f + delta / 1000.0f;
                            else                                    i1 = (-delta) / 10000.0f;
                        }

                        /* 正常输出（保持原格式） */
                        print_float("i",        cur_i,  2);
                        print_float("i_yuan",   i_yuan, 2);
                        rt_kprintf("i_flag=%d\n", i_flag);
                        print_float("i1",       i1,     2);
                        print_float("dianya",   average_voltage, 3);
                        rt_kprintf("dV(g_delta_v)==");
                        print_float("", g_delta_v, 3);
                        rt_kprintf("\r\n");

                        send_json_data(i1, temperature, cur_i, pressure);

                        /* —— 正常运行时“连续很多次 i==0”则重新标定 ΔV —— */
                        if (cur_i == 0.0f)
                        {
                            zero_streak++;
                            if (zero_streak >= ZERO_RECAL_N)
                            {
                                /* 轻微下调 ΔV 并回到调参模式 */
                                g_delta_v -= DV_STEP_MID;
                                if (g_delta_v < DV_MIN) g_delta_v = DV_MIN;

                                g_mode  = MODE_TUNE_DV;
                                i_yuan  = 0.0f;
                                i_flag  = 0;
                                zero_streak = 0;
                                dv_ok_streak = 0;

                                rt_kprintf("[TUNE] zero-streak(%d) -> re-tune, delta_v=%.3fV\n",
                                           ZERO_RECAL_N, g_delta_v);
                            }
                        }
                        else
                        {
                            zero_streak = 0; /* 只要出现非 0，就清零计数 */
                        }

                        i1 = 0.0f; /* 打印/上报后清零 */
                    }
                }

                /* 清零本周期计数，进入下一完整周期 */
                i  = 0.0f;
                rt_mutex_release(g_mutex);
            }

            flag++;
            dianya_flag = 1;
            rt_thread_mdelay(1000);
        }
        flag = 0;

        /* 你的周期性复位逻辑保留（不会影响 ΔV 已锁定的值） */
        biaoding_flag++;
        if (biaoding_flag == 50)
        {
            biaoding_flag = 0;
            if (g_mode != MODE_TUNE_DV)  /* 仅在已达标后按你原来的做法复位这些量 */
            {
                i_yuan        = 0.0f;
                i1            = 0.0f;
                i_flag        = 0;
                g_mode = MODE_TUNE_DV;//每个程序都要加这一行
            }
            dianya_flag   = 1;
        }
    }
}

/* ----------------- 主函数 ----------------- */
int main(void)
{
    g_pwm3 = (struct rt_device_pwm *)rt_device_find(PWM3_DEV_NAME);
//    g_pwm4 = (struct rt_device_pwm *)rt_device_find(PWM4_DEV_NAME);

    if (g_pwm3) {
        /* 例：15 kHz，占空比 40% */
        rt_pwm_set(g_pwm3, PWM3_CH, 66666, 26666);
        rt_pwm_enable(g_pwm3, PWM3_CH);
    }
//    if (g_pwm4) {
//        /* 例：20 kHz，占空比 50% */
//        rt_pwm_set(g_pwm4, PWM4_CH, 50000, 25000);
//        rt_pwm_enable(g_pwm4, PWM4_CH);
//    }


    /* GPIO 输出 */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED5_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_pin_write(LED5_PIN, PIN_LOW);

    /* 上电指示 */
    rt_pin_write(LED5_PIN, PIN_HIGH);
    rt_pin_write(LED1_PIN, PIN_HIGH);
    rt_thread_mdelay(6000);
    rt_pin_write(LED5_PIN, PIN_LOW);
    rt_pin_write(LED1_PIN, PIN_LOW);

    /* 串口2 */
    g_uart2 = rt_device_find(UART2_DEV_NAME);
    if (g_uart2)
    {
        config.baud_rate = 9600;
        rt_device_control(g_uart2, RT_DEVICE_CTRL_CONFIG, &config);
        rt_device_open(g_uart2, RT_DEVICE_OFLAG_WRONLY);
    }

//    /* PWM（如需） */
////    g_pwm4 = (struct rt_device_pwm *)rt_device_find(PWM4_DEV_NAME);
//    g_pwm3 = (struct rt_device_pwm *)rt_device_find(PWM3_DEV_NAME);
////    if (g_pwm4) { rt_pwm_set(g_pwm4, PWM4_CH, 100 * 1000,  5 * 1000); rt_pwm_enable(g_pwm4, PWM4_CH); }
//    if (g_pwm3) { rt_pwm_set(g_pwm3, PWM3_CH, 100 * 1000, 50 * 1000); rt_pwm_enable(g_pwm3, PWM3_CH); }




    /* 软件 I2C & HP203B */
    i2c_CfgGpio();
    HP203B_Reset();
    rt_thread_mdelay(100);
    HP203B_StartConv();

    /* 互斥量/事件 */
    g_mutex = rt_mutex_create("imtx", RT_IPC_FLAG_PRIO);
    if (!g_mutex) { rt_kprintf("mutex create failed\n"); return -1; }
    g_adc_evt = rt_event_create("adcev", RT_IPC_FLAG_PRIO);
    if (!g_adc_evt) { rt_kprintf("adcev create failed\n"); return -1; }

    /* 启动 ADC+DMA 连续采样 */
    TIM2_CC2_Init(ADC_SAMPLE_RATE_HZ);
    ADC1_DMA_Init();
    ADC1_DMA_Start();

    /* 线程（led1 优先级略高，避免 led0 吃满CPU） */
    {
        rt_thread_t t0 = rt_thread_create("led0", led0_task, RT_NULL, 768, 14, 10);
        rt_thread_t t1 = rt_thread_create("led1", led1_task, RT_NULL, 1024, 13, 10);
        if (t0) rt_thread_startup(t0); else rt_kprintf("led0 create fail\n");
        if (t1) rt_thread_startup(t1); else rt_kprintf("led1 create fail\n");
    }

    while (1) { rt_thread_mdelay(1000); }
}
