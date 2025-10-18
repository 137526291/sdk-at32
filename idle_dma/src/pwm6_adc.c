
#include "at32f421.h"
#include "log.h"
#include "foc_cfg.h"
#include "foc_f32.h"

uint16_t timer_period;
uint16_t t1ccr[3];

uint16_t adc1_regv[3];
uint16_t adc1_injv[3];
uint16_t injcnt;

//tim1 6pwm cfg, lowabc - highabc : a7 b0 b1 a8 a9 a10
int pwm6_adc_init(void)
{
    gpio_init_type gpio_init_struct = {0};
    tmr_output_config_type tmr_output_struct;
    crm_clocks_freq_type crm_clocks_freq_struct = {0};
    tmr_brkdt_config_type tmr_brkdt_config_struct = {0};
    adc_base_config_type adc_base_struct = {0};

    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* enable tmr1/gpioa/gpiob clock */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    crm_adc_clock_div_set(CRM_ADC_DIV_6);
    

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_init_struct);

    /* timer1 output pin configuration */
    gpio_init_struct.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);

    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = FALSE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = 1;
    adc_base_config(ADC1, &adc_base_struct);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_239_5);
    // adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_239_5);
    // adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 3, ADC_SAMPLETIME_239_5);
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
    // adc_dma_mode_enable(ADC1, TRUE);
    adc_ordinary_part_count_set(ADC1, 1);
    adc_ordinary_part_mode_enable(ADC1, TRUE);

    adc_preempt_channel_length_set(ADC1, 2);
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_13_5);
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_13_5);
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_6, 3, ADC_SAMPLETIME_13_5);
    adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);
    // adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_SOFTWARE, TRUE);
    adc_preempt_auto_mode_enable(ADC1, TRUE);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);

    adc_enable(ADC1, TRUE);
    adc_calibration_init(ADC1);
    while(adc_calibration_init_status_get(ADC1));
    adc_calibration_start(ADC1);
    while(adc_calibration_status_get(ADC1));

    nvic_irq_enable(ADC1_CMP_IRQn, 0, 0);

    /* tmr1 configuration to:

    1/ generate 3 complementary pwm signals with 3 different duty cycles:
      tmr1clk is fixed to system_core_clock, the tmr1 prescaler is equal to 0 so the
      tmr1 counter clock used is system_core_clock.
      * system_core_clock is set to 120 mhz .

      the objective is to generate pwm signal at 17.57 khz:
      - tmr1_period = (system_core_clock / 17570) - 1

      the three duty cycles are computed as the following description:

      the channel 1 duty cycle is set to 50% so channel 1n is set to 50%.
      the channel 2 duty cycle is set to 25% so channel 2n is set to 75%.
      the channel 3 duty cycle is set to 12.5% so channel 3n is set to 87.5%.
      the timer pulse is calculated as follows:
        - channelxpulse = duty_cycle * (tmr1_period - 1) / 100

    2/ insert a dead time equal to 11/system_core_clock ns
    3/ configure the brake feature, active at high level, and using the automatic
       output enable feature
    4/ use the locking parameters level1. */

    /* compute the value to be set in arr regiter to generate signal frequency at 17.57 khz */
    timer_period = (crm_clocks_freq_struct.sclk_freq / 17570) - 1;

    /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
    t1ccr[0] = (uint16_t)(((uint32_t)5 * (timer_period - 1)) / 10);

    /* compute ccr2 value to generate a duty cycle at 25%  for channel 2 and 2n */
    t1ccr[1] = (uint16_t)(((uint32_t)25 * (timer_period - 1)) / 100);

    /* compute ccr3 value to generate a duty cycle at 12.5%  for channel 3 and 3n */
    t1ccr[2] = (uint16_t)(((uint32_t)125 * (timer_period - 1)) / 1000);

    tmr_base_init(TMR1, PWM_ARR-1, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);//center mode

    /* channel 1, 2, 3 and 4 Configuration in output mode */
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_idle_state = FALSE;

    /* channel 1 */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);

    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, t1ccr[0]);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, t1ccr[1]);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, t1ccr[2]);
    TMR1->c1dt = PWM_ARR/2;
    TMR1->c2dt = PWM_ARR/2;
    TMR1->c3dt = PWM_ARR/2;
    TMR1->c4dt = 1;//PWM_ARR - 1;

    /* automatic output enable, stop, dead time and lock configuration */
    tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
    tmr_brkdt_config_struct.brk_enable = FALSE; //no brake func
    tmr_brkdt_config_struct.auto_output_enable = TRUE;
    tmr_brkdt_config_struct.deadtime = 8;
    tmr_brkdt_config_struct.fcsodis_state = TRUE;
    tmr_brkdt_config_struct.fcsoen_state = TRUE;
    tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
    tmr_brkdt_config_struct.wp_level = TMR_WP_LEVEL_3;
    tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);

    /* output enable */
    tmr_output_enable(TMR1, TRUE);

    /* enable tmr1 */
    tmr_counter_enable(TMR1, TRUE);
}
float norm0[3];
void ADC1_CMP_IRQHandler(void)
{
  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    // if(injcnt < 2)
    {
      adc1_injv[0] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
      adc1_injv[1] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_2);
      adc1_injv[2] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_3);
      injcnt++;
      extern foc_var_f32_t fv0;
      norm0[0] = -(adc1_injv[0] - 2048) * (1/2048.0f); // * ADC2NORM;
      norm0[1] = -(adc1_injv[1] - 2074) * (1/2048.0f); // * ADC2NORM;ic
      norm0[2] = -norm0[0] - norm0[1];//ib= -ia -ic
    // cs->jsia = inorm[0] * I_NORM2REAL * 1000;
    // cs->jsib = inorm[1] * I_NORM2REAL * 1000;
      foc_compute_f32(&fv0, norm0[0], norm0[2]);
    }
  }
}