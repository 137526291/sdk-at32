#include "foc_cfg.h"
#include "foc_f32.h"
#include "mc_math.h"
#include "at32f421.h"


foc_var_f32_t fv0;


const float cur_kp = KP_SERIAL; // 归一化后还差一个增益
const float cur_ki = KI_SERIAL;

// uint32_t enc_get0() { return ENC_MECH_1CIRCLE - TIMER_CNT(TIMER3); }
// uint32_t enc_get1() { return ENC_MECH_1CIRCLE - TIMER_CNT(TIMER2); }
// void enc_set0(uint32_t cnt) { TIMER_CNT(TIMER3) = cnt; }
// void enc_set1(uint32_t cnt) { TIMER_CNT(TIMER2) = cnt; }

void epwm_set0(const float ccr[])
{
    TMR1->c1dt = ccr[0] * PWM_ARR;
    TMR1->c2dt = ccr[1] * PWM_ARR;
    TMR1->c3dt = ccr[2] * PWM_ARR;
}

void pid_param_init_f32()
{
    // const float maxout = 0.90 * 2 - 1; // max 90%FIXME
    const float maxout = 0.3f;
    float iq_maxout = 0.1f;
    // const float cur_kp = KP_SERIAL;
    // const float cur_ki = KI_SERIAL;
    pid_struct_init_f32(&fv0.pid_id, maxout, maxout, cur_kp, cur_ki, 0);
    pid_struct_init_f32(&fv0.pid_iq, maxout, maxout, cur_kp, cur_ki, 0);
    // pid_struct_init_f32(&fv0.pid_tension, tension_maxout, tension_maxout*0.9, tp, ti, 0);
    // pid_struct_init_f32(&fv0.pid_speed, iq_maxout, iq_maxout, 0.005, 0.0001, 0);
    // pid_struct_init_f32(&fv0.pid_pos, 100, 60, 0.06, 0.000, 1);
}

void drive_foc_init()
{
    pid_param_init_f32();
    fv0.func_pwm_set = epwm_set0;
    // fv0.encoder_cnt_get = enc_get0;
    // fv0.encoder_cnt_set = enc_set0;
    foc_algorithm_init_f32(&fv0, FSM_STATE_ROTOR_ALIGN_IQ, 1); // 方向用于左右安装需要相反
}




uint32_t get_timestamp_ms(){ return 0; } // TODO implement it