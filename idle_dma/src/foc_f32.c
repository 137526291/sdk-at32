#include "foc_f32.h"
#include "foc_cfg.h"
#include "log.h"
#include <math.h>
#include "mc_math.h"
#include "systick.h"

// #define ALIGN_THETA_RAD ((3.0f / 4) * 2 * 3.1415926536)
const float ALIGN_THETA_RAD = ((3.0f / 4) * 2 * 3.1415926536);
const float k_u16theta2rad = (2 * 3.1415926536 / 65536.0f);
float iq_ref_amp = ALIGN_IQ_F32;
float wref = 300;

void foc_register_pwm_output_f32(foc_var_f32_t *pfvf, void (*f_pwm_set)(const float ccr[]))
{
    if (f_pwm_set)
    {
        pfvf->func_pwm_set = f_pwm_set;
    }
}

void foc_algorithm_init_f32(foc_var_f32_t *pfv, int state_after_align, int dir)
{
    if (dir != 1 && dir != -1)
    {
        dir = 1;
    }

    pfv->dir = dir; // 规定转动方向
    pfv->npp = POLE_PAIR;
    pfv->theta = ALIGN_THETA_RAD;
    pfv->last_theta = pfv->theta;
    // pfv->speed_ref_radps = SW_SPEED_RADPS;
    pfv->iq_ref = ALIGN_IQ_F32;
    pfv->vq = 0;

    pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_IQ;
    // pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_VQ;
    pfv->vq = ALIGN_VQ / VBUS_BASE; // R0.03
    pfv->fsm_after_align = state_after_align;
    pfv->align_start_timestamp = get_timestamp_ms();
    pfv->out_en = 1;

    // lpf_1rd_init(&pfv->lpf1_ia, T_SAMPLE, 10000);
    // lpf_1rd_init(&pfv->lpf1_ib, T_SAMPLE, 10000);
    lpf1_init(&pfv->lpf1_speed, T_SAMPLE, 200);   //53==800rpm
    // lpf_1rd_init(&pfv->lpf1_iq, T_SAMPLE, 100);
    // lpf1_init(&pfv->lpf1_iq, T_SAMPLE, 10);
    // hpf1_init(&pfv->hpf1_iq, T_SAMPLE, 10);
#if FLUX_PSIF_ESTIMATE
    flux_linkage_estimate_init(&pfv->fl_alfa, T_SAMPLE, PHASE_LS, PHASE_RS, 10);
    flux_linkage_estimate_init(&pfv->fl_beta, T_SAMPLE, PHASE_LS, PHASE_RS, 10);
#endif
    luenburger_observer_init(&pfv->observer, 0.045, 0.01, 0.6, 1e4, -1e4);
    pfv->observer.th2 = 2;
    pfv->observer.th1 = -0.6;
    // pfv->observer.iq1 = 0.02;

    pfv->comp_open = 1;
    pfv->vref_limit = 0.9f;
    // lpf_1rd_init(&pfv->lpf1_id, 0.05, 0);
}

static inline void foc_speed_calc(foc_var_f32_t *pfv)
{
    const int ENC2MECH_Q15 = 65536;
    pfv->raw_cnt = pfv->encoder_cnt_get ? pfv->encoder_cnt_get() : 0;
    pfv->theta_mechanical = pfv->raw_cnt * ENC2MECH_Q15;

    if (++pfv->speed_calc_div_cnt == (FOC_FREQ / SPEED_CTRL_FREQ))
    {
        pfv->speed_calc_div_cnt = 0;
        // pfv->raw_cnt_delta = calc_theta_delta_raw(&pfv->raw_cnt_prev, pfv->raw_cnt, ENC_MECH_1CIRCLE);
        pfv->raw_odmetry += pfv->dir * pfv->raw_cnt_delta;
        pfv->theta_delta_mechanical = pfv->raw_cnt_delta * ENC2MECH_Q15;
        pfv->speed_radps_mechanical = pfv->theta_delta_mechanical * SPEED_CTRL_FREQ;
        pfv->speed_rpm_mechanical = pfv->speed_radps_mechanical * 60 / 65536;
        // omega = delta theta / dt. 增大了10435
        pfv->speed_rpm10 = pfv->speed_rpm_mechanical;
        pfv->speed_rpm = pfv->dir *
                         lpf1_calc(&pfv->lpf1_speed, pfv->speed_rpm10);
        const float radps2rpm = (1 / 6.28f * (60));
        const float rpm2radps = 1.f / radps2rpm;
        pfv->speed_radps = pfv->speed_rpm * rpm2radps;
    }
}

// static inline void foc_speed_calc_f32(foc_var_f32_t *pfv)
// {
//     //  pfv->theta_delta = calc_theta_delta_f32(&pfv->last_theta, pfv->theta, 2*3.1416f);
//     if (++pfv->speed_calc_div_cnt == (FOC_FREQ / SPEED_CTRL_FREQ))
//     {
//         pfv->speed_calc_div_cnt = 0;
//         pfv->theta_delta = calc_theta_delta_f32(&pfv->last_theta, pfv->theta, 2 * 3.1416f);
//         pfv->_speed_radps = pfv->theta_delta * SPEED_CTRL_FREQ; // 1/dt = freq
//         pfv->speed_radps =
//             lpf1_calc(&pfv->lpf1_speed, pfv->_speed_radps);
//         const float radps2rpm = (1 / 6.28f * (60)) / POLE_PAIR;
//         pfv->speed_rpm = pfv->speed_radps * radps2rpm;
//     }
// }

static inline void foc_speed_ctrl_f32(foc_var_f32_t *pfv)
{
    if (++pfv->speed_ctrl_div_cnt == (CURRENT_CTRL_FREQ / SPEED_CTRL_FREQ))
    {
        pfv->speed_ctrl_div_cnt = 0;

        if (FSM_STATE_SENSOR_SPEED == pfv->fsm_state ||
            FSM_STATE_SENSOR_POSITION == pfv->fsm_state)
        {
            pfv->speed_ref_radps = pfv->dir * pfv->speed_ref_radps_set;
            pfv->iq_ref = pfv->dir *
                          pi_calc_serial_f32(&pfv->pid_speed,
                                             pfv->speed_radps,
                                             pfv->speed_ref_radps);
            // pfv->iq_ref =
            //     pi_calc_serial_f32(&pfv->pid_speed,
            //                         pfv->speed_radps,
            //                         pfv->speed_ref_radps);
        }
    }
}

static inline void foc_pos_ctrl_f32(foc_var_f32_t *pfv)
{
    if (++pfv->pos_ctrl_div_cnt == (CURRENT_CTRL_FREQ / POSITION_CTRL_FREQ))
    {
        pfv->pos_ctrl_div_cnt = 0;

        if (FSM_STATE_SENSOR_POSITION == pfv->fsm_state)
        {
            pfv->speed_ref_radps_set = pfv->dir *
                pid_calc_serial_f32(&pfv->pid_pos,
                                  pfv->pos_fdbk,
                                  pfv->pos_ref);
        }
    }
}
void foc_fsm_loop_f32(foc_var_f32_t *pfv)
{
    switch (pfv->fsm_state)
    {
    case FSM_STATE_IDLE:
    {
    }
    break;

    case FSM_STATE_ROTOR_ALIGN:
    {
        pfv->iq_ref = iq_ref_amp;
        pfv->id_ref = 0;
        // pfv->theta = ALIGN_THETA;
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_VQ:
    {
        pfv->vq = ALIGN_VQ; // R0.03
        pfv->vd = 0;
        // pfv->theta += -pfv->dir * 0.001; // 定零位前反方向回转一段
        // 
        //     pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_IQ;
        //     pfv->align_start_timestamp = get_timestamp_ms();
        //     pfv->theta = ALIGN_THETA_RAD;
        // 
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_IQ:
    {
        if (pfv->iq_sin_w)
        {
            pfv->iq_sin_theta += pfv->iq_sin_w * T_SAMPLE;
            pfv->iq_ref = pfv->iq_sin = iq_ref_amp * sinf(pfv->iq_sin_theta);
        }
        else
        {
            // normal align iq
            pfv->iq_ref = iq_ref_amp;
            pfv->id_ref = 0;
        }

#if 1
        if (get_timestamp_ms() - pfv->align_start_timestamp > ALIGN_TIME_MS)
        {
            if (pfv->encoder_cnt_set)
                pfv->encoder_cnt_set(0);
            pfv->raw_cnt_delta = 0;
            pfv->raw_cnt_prev = 0;
            pfv->raw_odmetry = 0;
            pfv->align_end_timestamp = get_timestamp_ms();
            pfv->try_startup_timestamp = get_timestamp_ms();
            // pfv->fsm_state = FSM_STATE_SENSOR_CURRENT;
            //  pfv->fsm_state = FSM_STATE_SENSOR_SPEED;
            pfv->fsm_state = pfv->fsm_after_align;
            // pfv->fsm_state = FSM_STATE_IDLE;
            pfv->start_speed_timestamp = get_timestamp_ms();
            pfv->iq_ref_set = 0.0;
            pfv->vd = 0;
            pfv->vq = 0;
            // LOGW("goto sensor openloop");
        }
#endif
    }
    break;

    case FSM_STATE_SENSOR_OPENLOOP:
    {
        // pfv->vq = pfv->dir * 15000;
        pfv->theta_u16 = pfv->theta_mechanical * POLE_PAIR;
        pfv->theta = pfv->theta_u16 * k_u16theta2rad;
    }
    break;

    case FSM_STATE_SENSOR_CURRENT:
    {
        // static float iq_ref_float;
        // 20k*0.1=2k 1s.
        //  iq_ref_float = ramp_calc_float(iq_ref_float, IQ_REF, 0.02f);
        //  pfv->iq_ref = iq_ref_float;
        // 使用光编更新电角度
        // ea = fmodf(ea, 2*3.1416f);
        pfv->pos_fdbk = pfv->raw_odmetry;
        pfv->theta_u16 = pfv->theta_mechanical * POLE_PAIR;
        pfv->theta = pfv->theta_u16 * k_u16theta2rad;

        // compensate.
        //  pfv->iq_tension_comp = pi_calc_serial_f32(&pfv->pid_tension, pfv->observer.tload_est, 0.6);
        //  if (pfv->iq_tension_comp < 0) {
        //      pfv->iq_tension_comp = pfv->pid_tension.out * 0.2;
        //  }
        //  if (!pfv->comp_open) {
        //      pfv->iq_tension_comp = 0;
        //  }
        //  pfv->iq_ref = pfv->dir * pfv->iq_ref_set + pfv->iq_tension_comp;

        // torque planner implemente
        pfv->iq_ref = pfv->dir * pfv->iq_ref_set + pfv->iq_tension_comp;
    }
    break;

    case FSM_STATE_SENSOR_SPEED:
    {
        pfv->theta_u16 = pfv->theta_mechanical * POLE_PAIR;
        pfv->theta = pfv->theta_u16 * k_u16theta2rad;
        foc_speed_ctrl_f32(pfv);//FIXME.
    }
    break;

    case FSM_STATE_TEST_FILTER:
    {
        if (pfv->iq_sin_w)
        {
            pfv->iq_sin_theta += pfv->iq_sin_w * T_SAMPLE;
            if (pfv->iq_sin_theta > 6.2832) {
                pfv->iq_sin_theta -= 6.2832;
            }
            pfv->iq_ref = pfv->iq_sin = 100 * sinf(pfv->iq_sin_theta);
        }
        pfv->iq_lpf = lpf1_calc(&pfv->lpf1_iq, pfv->iq_ref);
        pfv->iq_hpf = hpf1_calc(&pfv->hpf1_iq, pfv->iq_ref);
    }
    break;

    default:
        break;
    }
}

// unit: 使用标准单位，有利于将来公式推导仿真计算
// 电流使用安培A, 电压使用伏特V, etc...
// using normalization.
void foc_compute_f32(foc_var_f32_t *pfv, float _ia, float _ib)
{
    if (pfv->tp_pin_set)
        pfv->tp_pin_set(1);

    foc_fsm_loop_f32(pfv);

    pfv->ia = _ia; // 1 = max current measured.
    pfv->ib = _ib;
    // float vbus_base = pfv->vbus * (1.0f/1.73f);
    // if (pfv->theta_slide_mode_observer)
    // pfv->smo_theta = pfv->theta_slide_mode_observer(pfv->valfa, pfv->vbeta, pfv->ialfa, pfv->ibeta);
    // foc_speed_calc_sensorless_f32(pfv);
    foc_speed_calc(pfv);

    // clarke_float(pfv->ia, pfv->ib, &pfv->ialfa, &pfv->ibeta);

    // pfv->sin_theta = sinf(pfv->theta);
    // pfv->cos_theta = cosf(pfv->theta);
    pfv->sin_theta = fast_sin(pfv->theta);
    pfv->cos_theta = fast_cos(pfv->theta);

    // when θ = 0, id = ialfa, iq = ibeta.
    // park_float_inline(pfv->ialfa, pfv->ibeta, pfv->sin_theta, pfv->cos_theta, &pfv->id, &pfv->iq);
    dq0_inline(pfv->sin_theta, pfv->cos_theta, pfv->ia, pfv->ib, &pfv->id, &pfv->iq);

    // luenburger_observer_update(&pfv->observer, pfv->iq * I_NORM2REAL, pfv->speed_radps);

#if FLUX_PSIF_ESTIMATE
    //讲道理应该传入i和v都是标准单位，但v的影响比较大，所以输出乘上v系数即可
    flux_linkage_estimate(&pfv->fl_alfa, pfv->ialfa, pfv->valfa);
    flux_linkage_estimate(&pfv->fl_beta, pfv->ibeta, pfv->vbeta);
    pfv->psif = flux_psif_calc(&pfv->fl_alfa, &pfv->fl_beta) * VBUS_BASE;
#endif

    if (
        FSM_STATE_ROTOR_ALIGN_VQ != pfv->fsm_state &&
        FSM_STATE_SENSOR_OPENLOOP != pfv->fsm_state)
    {
        // need to implement pi regulatator
        pfv->vd = pi_calc_serial_f32(&pfv->pid_id, pfv->id, pfv->id_ref);
        pfv->vq = pi_calc_serial_f32(&pfv->pid_iq, pfv->iq, pfv->iq_ref);
    }
    if (pfv->fsm_state == FSM_STATE_IDLE)
    {
        pfv->vd = pfv->vq = 0;
    }

    // 将vq vd根据vbus缩放
    pfv->vd_norm = pfv->vd;
    pfv->vq_norm = pfv->vq;
    circle_limitation(&pfv->vd_norm, &pfv->vq_norm, pfv->vref_limit);

    /* vd,vq => valfa vbeta IPARK*/
    inverse_park_float_inline(pfv->vd_norm, pfv->vq_norm, pfv->sin_theta, pfv->cos_theta, &pfv->valfa, &pfv->vbeta);

    // 归一化的svpwm valfa和vbeta范围[-1,1]对应tabc[-1,1]
    svpwm_float(pfv->valfa, pfv->vbeta, pfv->tabc);
    if (pfv->out_en)
    {
        pfv->pwm[0] = 0.5f + pfv->tabc[0] * 0.5f;
        pfv->pwm[1] = 0.5f + pfv->tabc[1] * 0.5f;
        pfv->pwm[2] = 0.5f + pfv->tabc[2] * 0.5f;
    }
    else
    {
        pfv->pwm[0] = 0;
        pfv->pwm[1] = 0;
        pfv->pwm[2] = 0;
    }
    if (pfv->func_pwm_set)
        pfv->func_pwm_set(pfv->pwm);
    void js_watch_update(foc_var_f32_t * pfv);
    js_watch_update(pfv);

    if (pfv->tp_pin_set)
        pfv->tp_pin_set(0);
}

// //电机中点往外为ia正方向 仅计算电流环
// void foc_coref(foc_var_f32_t *pfv, float iq_ref)
// {
//     if (pfv->tp_pin_set) pfv->tp_pin_set(1);

//     pfv->sin_theta = FastSin(pfv->theta);
//     pfv->cos_theta = FastCos(pfv->theta);
//     //when θ = 0, id = ialfa, iq = ibeta.
//     dq0_inline(pfv->sin_theta,pfv->cos_theta, pfv->ia, pfv->ib, &pfv->id, &pfv->iq);

//     pfv->vd = pi_calc_serial_f32(&pfv->pid_id, pfv->id, pfv->id_ref);
//     pfv->vq = pi_calc_serial_f32(&pfv->pid_iq, pfv->iq, pfv->iq_ref);

//     inverse_park_float_inline(pfv->vd, pfv->vq,
//                             pfv->sin_theta,pfv->cos_theta,
//                             &pfv->valfa, &pfv->vbeta);
//     /* vq,vd => valfa vbeta IPARK*/
//     // inverse_park_float(pfv->vq, pfv->vd, pfv->theta, &pfv->valfa, &pfv->vbeta);

//     svpwm_float(pfv->valfa, pfv->vbeta, pfv->tabc);
//     if (pfv->out_en)
//     {
//         //ta tb tc range[-1, 1] = [-ARR/2, ARR/2]
//         pfv->pwm[0] = 0.5f + pfv->tabc[0] * 0.5f;
//         pfv->pwm[1] = 0.5f + pfv->tabc[1] * 0.5f;
//         pfv->pwm[2] = 0.5f + pfv->tabc[2] * 0.5f;
//         pfv->func_pwm_set(pfv->pwm);
//         //到底应该输出啥样的pwm数值？
//     }

//     if (pfv->tp_pin_set) pfv->tp_pin_set(0);
// }



volatile ob_var_t obv;

void js_watch_update(foc_var_f32_t *pfv)
{
    // const int igain = 1024;
    const int igain = I_NORM2REAL * 1000; // unit ma.
    pfv->jsia = pfv->ia * igain;
    pfv->jsib = pfv->ib * igain;
    pfv->jsid = pfv->id * igain;
    pfv->jsiq = pfv->iq * igain;
    // obv.ia = (int)(pfv->ia * igain);
    // obv.ib = (int)(pfv->ib * igain);
    // obv.id = (int)(pfv->id * igain);
    // obv.iq = (int)(pfv->iq * igain);
    // obv.iq_r = pfv->iq_ref * igain;
    // obv.iq_comp = pfv->iq_tension_comp * igain;
    // // obv.theta = (int) (pfv->theta * igain) ;
    // obv.theta = pfv->theta * (65536 / 6.2832);
    // obv.vd15 = pfv->vd * 32768;
    // obv.vq15 = pfv->vq * 32768;
    // smo
    //  obv.iest = (int) (psa->i_est * igain) ;
    //  obv.zalfa = (int) (psa->Z * 1024) ;
    //  obv.ealfa = (int) (psa->E * 1024) ;
    //  obv.ccr1 = pfv->pwm[0] - pfv->pwm_full_half;
    //  obv.ccr2 = pfv->pwm[1] - pfv->pwm_full_half;
    //  obv.ccr3 = pfv->pwm[2] - pfv->pwm_full_half;
}