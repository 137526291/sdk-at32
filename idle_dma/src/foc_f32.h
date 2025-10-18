#pragma once
#include <stdint.h>
#include <math.h>

#include "foc_cfg.h"
#include "mc_math.h"

// typedef struct _foc_var_f32_t foc_var_f32_t;

// enum{
//     FOC_CALI_IDLE = 0,
//     FOC_CALI_BUSY,
//     FOC_CALI_PASS,
//     FOC_CALI_FAIL,
// };

// uint32_t status.       ctrl mode | with_sensor | |
// ctrl mode : idle / openloop / current / speed
// sensor type : hall / mag / linear-hall / enc-abz / unknown / sensorless
// enum{
//     FOC_MODE_IDLE = 0,
//     // FOC_MODE_ALIGN_HOLD,
//     // FOC_MODE_STARTUP,
//     FOC_MODE_OPENLOOP,
//     FOC_MODE_CURRENT,
//     FOC_MODE_SPEED,
// };

// enum{
//     FOC_SENSOR_TYPE_HALL = 0,
//     FOC_SENSOR_TYPE_MAGNETIC,
//     FOC_SENSOR_TYPE_LINEAR_HALL,
//     FOC_SENSOR_TYPE_ENCORDER,
//     FOC_SENSOR_TYPE_STO_SENSORLESS,
//     FOC_SENSOR_TYPE_UNDEFINED,
// };

enum
{
    FSM_STATE_IDLE = 0, // motor cutoff output
    // FSM_STATE_ROTOR_ALIGN_CALI, //align and calib.
    FSM_STATE_ROTOR_ALIGN,      //1//align and hold. no auto jump.
    FSM_STATE_ROTOR_ALIGN_VQ,	//2
    FSM_STATE_ROTOR_ALIGN_IQ,	//3
    FSM_STATE_SENSOR_OPENLOOP,	//4
    FSM_STATE_SENSOR_CURRENT,	//5
    FSM_STATE_SENSOR_SPEED,		//6
    FSM_STATE_SENSOR_POSITION,	//7
    FSM_STATE_SENSOR_CURRENT_TENSION,	//8
    FSM_STATE_TEST_FILTER,
};

typedef struct _foc_func_set
{
    uint16_t (*get_sensor_theta_electrical)(); // 获取电角度
    uint16_t (*get_sensor_theta_mechanical)(); // 获取机械角度
    int32_t (*get_sensor_odmetry)();           // 传感器里程计
    void (*reset_sensor_zero)();               // 重设传感器角度，广编归零
    void (*calibrate_sensor_theta_offset)();   // 校准记录传感器0位
    void (*tp_pin_set)(int v);
} foc_func_set_t;

// typedef struct _sensorless_var
// {
//     /* data */
//     uint16_t sto_angle;
//     uint16_t sto_last_angle;
//     int32_t sto_delta;
//     int16_t sto_speed;
//     int16_t sto_angle_err;
//     int16_t sto_speed_rpm;
//     int16_t sto_speed_fil;

//     uint32_t align_start_timestamp;
//     uint32_t align_end_timestamp;
//     uint32_t try_startup_timestamp;
//     uint32_t start_speed_timestamp;
// }sensorless_var_t;

// typedef struct _foc_var_int_t{

//     int16_t ia;
//     int16_t ib;
//     int16_t ic;

//     int16_t ialfa;
//     int16_t ibeta;

//     int16_t id;
//     int16_t iq;

//     int16_t id_ref;
//     int16_t iq_ref;

//     int16_t iq_ref_ramp;
//     float iq_ref_rampf;

//     int16_t vd; //=k*id
//     int16_t vq;

//     int16_t valfa;
//     int16_t vbeta;

//     uint16_t theta;	//electical angle.
//     uint16_t last_theta;
//     uint16_t theta_mechanical;  //mechanical angle
//     uint16_t last_theta_mechanical;
//     uint16_t sto_theta;
//     uint16_t last_sto_theta;

//     int32_t align_mechanical_delta;
//     // uint16_t align_start_theta_mechanical;
//     // uint16_t align_end_theta_mechanical;
//     // int16_t sin_theta;
//     // int16_t cos_theta;

//     int16_t tabc[3];    //each channel switch time
//     uint32_t* pwm[3];
//     uint32_t pwm_full;

//     uint8_t sector;//space vector [1~6]

//     int16_t theta_offset;
//     int16_t out_en;
//     uint8_t foc_cali_status;    //0=idle. 1=calibrating. 2=ok.3=fail
//     uint8_t fsm_state;

//     lpf1_t lpf1_ia;
//     lpf1_t lpf1_ib;
//     lpf1_t lpf1_iq;
//     lpf1_t lpf1_id;

//     //speed ctrl.
//     int32_t theta_delta;
//     int32_t theta_delta_mechanical;
//     int32_t speed_radps_mechanical;    //omega, rad/s
//     int32_t speed_rpm;
//     int32_t speed_rpm10;   //低速电机使用 max +-327 rpm
//     int32_t speed_rpm_mechanical;
//     int32_t speed_ref;
//     // int32_t speed_ref100;
//     lpf1_t lpf1_speed;
//     // int16_t speed_pid_out;

//     //position ctrl
//     lpf1_t lpf1_pos;
//     int32_t pos_fdbk;
//     int32_t pos_ref;

//     uint8_t ctrl_mode;
//     uint8_t sensor_type;

//     uint32_t align_start_timestamp;

//     void (*tp_pin_set)(int v);
//     uint16_t (*get_sensor_theta)();
//     uint16_t (*get_sensor_theta_mechanical)();
//     int32_t (*get_sensor_odmetry)();
//     void (*reset_sensor_zero)();
//     void (*calibrate_sensor_theta_offset)();

//     //motor param.
//     uint8_t pole_pair;

//     pid_q15_t pid_id;
//     pid_q15_t pid_iq;
//     pid_q15_t pid_speed;
//     pid_q15_t pid_pos;
//     pid_q15_t pid_pos_only;

//     uint32_t current_ctrl_div_cnt;
//     uint32_t speed_calc_div_cnt;
//     uint32_t speed_ctrl_div_cnt;    //用于调节速度环相对电流环的控制频率
//     uint32_t pos_ctrl_div_cnt;

//     // sensorless_var_t sl;
// }foc_var_int_t;

// typedef struct{
//     float iq_sin_freq;  //iq = sin(w*t)
//     float iq_sin_dt;
// }foc_test_t;

typedef struct _foc_var_f32_t
{
    int32_t jsia;
    int32_t jsib;
    int32_t jsid;
    int32_t jsiq;
    float ia; // unit: ampere
    float ib;
    float ic;

    float ia_norm;
    float ib_norm;

    float ialfa;
    float ibeta;

    float id;
    float iq;

    float id_ref;
    float iq_ref;
    float iq_ref_set;
    float iq_tension_comp;

    float vd; //=k*id
    float vq;
    float vd_norm;
    float vq_norm;
    float vref_limit;

    float valfa;
    float vbeta;

    float theta; // electical angle rad.
    float last_theta;
    // float theta_path;   //[0, +inf]
    // float theta_deg;    //electical angle degree.

    // float smo_theta;
    // float smo_theta_accumulate;
    // float last_smo_theta;
    // float sensor_theta;
    float sin_theta;
    float cos_theta;

    // hardware motor
    float tabc[3]; // each channel switch time [-1, 1]
    float pwm[4];  // write to reg, pwm_duty = ccr/full
    // uint32_t pwm_full;
    // uint32_t pwm_full_half;
    float vbus;
    float ibus;

    // motor
    uint8_t npp; // pole pair.
    float ls;    // inductance
    float rs;    // resistance

    uint8_t sector; // space vector [1~6]

    float theta_offset;
    float theta_compensate; //
    uint8_t out_en;
    uint8_t foc_cali_status; // 0=idle. 1=calibrating. 2=ok.3=fail
    uint8_t fsm_state;
    uint8_t fsm_after_align; // current or speed or position

    // speed ctrl.
    //  float theta_delta;
    uint16_t theta_u16; // electical angle.
    uint16_t last_theta_u16;
    int32_t theta_delta;
    uint16_t theta_mechanical; // mechanical angle
    int32_t speed_rpm_mechanical;
    int32_t align_mechanical_delta;
    int32_t theta_delta_mechanical;
    int32_t speed_radps_mechanical; // omega, rad/s
    int32_t speed_rpm;              // round per min
    int32_t speed_rpm10;            // 低速电机使用 max +-327 rpm
    float speed_rps;                // round per sec
    float _speed_radps;             // rad per sec
    float speed_radps;
    // float speed_rpm;    //round per min
    // float speed_ref_set;
    float speed_ref_radps;
    float speed_ref_radps_set;
    float speed_ref_rpm;
    lpf1_t lpf1_speed;

    // position ctrl.
    float relative_pos;
    float relative_pos_deg;
    // float pos_ref;
    int32_t pos_fdbk;
    int32_t pos_ref;

    pid_f32_t pid_id;
    pid_f32_t pid_iq;
    pid_f32_t pid_speed;
    pid_f32_t pid_pos;
    pid_f32_t pid_tension;

    uint8_t ctrl_mode;
    uint8_t sensor_type;

    uint32_t align_start_timestamp;
    uint32_t align_end_timestamp;
    uint32_t try_startup_timestamp;
    uint32_t start_speed_timestamp;

    uint16_t raw_cnt_prev;
    uint16_t raw_cnt;
    int32_t raw_cnt_delta;
    int32_t raw_odmetry; // 累计里程计

    float startup_time;
    float startup_acc;
    float foc_dt; // dt = 1/f_foc
    float speed_1_div_dt;

    uint32_t current_ctrl_div_cnt;
    uint32_t speed_calc_div_cnt;
    uint32_t speed_ctrl_div_cnt; // 用于调节速度环相对电流环的控制频率
    uint32_t pos_ctrl_div_cnt;

    float iq_sin_w;
    float iq_sin_theta;
    float iq_sin;
    float iq_lpf;
    float iq_hpf;
    lpf1_t lpf1_iq;
    hpf1_t hpf1_iq;
#if FLUX_PSIF_ESTIMATE
    flux_linkage_t fl_alfa;
    flux_linkage_t fl_beta;
    float psif;
#endif
    
    uint32_t (*encoder_cnt_get)(void);
    void (*encoder_cnt_set)(uint32_t cnt);

    void (*tp_pin_set)(int v);
    void (*func_pwm_set)(const float pwm_ccr[3]);
    float (*theta_slide_mode_observer)(float valfa, float vbeta, float ialfa, float ibeta);
    uint16_t (*get_sensor_theta_mechanical)();
    int8_t dir; // 01都是正向（电角度增大） -1反向

    lbg_observer_t observer;
    uint8_t comp_open;
} foc_var_f32_t;

typedef struct _ob_var
{
    int32_t ia;
    int32_t ib;
    int32_t id;
    int32_t iq;
    int32_t iq_r;
    int32_t iq_comp;
    int16_t theta;
    int16_t vd15;
    int16_t vq15;
} ob_var_t;

// void pmsm_foc_pwm_init(void);
// void foc_register_sensor_func_set(foc_var_f32_t *pfv, foc_sensor_func_set_t *func_set);
void foc_register_func_set(foc_var_f32_t *pfv, foc_func_set_t *func_set);
void foc_compute(foc_var_f32_t *pfv, int16_t _ia, int16_t _ib);
void htim_update_callback_foc(void *htim);
void foc_register_pwm_output(foc_var_f32_t *pfv, uint32_t ch[]);
void foc_register_tp_pin_func(void (*tp_func)(int));
void foc_algorithm_init2(foc_var_f32_t *pfv);
int32_t calc_magnetic_encoder_speed_raw(uint16_t *last, uint16_t cur);

void foc_algorithm_init_f32(foc_var_f32_t *pfv, int state_after_align, int dir);
void foc_compute_f32(foc_var_f32_t *pfv, float _ia, float _ib);
void foc_register_pwm_output_f32(foc_var_f32_t *pfvf, void (*f_pwm_set)(const float ccr[])); // 0-1=0-100%

// void foc_mag_openloop(char argc, char* argv);
// void sw2closeloop_function(char argc, char *argv);
// void foc_sensorless_start(char argc, char *argv);
// void foc_align_rotor(char argc, char* argv);
// void foc_start_mag(char argc, char* argv);
// void foc_stop(char argc, char* argv);
// void foc_align_rotor_add(char argc, char* argv);
// void foc_align_rotor_calib_mag(char argc, char* argv);
// void foc_start_enc(char argc, char* argv);
// void foc_start_enc_current(char argc, char* argv);
// void foc_start_enc_speed(char argc, char* argv);
// void foc_speed_inc(char argc, char* argv);
// void foc_speed_dec(char argc, char* argv);
// void foc_record_to_flash(char argc, char* argv);

// void foc_button_ctrl();
// void foc_uart_ctrl(uint8_t* buf, uint16_t len );

// void foc_start_cali();
// void foc_start_align_then_sensor();
// void foc_start_align_then_sensorless();
// void foc_start_cali();
// void foc_stop();