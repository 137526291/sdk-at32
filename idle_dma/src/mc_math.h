#ifndef MC_MATH_H
#define MC_MATH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stdint.h>

#define ABS(x) (((x) > 0) ? (x) : -(x))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX_OF_3(a, b, c) MAX(a, MAX(b, c))
#define VA_LIMIT(x, min, max) \
    do                        \
    {                         \
        if ((x) < min)        \
            (x) = min;        \
        if ((x) > max)        \
            (x) = max;        \
    } while (0)

#define MAP(v, min1, max1, min2, max2) ((v - min1) * (max2 - min2) / (max1 - min1) + min2)

    typedef struct
    {
        int16_t cosx;
        int16_t sinx;
    } trig_value_t;

    typedef struct
    {
        uint8_t idx;
        int16_t *pfilter_buffer;
        uint8_t len;
        int16_t output;
    } slide_filter_t;

    typedef struct
    {
        float fc;
        float in;
        float a;    // aka. coeffient
        float _1_a; // aka. 1-a
        float yn;   // y(n)
        float yn_1; // y(n-1)
    } lpf1_t;

    void lpf1_init(lpf1_t *lpf1, float t_sample, float f_cutoff);
    float lpf1_calc(lpf1_t *lpf1rd, float new_sample);

    typedef struct
    {
        float a;
        float _1_a;
        float xn_1; // previous x.
        float yn;
        float yn_1;
    } hpf1_t;

    void hpf1_init(hpf1_t *hpf1, float ts, float f_cutoff);
    float hpf1_calc(hpf1_t *hpf1, float new_sample);

    trig_value_t calc_sin_cos(int16_t x, int16_t *sinx, int16_t *cosx);

    int16_t sigmoid_q15(int32_t); // q15 5~5
    float sigmoid_float(float x);
    float sigmoid_fake_float(float x);

    int16_t sin_q15(uint16_t);
    int16_t cos_q15(uint16_t);

    float fast_sin(float theta);
    float fast_cos(float theta);

    void clarke_q15(int16_t ia,
                    int16_t ib,
                    int16_t *ialfa,
                    int16_t *ibeta);

    // when θ = 0, id = ialfa, iq = ibeta.
    void park_q15(int16_t ialfa,
                  int16_t ibeta,
                  uint16_t theta,
                  int16_t *id,
                  int16_t *iq);

    void inverse_park_q15(int16_t vq,
                          int16_t vd,
                          uint16_t theta,
                          int16_t *valfa,
                          int16_t *vbeta);

    void svpwm_q15(int16_t valfa, int16_t vbeta, int16_t tabc[]);
    void svpwm_float(float valfa, float vbeta, float tabc[]);
    void svpwm_float2(float valfa,
                      float vbeta,
                      float tabc[],
                      uint16_t pwm_full,
                      uint16_t ccr[]);

    void clarke_float(float ia,
                      float ib,
                      float *ialfa,
                      float *ibeta);

    void park_float(float ialfa,
                    float ibeta,
                    float theta,
                    float *id,
                    float *iq);

    void inverse_park_float(float vd,
                            float vq,
                            float theta,
                            float *valfa,
                            float *vbeta);

    static inline void dq0_inline(float sin, float cos, float a, float b, float *d, float *q)
    {
        //   float cos = FastSin(theta);
        //   float sin = FastCos(theta);
        //   const float ONE_DIV_SQRT3 = 0.577350269f;
        // float ialfa = a;
        float ibeta = 0.577350269f * (2.0f * b + a);
        *d = a * cos + ibeta * sin;
        *q = -a * sin + ibeta * cos;
    }

    static inline void park_float_inline(
        float ialfa,
        float ibeta,
        float sin_theta,
        float cos_theta,
        float *id,
        float *iq)
    {
        // TODO: use look up tab to reduce time.
        *id = ialfa * cos_theta + ibeta * sin_theta;
        *iq = -ialfa * sin_theta + ibeta * cos_theta;
    }

    static inline void inverse_park_float_inline(
        float vd,
        float vq,
        float sin_theta,
        float cos_theta,
        float *valfa,
        float *vbeta)
    {
        *valfa = vd * cos_theta - vq * sin_theta;
        *vbeta = vd * sin_theta + vq * cos_theta;
    }

    static inline void abc_inline(float d, float q,
                                  float sin_theta,
                                  float cos_theta,
                                  float tabc[])
    {
    }

    int16_t slide_filter_int16(int16_t cur,
                               uint8_t *idx,
                               int16_t fil_buffer[],
                               uint8_t len);

    int max_of_3(int a, int b, int c);
    int min_of_3(int a, int b, int c);
    int32_t calc_theta_delta_raw(uint16_t *last, uint16_t cur, const int cnt_full_range);
    float calc_theta_delta_f32(float *last, float cur, const float cnt_full_range);

    int32_t ramp_calc(int32_t target, int32_t cur, int32_t dy);
    float ramp_calc_float(float cur, float target, float ramp_delta);

    void dq0(float theta, float a, float b, float *d, float *q);
    int16_t value_limit(int32_t x, int16_t MIN, int16_t MAX);

    typedef struct
    {
        float J;        //
        float one_by_J; // 1/J
        float sum4;     // sum of 4 terms
        float B;        // dynamic friction coef
        float w;        // omega rad/s
        float w_est;    //
        float w_rpm;
        float w_last;
        float ew;         // observer err of omega
        float et;         // observer err of tload
        float dw_est;     // d(w_est)/dt
        float dtload_est; // d(tload_est)/dt
        float Kt;         // torque const.
        float Te;         // Te = 1.5*npp*psif*iq = Kt * iq.
        float Te_last;
        float tload_est;
        float l1;  // aka. 1-a
        float l2;  //
        float th1; // threshold level high torque2
        float th2; // threshold level low torque1
        int sta;
        float iq0;
        float iq1;
        float iq2;
        float iq_comp;
    } lbg_observer_t;

    void luenburger_observer_init(lbg_observer_t *ob, float J, float B, float Kt, float l1, float l2);
    // input iq * Kt = Te
    // input w = omega_mechenical
    // eval output = Tl
    void luenburger_observer_update(lbg_observer_t *ob, float iq, float w);

    // alfa & beta each axis calc
    typedef struct _flux_linkage
    {
        hpf1_t hpf;
        float Ls;
        float Rs;
        float ts;   // dt
        float phi;  // ialfa * Rs + d(phi_alfa)/dt = ualfa
        float dphi; // dphi/dt
        float phi_filtered;
        float flux; // aka psi_f = integral of dphi of each axis.
    } flux_linkage_t;

    // ts = sample time delta
    void flux_linkage_estimate_init(flux_linkage_t *pfl, float ts, float ls, float rs, float hpf_cutoff);
    // fl: flux context struct
    // i : ialfa or ibeta,
    // u : ualfa or ubeta
    float flux_linkage_estimate(flux_linkage_t *pfl, float i, float u);

    // input cos as alfa, sin as beta.
    float flux_psif_calc(flux_linkage_t *pfl_cos, flux_linkage_t *pfl_sin);

    typedef struct pid_f32
    {
        float p;
        float i;
        float d;
        float input_max_err;
        float pi_coef; // fast for p*i

        float max_out;
        float integral_limit;

        float set;
        float get;

        float err;
        float last_err;

        float pout;
        float iout;
        float dout;
        float out;
        // TODO 标准形式的仿真与学习

        } pid_f32_t;

    typedef struct pid_q15
    {
        int32_t p;
        int32_t i;
        int32_t d;
        // int16_t input_max_err;
        int32_t err_limit;
        int32_t max_out;
        int32_t integral_limit;
        int32_t set;
        int32_t get;
        int32_t err;
        int32_t last_err;

        int32_t pout;
        int32_t iout;
        int32_t dout;
        int32_t out;
    } pid_q15_t;

    void pid_struct_init_f32(
        pid_f32_t *pid,
        float maxout,
        float intergral_limit,

        float kp,
        float ki,
        float kd);

    float pid_calc_parallel_f32(struct pid_f32 *pid, float fdb, float ref);
    float pid_calc_serial_f32(struct pid_f32 *pid, float get, float set);
    float pi_calc_serial_f32(struct pid_f32 *pid, float get, float set);

    /* ================== Q15 pid implement ======================*/

    void pid_struct_init_q15(
        pid_q15_t *pid,
        int32_t maxout,
        int32_t integral_limit,

        int32_t kp,
        int32_t ki,
        int32_t kd);

    int32_t pid_calculate_q15(pid_q15_t *pid, int16_t fdb, int16_t ref);

    float circle_limitation(float *x, float *y, float limit);

#ifdef __cplusplus
}
#endif

#endif