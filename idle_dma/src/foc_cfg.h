
#define FPVMOTO // sensorless motor



// =============================== 经典电机参数 ===============================
#define M11_48_PSIF (0.0267) // wb vabpp = 64V, fe=220hz.
#define M11L_RS (0.15f)      // 100hz测得0.3Ohm
#define M11L_LS (300e-6f)    // 1khz 测各位置取均值约600uHen
#define M15L_RS (0.09f)      // 100hz测得0.18hm
#define M15L_LS (150e-6f)    // 100hz 测各位置取均值约300uHen
#define M11H30_RS (12.8f)   // ohm
#define M11H30_LS (0.031f)  // Hen
#define M11H30_KT (3.617)   // Nm/Apeak from matlab ke = 439Vpeak/krpm
#define M11H30_PSIF (0.163) // unit Wb, 439Vabpeak/krpm.

#define DJI2312_RS (0.5f)    // ohm
#define DJI2312_LS (110e-6f) // Hen
#define DJI2312_POLE_PAIR (7)
#define DJI2312_KV (960)    // krpm/V
#define DJI2312_KE (60/6.28f/DJI2312_KV)  // Vpeak/rpm
// #define DJI2312_KT (0.042f)  // Nm/Apeak from matlab
#define DJI2312_PSIF (DJI2312_KE/DJI2312_POLE_PAIR) // unit Wb, 64Vabpp/220rpm

#define FPVMOTO_RS (0.5f)    // ohm
#define FPVMOTO_LS (555e-6f) // Hen
#define FPVMOTO_KT (0.042f)  // Nm/Apeak from matlab
#define FPVMOTO_KV (1500)    // krpm/V
#define FPVMOTO_KE (60/6.28f/FPVMOTO_KV)  // Vpeak/rpm
#define FPVMOTO_POLE_PAIR (7)
#define FPVMOTO_PSIF (FPVMOTO_KE/FPVMOTO_POLE_PAIR) // unit Wb, 64Vabpp/220rpm


// =============================== 经典电机参数 ===============================

#ifdef FPVMOTO
#define POLE_PAIR (7)
#define PHASE_RS FPVMOTO_RS
#define PHASE_LS FPVMOTO_LS
#endif

#define CPU_FREQ_MHZ (120)
#define PWM_FREQ (20000u) // also = freq of foc.
#define PWM_CKTIM (CPU_FREQ_MHZ * 1000000u)
#define PWM_ARR (PWM_CKTIM / 2 / PWM_FREQ) //  = 3000
// #define SIX_STEP_ARR        PWM_ARR//(PWM_CKTIM / PWM_PSC / PWM_FREQ) // 6 step close loop for current. also need sample at pwm open
#define FOC_FREQ PWM_FREQ
#define SPEED_CTRL_FREQ (1000)
#define POSITION_CTRL_FREQ (1000)
#define CURRENT_CTRL_FREQ FOC_FREQ

#define VBUS (12)
#define VBUS_BASE (12 / 1.732)
#define R_SHUNT (0.01) // 10mR
#define IPHASE_GAIN (5)//51k vs 5k
#define IBUS_RSHUNT (0.01) // 10mR
#define IBUS_GAIN (20)
#define RES_VCC 3900 // kohm both for vbus and vac
#define RES_GND 10   // kohm
#define VDC_GAIN    ((RES_VCC+RES_GND)/RES_GND) //vbus sense gain coef, aka vbus=vadc*5

#define IBASE (1.650 / R_SHUNT / IPHASE_GAIN) //板子能采样到的最大电流值33A
// const static float _IBASE = IBASE;
#define MA2INORM (0.001 * (1.0f / IBASE))
#define I_NORM2REAL (IBASE)
#define I_REAL2NORM (1.f / IBASE) // 0.013
#define Q15TONORM (1.f / 65536)
#define _2PI_U16 (10435) // 将2pi映射到65536
// const float ADC2NORM =  I_ADC2NORM;//(1.f/1241);
#define IBUS_OCP_LIMIT (5000)

#define T_SAMPLE (50e-6)
// #define F_COEF          (float)(1-PHASE_RS*T_SAMPLE/PHASE_LS)
// #define G_COEF          (float)(T_SAMPLE / PHASE_LS)
#define CURRENT_LOOP_BANDWIDTH (100)                                       // Wc, unit rad/s
#define KP_SERIAL (CURRENT_LOOP_BANDWIDTH * PHASE_LS * (IBASE / VBUS_BASE)) // wc电流环带宽等于1k
#define KI_SERIAL (PHASE_RS / PHASE_LS * T_SAMPLE)                          // 0.01

#define ALIGN_VQ (0.05)                       // use vq , theta = -90.
#define ALIGN_THETA ((49152 / 65536) * 6.28) //
#define ALIGN_IQ_Q15 (2000)                  // 6A align
#define ALIGN_IQ_F32 (0.1f / IBASE)          // 3A align
#define ALIGN_TIME_MS 991000                   // ms