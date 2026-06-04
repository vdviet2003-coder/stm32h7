#include "smo.h"
#include <math.h>

/* Hàm dau vao vùng ch?t d? gi?m chattering */
static inline float sign_function(float x, float deadzone) {
    if (x > deadzone) {
        return 1.0f;
    } else if (x < -deadzone) {
        return -1.0f;
    } else {
        return x / deadzone;
    }
}
static inline float signmoid_function(float x) {
    const float a = 0.01f;   // H? s? d? d?c, có th? tinh ch?nh
    return 2.0f / (1.0f + expf(-a * x)) - 1.0f;
}

/**
 * Kh?i t?o SMO
 * @param smo               con tr? d?n c?u trúc SMO
 * @param Rs                di?n tr? stator (ohm)
 * @param Ls                di?n c?m stator (H)
 * @param psi_f             t? thông nam châm vinh c?u (Wb)
 * @param pole_pairs        s? c?p c?c
 * @param speed_max_rpm     t?c d? co t?i da (vòng/phút)
 * @param speed_min_rpm     t?c d? co t?i thi?u (vòng/phút)
 * @param dt                chu k? l?y m?u FOC (giây)
 */
void SMO_Init(SMO_Handle *smo, float Rs, float Ls, float psi_f, float pole_pairs,
              float speed_max_rpm, float speed_min_rpm, float dt) {
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->psi_f = psi_f;
    smo->pole_pairs = pole_pairs;

    // 1. Tính t?c d? di?n t?i da (rad/s) và h? s? tru?t Ks
    float omega_max = speed_max_rpm * (2.0f * M_PI / 60.0f) * pole_pairs;   // rad/s
    float e_max = omega_max * psi_f;                                         // V
    smo->Ks = 1.5f * e_max;                                                  // V

    // 2. Bo loc thông thap trích xuat suc phan dien
    smo->fc_filter = 1000.0f;                                                 // Hz tang len 1khz
    float wc = 2.0f * M_PI * smo->fc_filter;                                 // rad/s (˜ 3141.6)
    smo->alpha_filter = wc * dt / (1.0f + wc * dt);                          // s? th?c (˜ 0.0728)

    // 3. Tham so PLL voi bang thông fc_pll = 200 Hz// tang len tam khoang 300
    float wn_pll = 2.0f * M_PI * 200.0f;                                     // rad/s (˜ 1256.6)
    smo->Kp_pll = 1.414f * wn_pll;                                           // ˜ 1777
    smo->Ki_pll = wn_pll * wn_pll;                                           // ˜ 1.58e6

    // 4. Kh?i t?o các bi?n tr?ng thái
    smo->ialpha_est = 0.0f;
    smo->ibeta_est = 0.0f;
    smo->ealpha_est = 0.0f;
    smo->ebeta_est = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->pll_integral = 0.0f;

    // 5. Giôi han toc do dien (rad/s)
    smo->omega_max = omega_max;
    smo->omega_min = speed_min_rpm * (2.0f * M_PI / 60.0f) * pole_pairs;   // rad/s
}

/**
 * C?p nh?t SMO (g?i m?i chu k? FOC)
 * @param smo       con tr? SMO
 * @param ualpha    di?n áp alpha (V)
 * @param ubeta     di?n áp beta (V)
 * @param ialpha    dòng do alpha (A)
 * @param ibeta     dòng do beta (A)
 * @param dt        chu k? l?y m?u (giây)
 */
void SMO_Update(SMO_Handle *smo, float ualpha, float ubeta,
                float ialpha, float ibeta, float dt) {
    // 1. Sai lech dòng dien
    float s_alpha = smo->ialpha_est - ialpha;
    float s_beta  = smo->ibeta_est  - ibeta;

    // 2. Ðau ra cua khâu truot (hàm sign voi deadzone 0.1A)
    //float z_alpha = smo->Ks * sign_function(s_alpha, 0.1f);
    //float z_beta  = smo->Ks * sign_function(s_beta, 0.1f);
									
		float z_alpha = smo->Ks  * signmoid_function(s_alpha);
    float z_beta  = smo->Ks  * signmoid_function(s_beta);
									
    // 3. Mô hình dòng dien (Euler)
    float dialpha_dt = (ualpha - smo->Rs * smo->ialpha_est - z_alpha) / smo->Ls;
    float dibeta_dt  = (ubeta  - smo->Rs * smo->ibeta_est  - z_beta)  / smo->Ls;
    smo->ialpha_est += dialpha_dt * dt;
    smo->ibeta_est  += dibeta_dt  * dt;

    // 4. Loc thông thap trích xuat sac phan dien
    smo->ealpha_est = smo->alpha_filter * z_alpha +
                      (1.0f - smo->alpha_filter) * smo->ealpha_est;
    smo->ebeta_est  = smo->alpha_filter * z_beta  +
                      (1.0f - smo->alpha_filter) * smo->ebeta_est;
		// chuan ealpha_est va ebeta_est
		
		smo->E = sqrt(smo->ealpha_est*smo->ealpha_est + smo->ebeta_est*smo->ebeta_est);
		
		smo->E_ealpha_est = smo->ealpha_est / smo->E;
		smo->E_ebeta_est  = smo->ebeta_est  / smo->E;
			
    // 5. PLL uoc luong vi trí và toc do
    float cos_theta = cosf(smo->theta_est);
    float sin_theta = sinf(smo->theta_est);
    //float error = smo->E_ebeta_est * cos_theta - smo->E_ealpha_est* sin_theta;//// 
		float error = -smo->E_ealpha_est * cos_theta - smo->E_ebeta_est * sin_theta;
		
		
    smo->pll_integral += error * dt;
    float omega_pll = smo->Kp_pll * error + smo->Ki_pll * smo->pll_integral;

    // Gioi han toc do (ant-windup)
    if (omega_pll > smo->omega_max) {
        omega_pll = smo->omega_max;
        smo->pll_integral = omega_pll / smo->Ki_pll;
    } else if (omega_pll < -smo->omega_max) {
        omega_pll = -smo->omega_max;
        smo->pll_integral = omega_pll / smo->Ki_pll;
    }
		
    smo->omega_est = omega_pll;
    smo->theta_est += omega_pll * dt;
		
    // Chu?n hóa góc v? [0, 2p)
    while (smo->theta_est >= 2.0f * M_PI) smo->theta_est -= 2.0f * M_PI;
    while (smo->theta_est < 0.0f)         smo->theta_est += 2.0f * M_PI;
}

/**
 * Tr? v? góc di?n dã bù tr? pha b? l?c thông th?p
 * @return  góc di?n (rad)
 */
float SMO_GetCompensatedTheta(SMO_Handle *smo) {
    float wc = 2.0f * M_PI * smo->fc_filter;                  // rad/s
    float phase_delay = atan2f(smo->omega_est, wc);          // rad
    return smo->theta_est + phase_delay;
}

/**
 * Tr? v? t?c d? di?n u?c lu?ng
 * @return  t?c d? di?n (rad/s)
 */
float SMO_GetOmega(SMO_Handle *smo) {
    return smo->omega_est ;
}

/**
 * Tr? v? t?c d? co (RPM)
 * @return  RPM
 */
float SMO_GetSpeedRPM(SMO_Handle *smo) {
    return smo->omega_est * 60.0f / (2.0f * M_PI * smo->pole_pairs);
}