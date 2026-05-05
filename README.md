# STM32H7 Sensorless FOC Motor Control

Firmware điều khiển động cơ PMSM theo phương pháp **Field-Oriented Control (FOC) không cảm biến vị trí (sensorless)** chạy trên vi điều khiển **STM32H723ZGTx**, sử dụng Sliding Mode Observer (SMO) để ước lượng góc và tốc độ rotor.

---

## Mục lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Phần cứng và thông số động cơ](#2-phần-cứng-và-thông-số-động-cơ)
3. [Cấu trúc thư mục](#3-cấu-trúc-thư-mục)
4. [Sơ đồ khối điều khiển](#4-sơ-đồ-khối-điều-khiển)
5. [Luồng thực thi chính](#5-luồng-thực-thi-chính)
6. [Các module và logic chi tiết](#6-các-module-và-logic-chi-tiết)
   - 6.1 [conf.h – Cấu hình toàn cục](#61-confh--cấu-hình-toàn-cục)
   - 6.2 [FOC Transform – Clarke & Park](#62-foc-transform--clarke--park)
   - 6.3 [SVPWM – Space Vector PWM](#63-svpwm--space-vector-pwm)
   - 6.4 [ADC Driver – Đo dòng điện](#64-adc-driver--đo-dòng-điện)
   - 6.5 [Sensor – Hall & Encoder](#65-sensor--hall--encoder)
   - 6.6 [SMO – Sliding Mode Observer](#66-smo--sliding-mode-observer)
   - 6.7 [PID Controller](#67-pid-controller)
   - 6.8 [PI Clamping Controller](#68-pi-clamping-controller)
   - 6.9 [Speed Ramp Profiles](#69-speed-ramp-profiles)
   - 6.10 [UART DMA Library](#610-uart-dma-library)
7. [Máy trạng thái điều khiển (state machine)](#7-máy-trạng-thái-điều-khiển-state-machine)
   - 7.1 [Trạng thái chính (state)](#71-trạng-thái-chính-state)
   - 7.2 [Trạng thái SMO sensorless (state_smo)](#72-trạng-thái-smo-sensorless-state_smo)
8. [Cấu hình Timer và Ngắt](#8-cấu-hình-timer-và-ngắt)
9. [Thông số bộ điều khiển PID](#9-thông-số-bộ-điều-khiển-pid)
10. [Cấu hình Clock hệ thống](#10-cấu-hình-clock-hệ-thống)
11. [Build và nạp firmware](#11-build-và-nạp-firmware)

---

## 1. Tổng quan hệ thống

Hệ thống thực hiện điều khiển FOC hoàn chỉnh gồm 3 vòng điều khiển lồng nhau:

```
Tốc độ đặt (RPM/rad·s⁻¹)
        │
        ▼
  [Vòng tốc độ - PID]
        │  iq_ref
        ▼
  [Vòng dòng iq - PID] ──► vq
  [Vòng dòng id - PID] ──► vd    (id_ref = 0, kiểm soát từ thông)
        │
        ▼
  [Inv. Park] ──► vα, vβ
        │
        ▼
    [SVPWM] ──► TIM1 (PWM 3 pha)
        │
        ▼
     Động cơ
        │
        ▼
  [ADC] ─── iu, iv, iw
  [SMO] ─── θe, ωe  (sensorless)
        │
  [Clarke + Park] ──► id, iq (phản hồi)
```

---

## 2. Phần cứng và thông số động cơ

| Tham số | Giá trị |
|---------|---------|
| Vi điều khiển | STM32H723ZGTx |
| Toolchain | Keil MDK-ARM |
| Điện áp DC bus (VDC) | 60 V |
| Số cặp cực | 4 |
| Điện trở stator (Rs) | 5.83 Ω |
| Điện cảm stator (Ls) | 12.23 mH |
| Từ thông nam châm vĩnh cửu (ψf) | 0.0756 Wb |
| Tần số PWM | 20 kHz |
| Tần số vòng FOC | 40 kHz |
| Cảm biến dòng | LA-25-NP (Hall effect) |
| Điện trở đo | 120 Ω |
| Tỉ số biến đổi | 1:1000 (3 mA/A) |

### Phân công GPIO Hall sensor

| Tín hiệu | GPIO | Pin |
|----------|------|-----|
| Hall U | GPIOD | PIN_12 |
| Hall V | GPIOD | PIN_13 |
| Hall W | GPIOD | PIN_14 |

---

## 3. Cấu trúc thư mục

```
no encoder/
└── GPIO/
    ├── Core/
    │   ├── Inc/           # Header STM32 HAL (adc.h, tim.h, gpio.h, ...)
    │   └── Src/
    │       ├── main.c     # Vòng lặp chính, FOC ISR, khởi tạo
    │       ├── adc.c      # Cấu hình ADC HAL (injected mode)
    │       ├── tim.c      # Cấu hình Timer HAL
    │       ├── gpio.c     # Cấu hình GPIO
    │       ├── dma.c      # Cấu hình DMA
    │       └── usart.c    # Cấu hình UART
    ├── MDK-ARM/           # Mã nguồn ứng dụng + project Keil
    │   ├── conf.h         # *** Toàn bộ thông số hệ thống ***
    │   ├── foc_transform.c/h   # Clarke, Park, Inv.Park transforms
    │   ├── svpwm.c/h           # SVPWM 6 sector + bù dead-time
    │   ├── adc_driver.c/h      # Đọc dòng từ LA-25-NP
    │   ├── sensor.c/h          # Hall sensor + Encoder ABZ
    │   ├── smo.c/h             # Sliding Mode Observer
    │   ├── pid.c/h             # PID Tustin + anti-windup
    │   ├── pi_control.c/h      # PI clamping
    │   ├── scurve_ramp.c/h     # S-curve tốc độ
    │   ├── multi_ramp.c/h      # Multi-segment ramp
    │   └── uart_dma_lib.c/h    # UART DMA gửi/nhận
    └── Drivers/
        ├── CMSIS/         # CMSIS core, DSP library
        └── STM32H7xx_HAL_Driver/  # HAL drivers
```

---

## 4. Sơ đồ khối điều khiển

```
                    ┌─────────────────────────────────────────────┐
                    │               FOC LOOP (40 kHz - TIM6)      │
                    │                                             │
speed_ref ──►[Ramp]──►[PID_speed]──► iq_ref                     │
                                        │                        │
              id_ref=0 ───────────────► [PID_id] ──► vd          │
                    iq_ref ──────────► [PID_iq] ──► vq           │
                                                     │           │
                                              [Inv.Park(θe)]     │
                                                     │           │
                                              [SVPWM] ──► TIM1   │
                                                                 │
    ADC ──► [Clarke] ──► iα,iβ ──► [Park(θe)] ──► id, iq        │
                  │                                              │
                  └──► [SMO] ──► θe, ωe (sensorless feedback)   │
                    └─────────────────────────────────────────────┘
```

---

## 5. Luồng thực thi chính

### `main()` – Khởi tạo một lần

```
1. MPU_Config()           – Cấu hình vùng bộ nhớ (DMA non-cacheable)
2. SystemClock_Config()   – PLL: HSE → 440 MHz SYSCLK
3. MX_*_Init()            – Khởi tạo tất cả ngoại vi HAL
4. SVPWM_Init()           – Cài đặt TIM1 center-aligned PWM
5. SVPWM_Start()          – Bật PWM 3 pha + complementary
6. ADC_Driver_Init()      – Khởi động ADC injected mode
7. ADC_Driver_CalibrateOffset() – Lấy 100,000 mẫu để tính offset
8. Hall_Sensor_Init()     – Đọc trạng thái Hall ban đầu
9. Encoder_Sensor_Init()  – Cấu hình TIM2 (encoder mode)
10. UART_DMA_Init()       – Bật UART1 DMA RX/TX
11. PID_Init(x3)          – Khởi tạo 3 vòng PID (Id, Iq, Speed)
12. SMO_Init()            – Khởi tạo Sliding Mode Observer
13. while(1) {}           – Vòng lặp trống (mọi xử lý qua ISR)
```

---

## 6. Các module và logic chi tiết

### 6.1 `conf.h` – Cấu hình toàn cục

File duy nhất chứa tất cả hằng số hệ thống. Thay đổi thông số ở đây sẽ tác động toàn bộ firmware.

```c
#define POLE_PAIRS          4.0f
#define MOTOR_PWM_FREQ      20000.0f   // Hz
#define MOTOR_SPEED_CALC_FREQ 40000.0f // Hz (vòng FOC)
#define MOTOR_RS            5.83f      // Ω
#define MOTOR_LS            0.01223f   // H
#define MOTOR_PSI_F         0.075595f  // Wb
#define VDC_BUS             60.0f      // V
```

---

### 6.2 FOC Transform – Clarke & Park

**File**: `foc_transform.c/h`

Thực hiện các phép biến đổi tọa độ cơ bản của FOC:

#### Clarke Transform (3 pha → αβ cố định)
$$i_\alpha = \frac{2}{3}i_u - \frac{1}{3}i_v - \frac{1}{3}i_w$$
$$i_\beta = \frac{1}{\sqrt{3}}(i_v - i_w)$$

#### Park Transform (αβ → dq quay theo rotor)
$$i_d = i_\alpha \cos\theta_e + i_\beta \sin\theta_e$$
$$i_q = -i_\alpha \sin\theta_e + i_\beta \cos\theta_e$$

#### Inverse Park (dq → αβ)
$$v_\alpha = v_d \cos\theta_e - v_q \sin\theta_e$$
$$v_\beta = v_d \sin\theta_e + v_q \cos\theta_e$$

> **Lưu ý**: Dùng biến đổi **amplitude-invariant** (không nhân 2/3), phù hợp với điều khiển dòng trực tiếp.

---

### 6.3 SVPWM – Space Vector PWM

**File**: `svpwm.c/h`

Thuật toán SVPWM 6 sector, center-aligned, với bù dead-time tự động.

#### Các bước tính toán:

1. **Xác định sector** – Dùng phương pháp dấu (Sign Method):
   ```
   U1 = Vβ
   U2 = (√3/2)·Vα - 0.5·Vβ
   U3 = -(√3/2)·Vα - 0.5·Vβ
   N = sign(U1) + 2·sign(U2) + 4·sign(U3)  → tra bảng sector_table[]
   ```

2. **Tính thời gian vector X, Y, Z** (chuẩn hóa):
   $$X = \frac{\sqrt{3} V_\beta}{V_{DC}}, \quad Y = \frac{1.5V_\alpha + 0.5\sqrt{3}V_\beta}{V_{DC}}, \quad Z = \frac{1.5V_\alpha - 0.5\sqrt{3}V_\beta}{V_{DC}}$$

3. **Gán Tx, Ty theo sector** và xử lý overmodulation (chuẩn hóa nếu Tx+Ty > 1)

4. **Tính duty cycle 3 pha**:
   $$T_a = 0.5 + \frac{V_\alpha}{V_{DC}}, \quad T_b = 0.5 + \frac{-0.5V_\alpha + \frac{\sqrt{3}}{2}V_\beta}{V_{DC}}$$

5. **Bù dead-time** (tự động theo chiều dòng điện):
   - Dead-time: 0.484 µs (= 133 chu kỳ / 275 MHz)
   - Điện áp diode: 0.8 V
   - Bù dương/âm tùy thuộc vào dấu dòng pha

6. **Ghi CCR** vào TIM1 (Center-aligned mode)

---

### 6.4 ADC Driver – Đo dòng điện

**File**: `adc_driver.c/h`

Đọc dòng điện từ 3 cảm biến **LA-25-NP** (Hall effect, ratio 1:1000) qua ADC injected mode đồng bộ với PWM.

#### Công thức chuyển đổi:
$$I_P = \frac{2 \cdot V_{ADC} + V_{offset}}{-R_m \cdot n} = \frac{2 \cdot V_{ADC} - 2.5}{-120 \times 0.003}$$

Trong đó:
- $V_{ADC}$: điện áp đọc từ ADC (0..3.3V)
- $V_{offset}$ = -2.5 V (điểm zero của cảm biến)
- $R_m$ = 120 Ω (điện trở đo)
- $n$ = 0.003 (tỉ số biến đổi = 3mA/A)

#### Hiệu chỉnh offset:
Khi khởi động, `ADC_Driver_CalibrateOffset()` lấy **100,000 mẫu** ở trạng thái không tải để xác định giá trị offset DC, bù vào các lần đọc sau.

---

### 6.5 Sensor – Hall & Encoder

**File**: `sensor.c/h`

#### Hall Sensor
- 3 bit Hall (U, V, W) → 6 bước (step 1..6)
- Mỗi bước tương ứng góc điện 60°
- `HallSensor_Update()` gọi trong `HAL_TIM_IC_CaptureCallback` (TIM4)
- `Sensor_Get_Electrical_Angle_Hall()` trả về góc điện (rad)

| Step | Góc điện |
|------|----------|
| 1    | 0°       |
| 2    | 60°      |
| 3    | 120°     |
| 4    | 180°     |
| 5    | 240°     |
| 6    | 300°     |

#### Encoder ABZ
- TIM2 (32-bit) ở chế độ encoder quadrature
- 2500 PPR × 4 = 10,000 counts/vòng cơ
- `EncoderSensor_Update()` gọi trong TIM3 ISR (1 kHz)
- Hỗ trợ 3 phương pháp tính tốc độ:
  - **M method**: đếm xung trong thời gian cố định (tốc độ cao)
  - **T method**: đo thời gian giữa xung (tốc độ thấp)
  - **M/T method**: kết hợp hai phương pháp
  - **AUTO**: tự động chọn theo tốc độ

#### Bù trễ góc (Angle Prediction)
Trong vòng FOC, góc điện được dự đoán trước để bù trễ tính toán và ADC:
$$\theta_{pred} = \theta_{enc} + \omega_e \cdot (T_s + t_{delay})$$
Trong đó $t_{delay}$ = 3.87 µs (trễ ADC + tính toán).

---

### 6.6 SMO – Sliding Mode Observer

**File**: `smo.c/h`

Observer phi tuyến để ước lượng góc và tốc độ rotor **không cần encoder vật lý**. Triển khai theo mô hình dòng điện stator trong hệ tọa độ tĩnh αβ, kết hợp bộ lọc thông thấp và PLL để trích xuất sức phản điện (back-EMF).

---

#### Nguyên lý tổng quát

```
  [u_α, u_β]           [i_α, i_β]
       │                    │
       ▼                    │
  ┌─────────────────┐       │
  │  Mô hình dòng   │◄──────┘   (sai lệch → z_α, z_β)
  │  điện stator    │
  │  (Euler)        │──► [î_α, î_β]
  └────────┬────────┘
           │ z_α, z_β (sliding output)
           ▼
     [LPF 500 Hz]──► [ê_α, ê_β]  (back-EMF ước lượng)
           │
           ▼
       [PLL 200 Hz]──► θ_est, ω_est
           │
           ▼
   [Bù pha LPF]──► θ_compensated
```

---

#### Bước 1 – Hàm trượt (Sliding function) với deadzone

Để giảm chattering (dao động cao tần tại mặt trượt), hàm sign được thay bằng hàm tuyến tính trong vùng deadzone:

```c
static inline float sign_function(float x, float deadzone) {
    if      (x >  deadzone) return  1.0f;
    else if (x < -deadzone) return -1.0f;
    else                    return  x / deadzone;  // tuyến tính trong [-dz, +dz]
}
```

Ngõ ra khuếch đại trượt:

$$
z_{\alpha} = K_s \cdot \text{sign}(i_{\alpha,est} - i_{\alpha},\ 0.1\,\text{A})
$$

$$
z_{\beta} = K_s \cdot \text{sign}(i_{\beta,est} - i_{\beta},\ 0.1\,\text{A})
$$

Hệ số trượt $K_s$ chọn đủ lớn để đảm bảo điều kiện trượt:
$$K_s = 1.5 \cdot \omega_{max,elec} \cdot \psi_f \quad [\text{V}]$$

Ví dụ: $\omega_{max}$ = 3000 RPM × 4 pp × 2π/60 ≈ 1257 rad/s → $K_s = 1.5 \times 1257 \times 0.0756 \approx 142\,\text{V}$

---

#### Bước 2 – Mô hình dòng điện stator (tích phân Euler)

$$\hat{i}_{\alpha}[k+1] = \hat{i}_{\alpha}[k] + \frac{T_s}{L_s}\left(u_\alpha - R_s \hat{i}_{\alpha}[k] - z_\alpha\right)$$
$$\hat{i}_{\beta}[k+1]  = \hat{i}_{\beta}[k]  + \frac{T_s}{L_s}\left(u_\beta  - R_s \hat{i}_{\beta}[k]  - z_\beta\right)$$

Khi đạt chế độ trượt ($s_\alpha \to 0$, $s_\beta \to 0$), $z_\alpha \approx e_\alpha$ và $z_\beta \approx e_\beta$ (sức phản điện thực).

---

#### Bước 3 – Lọc thông thấp trích xuất back-EMF

Bộ lọc IIR bậc 1 (forward Euler) với $f_c$ = 500 Hz:

$$
\hat{e}_{\alpha}[k] = \alpha_f \cdot z_{\alpha} + (1-\alpha_f) \cdot \hat{e}_{\alpha}[k-1]
$$

$$
\hat{e}_{\beta}[k] = \alpha_f \cdot z_{\beta} + (1-\alpha_f) \cdot \hat{e}_{\beta}[k-1]
$$

Hệ số lọc:
$$\alpha_f = \frac{\omega_c T_s}{1 + \omega_c T_s}, \quad \omega_c = 2\pi \times 500 \approx 3141.6\,\text{rad/s}$$

Tại $T_s = 25\,\mu\text{s}$: $\alpha_f \approx 0.0728$

> **Lưu ý**: Bộ lọc thông thấp gây trễ pha phụ thuộc tốc độ, được bù lại ở bước 5.

---

#### Bước 4 – PLL ước lượng góc và tốc độ

Tín hiệu lỗi PLL (cross-product):

$$
\varepsilon = \hat{e}_{\beta} \cos\hat{\theta} - \hat{e}_{\alpha} \sin\hat{\theta}
$$

Khi hội tụ: $\hat{\theta} \to \theta_{real}$ → $\varepsilon \to 0$.

Bộ điều khiển PI của PLL:
$$\hat{\omega} = K_{p,PLL} \cdot \varepsilon + K_{i,PLL} \cdot \int \varepsilon \, dt$$

Tích phân góc (Euler):
$$\hat{\theta}[k+1] = \hat{\theta}[k] + \hat{\omega} \cdot T_s$$

Chuẩn hóa về $[0, 2\pi)$ sau mỗi bước.

**Anti-windup tốc độ**: khi $|\hat{\omega}| > \omega_{max}$, giới hạn cứng và reset tích phân:
```c
if (omega_pll > smo->omega_max) {
    omega_pll = smo->omega_max;
    smo->pll_integral = omega_pll / smo->Ki_pll;   // back-calculation
}
```

**Thông số PLL** (thiết kế theo $\zeta = 1/\sqrt{2}$, $f_n$ = 200 Hz):

| Tham số | Công thức | Giá trị |
|---------|-----------|---------|
| $\omega_n$ | $2\pi \times 200$ | 1256.6 rad/s |
| $K_{p,PLL}$ | $\sqrt{2} \cdot \omega_n$ | ≈ 1777 |
| $K_{i,PLL}$ | $\omega_n^2$ | ≈ 1.58 × 10⁶ |

---

#### Bước 5 – Bù pha bộ lọc thông thấp (`SMO_GetCompensatedTheta`)

Bộ lọc thông thấp 500 Hz gây trễ pha phụ thuộc tốc độ. Góc thực được bù động:
$$\phi_{delay} = \arctan\!\left(\frac{\hat{\omega}}{\omega_c}\right)$$
$$\theta_{comp} = \hat{\theta} + \phi_{delay}$$

```c
float SMO_GetCompensatedTheta(SMO_Handle *smo) {
    float wc = 2.0f * M_PI * smo->fc_filter;
    float phase_delay = atan2f(smo->omega_est, wc);
    return smo->theta_est + phase_delay;
}
```

> Khác với xấp xỉ cố định $+\pi/2$, cách này bù chính xác ở mọi tốc độ: khi $\hat{\omega} \ll \omega_c$ thì $\phi \to 0$; khi $\hat{\omega} \gg \omega_c$ thì $\phi \to \pi/2$.

---

#### Tóm tắt cấu trúc `SMO_Handle`

| Trường | Ý nghĩa | Đơn vị |
|--------|---------|--------|
| `Rs`, `Ls`, `psi_f` | Thông số điện động cơ | Ω, H, Wb |
| `pole_pairs` | Số cặp cực | – |
| `Ks` | Hệ số khuếch đại trượt | V |
| `fc_filter`, `alpha_filter` | Bộ lọc LPF 500 Hz | Hz, – |
| `Kp_pll`, `Ki_pll` | Gains PI của PLL | – |
| `ialpha_est`, `ibeta_est` | Dòng ước lượng | A |
| `ealpha_est`, `ebeta_est` | Back-EMF ước lượng | V |
| `theta_est` | Góc điện ước lượng | rad |
| `omega_est` | Tốc độ điện ước lượng | rad/s |
| `pll_integral` | Tích phân PLL | – |
| `omega_max`, `omega_min` | Giới hạn tốc độ điện | rad/s |

---

#### API

| Hàm | Mô tả |
|-----|-------|
| `SMO_Init(...)` | Khởi tạo, tự tính Ks, α_filter, Kp/Ki_pll |
| `SMO_Update(smo, uα, uβ, iα, iβ, dt)` | Gọi mỗi chu kỳ FOC (40 kHz) |
| `SMO_GetCompensatedTheta(smo)` | Trả về góc đã bù trễ LPF (rad) |
| `SMO_GetOmega(smo)` | Trả về tốc độ điện (rad/s) |
| `SMO_GetSpeedRPM(smo)` | Trả về tốc độ cơ (RPM) |

---

### 6.7 PID Controller

**File**: `pid.c/h`

Bộ điều khiển PID rời rạc dùng **Tustin (bilinear) discretization** để bảo toàn tính ổn định.

#### Rời rạc hóa Tustin:
$$u[k] = u[k-1] + \alpha \cdot e[k] + \beta \cdot e[k-1] + \gamma \cdot e[k-2]$$

Trong đó:
$$\alpha = K_p + \frac{K_i T}{2} + \frac{2K_d}{T}, \quad \beta = K_i T - \frac{4K_d}{T}, \quad \gamma = -K_p + \frac{K_i T}{2} + \frac{2K_d}{T}$$

#### Anti-windup:
Khi ngõ ra bão hòa (|u| = u_max), bộ tích phân tạm dừng (conditional integration).

---

### 6.8 PI Clamping Controller

**File**: `pi_control.c/h`

Bộ PI đơn giản với clamping (giới hạn cứng ngõ ra):
$$I_{prev} = I_{prev} + K_i \cdot T_s \cdot e$$
$$u = \text{clamp}(K_p \cdot e + I_{prev}, U_{min}, U_{max})$$

Dùng làm bộ điều khiển dự phòng thay thế cho PID trong trường hợp cần tuning nhanh.

---

### 6.9 Speed Ramp Profiles

#### S-Curve Ramp (`scurve_ramp.c/h`)
Tạo profile tốc độ dạng chữ S để tránh thay đổi đột ngột:
- Tham số: gia tốc tăng, gia tốc giảm, hằng số thời gian lọc τ
- Lọc exponential qua linear ramp → output mượt mà

#### Multi-Segment Ramp (`multi_ramp.c/h`)
Gia tốc khác nhau ở các vùng tốc độ khác nhau:
- Dưới ngưỡng (`threshold`): dùng `AccUp_1`, `AccDown_1` (gia tốc thấp vùng thấp tốc)
- Trên ngưỡng: dùng `AccUp_2`, `AccDown_2` (gia tốc cao vùng cao tốc)

Cấu hình mặc định: `threshold=5`, `up1=10`, `up2=10`, `down1=20`, `down2=20` (rad/s²)

---

### 6.10 UART DMA Library

**File**: `uart_dma_lib.c/h`

Giao tiếp UART1 bằng DMA không chặn (non-blocking):
- **TX**: Gửi chuỗi/dữ liệu qua DMA, không chờ
- **RX**: Nhận bằng DMA + IDLE interrupt để phát hiện kết thúc frame
- Hỗ trợ callback: `UART_DMA_RegisterRxCallback()` để xử lý dữ liệu nhận

---

## 7. Máy trạng thái điều khiển (State Machine)

### 7.1 Trạng thái chính (`state`)

Biến `state` điều khiển chế độ hoạt động của vòng FOC (TIM6 ISR, 40kHz):

```
┌─────────────────────────────────────────────────────────────────────┐
│  state = 0  │  DỪNG – PWM output = 0, reset SMO, tắt LED          │
├─────────────────────────────────────────────────────────────────────┤
│  state = 1  │  ALIGN – Căn chỉnh rotor:                           │
│             │  · InvPark(Id=0.8A, Iq=0, θ=0) → vα, vβ            │
│             │  · Reset encoder sau khi căn                         │
├─────────────────────────────────────────────────────────────────────┤
│  state = 2  │  CLOSED LOOP với Encoder:                           │
│             │  · Clarke → Park(θ_encoder_predicted)               │
│             │  · PID_speed → iq_ref                               │
│             │  · PID_iq, PID_id → vq, vd                         │
│             │  · InvPark → SVPWM                                  │
├─────────────────────────────────────────────────────────────────────┤
│  state = 3  │  OPEN LOOP điện áp:                                 │
│             │  · Dùng vd, vq đặt thủ công                        │
│             │  · InvPark(vd, vq, θ_encoder) → SVPWM              │
├─────────────────────────────────────────────────────────────────────┤
│  state = 4  │  SENSORLESS FOC (SMO):                             │
│             │  · 4 sub-state: ALIGN → OPENLOOP → SWITCH → RUN   │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2 Trạng thái SMO sensorless (`state_smo`)

Khi `state = 4`, máy trạng thái con `state_smo` quản lý quá trình khởi động sensorless:

```
       ┌─────────┐  ss_align_timer ≥ 1s   ┌────────────┐
  ───► │ SS_ALIGN│ ──────────────────────► │ SS_OPENLOOP│
       │         │                         │            │
       │Id=0.8A  │                         │Tăng ω mỗi  │
       │Iq=0     │                         │100rad/s²  │
       │θe = 0   │                         │            │
       └─────────┘                         └──────┬─────┘
                                                  │ ω ≥ ω_min
                                                  ▼
                                           ┌────────────┐
                                           │ SS_SWITCH  │
                                           │            │
                                           │Chờ SMO hội │
                                           │tụ: |î-i|  │
                                           │  < 0.3A   │
                                           └──────┬─────┘
                                                  │ hội tụ
                                                  ▼
                                           ┌────────────┐
                                           │  SS_RUN    │
                                           │            │
                                           │θe = SMO    │
                                           │ωe = SMO    │
                                           │PID_speed   │
                                           └────────────┘
```

#### Chi tiết từng sub-state:

| Sub-state | Góc θe | Dòng đặt | Điều kiện chuyển |
|-----------|--------|----------|-----------------|
| `SS_ALIGN` | 0 rad (cố định) | Id=0.8A, Iq=0 | timer ≥ 1 giây |
| `SS_OPENLOOP` | Tăng dần (tích phân ω) | Id=0.8A, Iq=0 | ω_openloop ≥ ω_min (900 RPM × 4pp) |
| `SS_SWITCH` | Open-loop giữ nguyên | Id=0.8A, Iq=0 | \|î_α - i_α\| < 0.3A **và** \|î_β - i_β\| < 0.3A |
| `SS_RUN` | `SMO_GetCompensatedTheta()` | Id=0, Iq=PID_speed | – |

---

## 8. Cấu hình Timer và Ngắt

| Timer | Chức năng | Tần số / Mode |
|-------|-----------|---------------|
| **TIM1** | SVPWM 3 pha + complementary | 20 kHz, center-aligned, PWM mode |
| **TIM2** | Encoder ABZ (32-bit) | Encoder interface mode, 10,000 cnt/rev |
| **TIM3** | Cập nhật tốc độ encoder | 1 kHz, base timer interrupt |
| **TIM4** | Hall sensor input capture | Capture trên cạnh lên của Hall |
| **TIM5** | Encoder capture (Z pulse) | Input capture channel 3 |
| **TIM6** | **FOC loop interrupt** | **40 kHz** – tất cả xử lý FOC |
| **TIM12** | Z-pulse (dự phòng) | – |

### ISR quan trọng

```c
// Gọi 40,000 lần/giây – xử lý FOC toàn bộ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 1. Đọc cảm biến (Hall, Encoder, ADC)
        // 2. Chạy máy trạng thái state/state_smo
        // 3. Clarke → Park → PID → InvPark → SVPWM
    }
    if (htim->Instance == TIM3) {
        EncoderSensor_Update();   // Tính tốc độ M/T
    }
}
```

---

## 9. Thông số bộ điều khiển PID

| Vòng | Kp | Ki | Kd | Giới hạn ngõ ra |
|------|----|----|----|-----------------| 
| **PID_id** (dòng d-axis) | 12.0 | 5830 | 0 | ±VDC/√3 = ±34.64 V |
| **PID_iq** (dòng q-axis) | 12.0 | 5830 | 0 | ±VDC/√3 = ±34.64 V |
| **PID_speed** (tốc độ) | 0.0053 | 2.37 | 0 | ±2.8 A |

> **Thiết kế vòng dòng**: Kp và Ki được tính theo phương pháp pole-zero cancellation:  
> $K_p = \frac{L_s}{\tau_{cl} \cdot T_s}$, $K_i = \frac{R_s}{\tau_{cl} \cdot T_s}$  
> với băng thông đóng $f_{cl}$ ~ 1 kHz.

---

## 10. Cấu hình Clock hệ thống

```
HSE (8 MHz) → PLL1:
  PLLM = 2  → VCI = 4 MHz
  PLLN = 44 → VCO = 176 MHz × 2 = 352 MHz  (wide VCO)
  PLLP = 1  → SYSCLK = 440 MHz
  AHB  /2   → HCLK  = 220 MHz
  APB1 /2   → PCLK1 = 110 MHz  (TIM2, TIM3, TIM4, TIM5, TIM6, TIM12)
  APB2 /2   → PCLK2 = 110 MHz  (TIM1, USART1)
```

- **Điện áp lõi**: Scale 0 (hiệu suất cao nhất)
- **Flash latency**: 3 wait states

---

## 11. Build và nạp firmware

### Yêu cầu
- **Keil MDK-ARM** v5.38 trở lên
- **STM32H7 Device Family Pack** (DFP)
- **ST-Link** hoặc J-Link debugger
- **CMSIS-DSP Library** (đã bao gồm trong `Drivers/CMSIS/DSP/`)

### Build
1. Mở `GPIO/MDK-ARM/GPIO.uvprojx` trong Keil uVision
2. Chọn target **GPIO**
3. Build: `F7` hoặc `Project → Build Target`

### Nạp và Debug
1. Kết nối ST-Link với board STM32H7
2. Flash: `Flash → Download` hoặc `F8`
3. Debug: `Debug → Start/Stop Debug Session` (`Ctrl+F5`)

### UART Monitor
- **UART1**, **115200 baud**, 8N1
- Khi khởi động, board gửi: `"UART DMA with IDLE+HT/TC ready\r\n"`
- Có thể gửi lệnh để thay đổi `state` và `speed_ref_rads`

---

## Ghi chú kỹ thuật

- **Dead-time**: 133 chu kỳ clock (≈ 0.484 µs tại 275 MHz APB) được bù tự động theo chiều dòng
- **ADC Injected mode**: ADC triggered đồng bộ với PWM (đỉnh counter TIM1) để lấy mẫu tại điểm tốt nhất (giữa thời gian dẫn)
- **Câu 3 pha từ 2 cảm biến**: `iw = -iu - iv` (định luật Kirchhoff), chỉ cần 2 cảm biến LA-25-NP vật lý
- **SMO ω_min**: 900 RPM × 4 cặp cực × 2π/60 ≈ 376 rad/s điện – tốc độ tối thiểu để SMO hội tụ ổn định
- **MPU**: Cấu hình vùng bộ nhớ DMA là non-cacheable để tránh data coherency issue với DMA
