
#include "LPF.h"

/*
 * Initialize first-order low-pass filter
 *
 * fc: cut-off frequency in Hz
 * ts: sampling period in second
 *
 * Equation:
 * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 *
 * alpha = Ts / (Tau + Ts)
 * Tau   = 1 / (2*pi*fc)
 */
void LPF_FirstOrder_Init(LPF_FirstOrder_t *lpf, float fc, float ts)
{
    float tau;

    if (lpf == 0)
    {
        return;
    }

    if (fc <= 0.0f)
    {
        fc = 1.0f;
    }

    if (ts <= 0.0f)
    {
        ts = 0.0001f;
    }

    tau = 1.0f / (2.0f * M_PI * fc);

    lpf->fc = fc;
    lpf->ts = ts;
    lpf->alpha = ts / (tau + ts);
    lpf->y = 0.0f;
    lpf->initialized = 0;
}


/*
 * Run low-pass filter
 */
float LPF_FirstOrder_Update(LPF_FirstOrder_t *lpf, float x)
{
    if (lpf == 0)
    {
        return 0.0f;
    }

    if (!isfinite(x))
    {
        return lpf->y;
    }

    if (lpf->initialized == 0)
    {
        lpf->y = x;
        lpf->initialized = 1;
        return lpf->y;
    }

    lpf->y = lpf->y + lpf->alpha * (x - lpf->y);

    return lpf->y;
}


/*
 * Reset filter output
 */
void LPF_FirstOrder_Reset(LPF_FirstOrder_t *lpf, float value)
{
    if (lpf == 0)
    {
        return;
    }

    lpf->y = value;
    lpf->initialized = 1;
}