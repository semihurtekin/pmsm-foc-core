#include "foc_math.h"

#include <limits.h>
#include <stddef.h>

#define FOC_Q15_SCALE_I32    (INT32_C(32768))
#define FOC_Q15_SCALE_I64    (INT64_C(32768))

/*
 * Integer square root used by the circular voltage limiter. The restoring
 * algorithm keeps this module independent of floating-point and libm.
 */
static uint32_t Foc_IntegerSqrt(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = UINT32_C(1) << 30U;

    /* Start at the highest base-four position present in the input. */
    while(bit > value)
    {
        bit >>= 2U;
    }

    while(bit != 0U)
    {
        if(value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }

        bit >>= 2U;
    }

    return result;
}

/*
 * Convert one Q1.15 component to an unsigned magnitude without negating it in
 * its original 16-bit type. This also handles INT16_MIN safely.
 */
static uint32_t Foc_Q15Magnitude(foc_q15_t value)
{
    int32_t widened_value = (int32_t)value;
    uint32_t magnitude;

    if(widened_value < 0)
    {
        magnitude = (uint32_t)(-widened_value);
    }
    else
    {
        magnitude = (uint32_t)widened_value;
    }

    return magnitude;
}

/* Narrow a wider intermediate without allowing signed wraparound. */
foc_q15_t Foc_Q15Saturate(int32_t value)
{
    foc_q15_t result;

    if(value > INT16_MAX)
    {
        result = (foc_q15_t)INT16_MAX;
    }
    else if(value < INT16_MIN)
    {
        result = (foc_q15_t)INT16_MIN;
    }
    else
    {
        result = (foc_q15_t)value;
    }

    return result;
}

/* Q1.15 multiplied by Q1.15 gives a Q2.30 intermediate. */
foc_q15_t Foc_Q15Mul(foc_q15_t a, foc_q15_t b)
{
    int32_t product = (int32_t)a * (int32_t)b;

    /*
     * Division is used instead of a signed right shift because shifting a
     * negative signed value is implementation-defined in C.
     */
    return Foc_Q15Saturate(product / FOC_Q15_SCALE_I32);
}

/*
 * Amplitude-invariant Clarke transform for a balanced three-phase system.
 * Alpha is aligned with phase A; phase C is not needed by this reduced form.
 */
Foc_AlphaBeta Foc_Clarke(Foc_Abc current)
{
    Foc_AlphaBeta result;
    int32_t beta_numerator;

    result.alpha = current.a;

    /*
     * The sum can temporarily exceed Q1.15 even when each phase sample is in
     * range, so it is formed in 32 bits before applying 1/sqrt(3).
     */
    beta_numerator =
        (int32_t)current.a +
        (INT32_C(2) * (int32_t)current.b);
    result.beta = Foc_Q15Saturate(
        (beta_numerator * (int32_t)FOC_Q15_INV_SQRT3) /
        FOC_Q15_SCALE_I32);

    return result;
}

/*
 * Rotate alpha-beta into the electrical rotor frame. The selected convention
 * is d = alpha*cos + beta*sin and q = -alpha*sin + beta*cos.
 */
Foc_Dq Foc_Park(Foc_AlphaBeta stationary, Foc_SinCos angle)
{
    Foc_Dq result;
    int64_t d_accumulator;
    int64_t q_accumulator;

    /*
     * Two full-scale Q1.15 products can exceed int32_t when added. Keeping the
     * accumulators in int64_t also makes the corner-case behavior explicit.
     */
    d_accumulator =
        ((int64_t)stationary.alpha * (int64_t)angle.cos) +
        ((int64_t)stationary.beta * (int64_t)angle.sin);
    q_accumulator =
        (-((int64_t)stationary.alpha * (int64_t)angle.sin)) +
        ((int64_t)stationary.beta * (int64_t)angle.cos);

    result.d = Foc_Q15Saturate(
        (int32_t)(d_accumulator / FOC_Q15_SCALE_I64));
    result.q = Foc_Q15Saturate(
        (int32_t)(q_accumulator / FOC_Q15_SCALE_I64));

    return result;
}

/* Apply the inverse rotation used after d-q voltage control. */
Foc_AlphaBeta Foc_InversePark(Foc_Dq rotating, Foc_SinCos angle)
{
    Foc_AlphaBeta result;
    int64_t alpha_accumulator;
    int64_t beta_accumulator;

    alpha_accumulator =
        ((int64_t)rotating.d * (int64_t)angle.cos) -
        ((int64_t)rotating.q * (int64_t)angle.sin);
    beta_accumulator =
        ((int64_t)rotating.d * (int64_t)angle.sin) +
        ((int64_t)rotating.q * (int64_t)angle.cos);

    result.alpha =
        Foc_Q15Saturate(
            (int32_t)(alpha_accumulator / FOC_Q15_SCALE_I64));
    result.beta =
        Foc_Q15Saturate(
            (int32_t)(beta_accumulator / FOC_Q15_SCALE_I64));

    return result;
}

/*
 * Limit the d-q magnitude with one common scale factor. Independent axis
 * clipping would change the requested voltage-vector angle.
 */
bool Foc_LimitVector(Foc_Dq input, foc_q15_t magnitude_limit, Foc_Dq *output)
{
    bool limited = false;

    if(output != NULL)
    {
        if(magnitude_limit <= 0)
        {
            output->d = 0;
            output->q = 0;
            limited = (input.d != 0) || (input.q != 0);
        }
        else
        {
            uint32_t d_magnitude = Foc_Q15Magnitude(input.d);
            uint32_t q_magnitude = Foc_Q15Magnitude(input.q);
            uint32_t magnitude_squared =
                (d_magnitude * d_magnitude) +
                (q_magnitude * q_magnitude);
            uint32_t magnitude = Foc_IntegerSqrt(magnitude_squared);
            uint32_t magnitude_limit_u32 =
                (uint32_t)magnitude_limit;

            *output = input;
            if(magnitude > magnitude_limit_u32)
            {
                int32_t scale_q15 =
                    ((int32_t)magnitude_limit * FOC_Q15_SCALE_I32) /
                    (int32_t)magnitude;

                output->d = Foc_Q15Saturate(
                    ((int32_t)input.d * scale_q15) /
                    FOC_Q15_SCALE_I32);
                output->q = Foc_Q15Saturate(
                    ((int32_t)input.q * scale_q15) /
                    FOC_Q15_SCALE_I32);
                limited = true;
            }
            else
            {
                /* The requested vector is already inside the limit. */
            }
        }
    }
    else
    {
        /* A null output pointer is reported by the false return value. */
    }

    return limited;
}
