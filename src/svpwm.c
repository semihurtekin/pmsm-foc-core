#include "svpwm.h"

#include "foc_math.h"

#define FOC_DUTY_CENTER_I32     (INT32_C(16384))
#define FOC_DUTY_FULL_I32       (INT32_C(32768))
#define FOC_DUTY_FULL_U32       (UINT32_C(65535))
#define FOC_Q15_SCALE_I32       (INT32_C(32768))
#define FOC_SQRT3_Q15_I32       (INT32_C(56756))

/* Map the internal [0, 32768] duty interval to the full uint16_t range. */
static foc_duty_uq16_t Foc_NormalizedDuty(int32_t duty_q15)
{
    uint32_t duty;

    if(duty_q15 <= 0)
    {
        duty = 0U;
    }
    else if(duty_q15 >= FOC_DUTY_FULL_I32)
    {
        duty = FOC_DUTY_FULL_U32;
    }
    else
    {
        /* Half-divisor addition gives nearest-integer rounding. */
        duty =
            (((uint32_t)duty_q15 * FOC_DUTY_FULL_U32) +
             UINT32_C(16384)) /
            UINT32_C(32768);
    }

    return (foc_duty_uq16_t)duty;
}

/*
 * Determine the 60-degree sector without reconstructing an angle. The sector
 * boundaries are beta = +/-sqrt(3)*alpha.
 */
static uint8_t Foc_SvpwmSector(Foc_AlphaBeta voltage)
{
    uint8_t sector;

    if((voltage.alpha == 0) && (voltage.beta == 0))
    {
        /* The zero vector has no unique direction. */
        sector = 0U;
    }
    else
    {
        int32_t sqrt3_alpha =
            ((int32_t)voltage.alpha * FOC_SQRT3_Q15_I32) /
            FOC_Q15_SCALE_I32;
        int32_t beta = voltage.beta;

        if(beta >= 0)
        {
            if(voltage.alpha >= 0)
            {
                if(sqrt3_alpha >= beta)
                {
                    sector = 1U;
                }
                else
                {
                    sector = 2U;
                }
            }
            else
            {
                if(-sqrt3_alpha >= beta)
                {
                    sector = 3U;
                }
                else
                {
                    sector = 2U;
                }
            }
        }
        else if(voltage.alpha < 0)
        {
            if(-sqrt3_alpha >= -beta)
            {
                sector = 4U;
            }
            else
            {
                sector = 5U;
            }
        }
        else
        {
            if(sqrt3_alpha >= -beta)
            {
                sector = 6U;
            }
            else
            {
                sector = 5U;
            }
        }
    }

    return sector;
}

/*
 * Generate phase duties with min-max common-mode injection. This is
 * algebraically equivalent to sector/T1/T2 SVPWM in the linear region.
 */
Foc_SvpwmOutput Foc_Svpwm(Foc_AlphaBeta voltage)
{
    Foc_SvpwmOutput result;
    int32_t phase_a;
    int32_t phase_b;
    int32_t phase_c;
    int32_t maximum;
    int32_t minimum;
    int32_t common_mode;

    /* Inverse Clarke reconstruction of the three phase references. */
    phase_a = voltage.alpha;
    phase_b =
        (-(int32_t)voltage.alpha / INT32_C(2)) +
        (((int32_t)voltage.beta * FOC_Q15_SQRT3_BY_2) /
         FOC_Q15_SCALE_I32);
    phase_c =
        (-(int32_t)voltage.alpha / INT32_C(2)) -
        (((int32_t)voltage.beta * FOC_Q15_SQRT3_BY_2) /
         FOC_Q15_SCALE_I32);

    maximum = phase_a;
    if(phase_b > maximum)
    {
        maximum = phase_b;
    }
    else
    {
        /* Phase A remains the current maximum. */
    }

    if(phase_c > maximum)
    {
        maximum = phase_c;
    }
    else
    {
        /* The current maximum remains unchanged. */
    }

    minimum = phase_a;
    if(phase_b < minimum)
    {
        minimum = phase_b;
    }
    else
    {
        /* Phase A remains the current minimum. */
    }

    if(phase_c < minimum)
    {
        minimum = phase_c;
    }
    else
    {
        /* The current minimum remains unchanged. */
    }

    /*
     * The same common-mode value is added to every phase, so line-to-line
     * voltages are unchanged while the available DC-link range is centered.
     */
    common_mode = -(maximum + minimum) / INT32_C(2);

    result.phase_a =
        Foc_NormalizedDuty(FOC_DUTY_CENTER_I32 + phase_a + common_mode);
    result.phase_b =
        Foc_NormalizedDuty(FOC_DUTY_CENTER_I32 + phase_b + common_mode);
    result.phase_c =
        Foc_NormalizedDuty(FOC_DUTY_CENTER_I32 + phase_c + common_mode);

    /* Min-max modulation does not require the sector; it is diagnostic data. */
    result.sector = Foc_SvpwmSector(voltage);

    return result;
}
