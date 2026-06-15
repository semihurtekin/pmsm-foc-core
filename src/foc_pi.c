#include "foc_pi.h"

#include <stddef.h>

#define FOC_GAIN_SCALE    (INT64_C(65536))

/* Convert the Q1.15 output limit to the integrator's internal scale. */
static int64_t Foc_PiIntegratorLimit(const Foc_PiConfig *config)
{
    return (int64_t)config->output_limit * FOC_GAIN_SCALE;
}

/* Clear only the dynamic part of a PI controller. */
void Foc_PiReset(Foc_PiState *state)
{
    if(state != NULL)
    {
        state->integrator_q31 = 0;
    }
    else
    {
        /* A null state requires no reset action. */
    }
}

/*
 * One iteration of the parallel PI form. The configured integral gain already
 * contains the sample time, so the caller does not pass Ts on every step.
 */
Foc_PiResult Foc_PiStep(Foc_PiState *state, const Foc_PiConfig *config,
                        foc_q15_t reference, foc_q15_t feedback,
                        foc_q15_t feed_forward)
{
    Foc_PiResult result;

    result.output = 0;
    result.saturated = false;

    if((state != NULL) && (config != NULL))
    {
        if(config->output_limit > 0)
        {
            int32_t error = (int32_t)reference - (int32_t)feedback;
            int64_t proportional =
                ((int64_t)error * (int64_t)config->kp) / FOC_GAIN_SCALE;

            /*
             * The integral state keeps the gain's fractional bits. This
             * prevents small increments from disappearing on each iteration.
             */
            int64_t integral_increment =
                (int64_t)error * (int64_t)config->ki;
            int64_t candidate_integrator =
                state->integrator_q31 + integral_increment;
            int64_t integrator_limit = Foc_PiIntegratorLimit(config);
            int64_t candidate_output;
            bool reject_integrator;

            if(candidate_integrator > integrator_limit)
            {
                candidate_integrator = integrator_limit;
            }
            else if(candidate_integrator < -integrator_limit)
            {
                candidate_integrator = -integrator_limit;
            }
            else
            {
                /* No limiting is needed. */
            }

            candidate_output =
                proportional +
                (candidate_integrator / FOC_GAIN_SCALE) +
                (int64_t)feed_forward;

            /*
             * Conditional integration rejects only an update that would push
             * a saturated output farther in the same direction.
             */
            reject_integrator =
                ((candidate_output > config->output_limit) && (error > 0)) ||
                ((candidate_output < -config->output_limit) && (error < 0));

            if(!reject_integrator)
            {
                state->integrator_q31 = candidate_integrator;
            }
            else
            {
                /* Keep the previous state while saturation is worsening. */
            }

            candidate_output =
                proportional +
                (state->integrator_q31 / FOC_GAIN_SCALE) +
                (int64_t)feed_forward;

            if(candidate_output > config->output_limit)
            {
                result.output = config->output_limit;
                result.saturated = true;
            }
            else if(candidate_output < -config->output_limit)
            {
                result.output = (foc_q15_t)(-config->output_limit);
                result.saturated = true;
            }
            else
            {
                result.output = (foc_q15_t)candidate_output;
            }
        }
        else
        {
            /* The initialized zero result represents an invalid limit. */
        }
    }
    else
    {
        /* The initialized zero result represents invalid arguments. */
    }

    return result;
}
