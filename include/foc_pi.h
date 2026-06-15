#ifndef FOC_PI_H
#define FOC_PI_H

#include "foc_types.h"

typedef struct
{
    foc_gain_q16_t kp;       /* Proportional gain in Q16.16. */
    foc_gain_q16_t ki;       /* Discrete Ki*Ts gain in Q16.16. */
    foc_q15_t output_limit;  /* Symmetric output limit in Q1.15. */
} Foc_PiConfig;

typedef struct
{
    /*
     * The integral state retains the fractional bits introduced by the
     * Q16.16 gain to reduce loss of small increments.
     */
    int64_t integrator_q31;
} Foc_PiState;

typedef struct
{
    foc_q15_t output;
    bool saturated;
} Foc_PiResult;

/**
 * @brief Clear the stored integral contribution.
 *
 * @param state PI state to reset. A null pointer is ignored.
 */
void Foc_PiReset(Foc_PiState *state);

/**
 * @brief Execute one discrete PI-controller iteration.
 *
 * @param state Persistent integral state.
 * @param config Gains and symmetric output limit.
 * @param reference Requested current in Q1.15.
 * @param feedback Measured current in Q1.15.
 * @param feed_forward Optional voltage contribution in Q1.15.
 * @return Limited voltage request and saturation status.
 *
 * The controller uses the parallel form. `config->ki` must already include
 * the loop period, and conditional integration is used during saturation.
 */
Foc_PiResult Foc_PiStep(Foc_PiState *state, const Foc_PiConfig *config,
                        foc_q15_t reference, foc_q15_t feedback,
                        foc_q15_t feed_forward);

#endif
