#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "foc_math.h"
#include "foc_pi.h"
#include "svpwm.h"

typedef struct
{
    Foc_PiConfig id_pi;
    Foc_PiConfig iq_pi;

    /*
     * Shared d-q voltage magnitude limit. FOC_Q15_INV_SQRT3 is suitable for
     * linear SVPWM when voltage is normalized to the DC link.
     */
    foc_q15_t voltage_limit;
} Foc_Config;

typedef struct
{
    Foc_Config config;
    Foc_PiState id_pi;
    Foc_PiState iq_pi;
} Foc_Controller;

typedef struct
{
    Foc_AlphaBeta current;
    Foc_SinCos electrical_angle;
    Foc_Dq current_reference;
    Foc_Dq voltage_feed_forward;
} Foc_Input;

typedef struct
{
    Foc_Dq measured_current;
    Foc_Dq voltage_dq;
    Foc_AlphaBeta voltage_alpha_beta;
    Foc_SvpwmOutput modulation;
    bool voltage_saturated;
} Foc_Output;

/**
 * @brief Initialize one FOC current-loop instance.
 *
 * @param controller Control instance owned by the application.
 * @param config PI settings and shared voltage limit.
 * @return `true` when the pointers and configuration are valid.
 */
bool Foc_Init(Foc_Controller *controller, const Foc_Config *config);

/**
 * @brief Reset both PI integrators without changing configuration.
 *
 * @param controller Control instance to reset. A null pointer is ignored.
 */
void Foc_Reset(Foc_Controller *controller);

/**
 * @brief Execute one FOC inner current-control iteration.
 *
 * @param controller Initialized control instance.
 * @param input Synchronized current, angle and reference inputs.
 * @param output Calculated d-q values, alpha-beta voltage and PWM duties.
 * @return `true` when all pointers are valid.
 *
 * Processing order: Park transform, d/q PI control, circular voltage limit,
 * inverse Park transform and SVPWM.
 */
bool Foc_Step(Foc_Controller *controller, const Foc_Input *input,
              Foc_Output *output);

#endif
