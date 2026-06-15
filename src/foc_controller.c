#include "foc_controller.h"

#include <stddef.h>

/* Initialize one current-loop instance and clear its dynamic state. */
bool Foc_Init(Foc_Controller *controller, const Foc_Config *config)
{
    bool valid = false;

    /*
     * Validate pointers before reading configuration fields. Keeping pointer
     * checks separate also makes the intended evaluation order explicit.
     */
    if((controller != NULL) && (config != NULL))
    {
        if((config->voltage_limit > 0) &&
            (config->id_pi.output_limit > 0) &&
            (config->iq_pi.output_limit > 0) &&
            (config->id_pi.kp >= 0) &&
            (config->id_pi.ki >= 0) &&
            (config->iq_pi.kp >= 0) &&
            (config->iq_pi.ki >= 0))
        {
            valid = true;
        }
        else
        {
            /* Configuration is not valid. */
        }
    }
    else
    {
        /* A null argument cannot be initialized. */
    }

    if(valid)
    {
        controller->config = *config;
        Foc_PiReset(&controller->id_pi);
        Foc_PiReset(&controller->iq_pi);
    }
    else
    {
        /* No state is modified for an invalid configuration. */
    }

    return valid;
}

/* Reset the PI memories while preserving the configured gains and limits. */
void Foc_Reset(Foc_Controller *controller)
{
    if(controller != NULL)
    {
        Foc_PiReset(&controller->id_pi);
        Foc_PiReset(&controller->iq_pi);
    }
    else
    {
        /* A null control instance requires no reset action. */
    }
}

/*
 * Execute the mathematical inner current-control path. Current sampling,
 * angle generation and timer updates are intentionally outside this function.
 */
bool Foc_Step(Foc_Controller *controller, const Foc_Input *input,
              Foc_Output *output)
{
    bool valid = false;

    if((controller != NULL) &&
        (input != NULL) &&
        (output != NULL))
    {
        valid = true;
    }
    else
    {
        /* Invalid arguments leave the controller and output unchanged. */
    }

    if(valid)
    {
        Foc_PiResult vd_result;
        Foc_PiResult vq_result;
        Foc_Dq requested_voltage;
        int64_t previous_id_integrator = controller->id_pi.integrator_q31;
        int64_t previous_iq_integrator = controller->iq_pi.integrator_q31;

        output->measured_current =
            Foc_Park(input->current, input->electrical_angle);

        vd_result =
            Foc_PiStep(&controller->id_pi, &controller->config.id_pi,
                       input->current_reference.d,
                       output->measured_current.d,
                       input->voltage_feed_forward.d);

        vq_result =
            Foc_PiStep(&controller->iq_pi, &controller->config.iq_pi,
                       input->current_reference.q,
                       output->measured_current.q,
                       input->voltage_feed_forward.q);

        requested_voltage.d = vd_result.output;
        requested_voltage.q = vq_result.output;

        /*
         * The inverter imposes one magnitude limit on the combined d-q
         * request. Scaling both components preserves the requested angle.
         */
        output->voltage_saturated =
            Foc_LimitVector(requested_voltage,
                            controller->config.voltage_limit,
                            &output->voltage_dq);

        /*
         * This small implementation handles circular-limit windup by
         * discarding both integral updates from the saturated iteration.
         */
        if(output->voltage_saturated)
        {
            controller->id_pi.integrator_q31 = previous_id_integrator;
            controller->iq_pi.integrator_q31 = previous_iq_integrator;
        }
        else
        {
            /* The accepted integral updates are retained. */
        }

        output->voltage_alpha_beta =
            Foc_InversePark(output->voltage_dq, input->electrical_angle);
        output->modulation = Foc_Svpwm(output->voltage_alpha_beta);

        output->voltage_saturated =
            output->voltage_saturated ||
            vd_result.saturated ||
            vq_result.saturated;
    }
    else
    {
        /* Invalid arguments were handled before entering the control path. */
    }

    return valid;
}
