#ifndef SVPWM_H
#define SVPWM_H

#include "foc_types.h"

/**
 * @brief Generate normalized three-phase duties with min-max SVPWM.
 *
 * @param voltage Alpha-beta voltage normalized to the DC-link voltage.
 * @return Three UQ0.16 duties and the diagnostic sector number.
 *
 * The caller must keep the input inside the selected linear modulation limit.
 * Timer scaling, dead time and minimum-pulse handling are target-specific.
 */
Foc_SvpwmOutput Foc_Svpwm(Foc_AlphaBeta voltage);

#endif
