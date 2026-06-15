#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_types.h"

#define FOC_Q15_ONE             ((foc_q15_t)32767)
#define FOC_Q15_HALF            ((foc_q15_t)16384)
#define FOC_Q15_INV_SQRT3       ((foc_q15_t)18919)
#define FOC_Q15_SQRT3_BY_2      ((foc_q15_t)28378)

/**
 * @brief Saturate a signed intermediate to the Q1.15 storage range.
 *
 * @param value Value to limit.
 * @return Value limited to [-32768, 32767].
 */
foc_q15_t Foc_Q15Saturate(int32_t value);

/**
 * @brief Multiply two signed Q1.15 values.
 *
 * @param a First operand.
 * @param b Second operand.
 * @return Saturated Q1.15 product.
 */
foc_q15_t Foc_Q15Mul(foc_q15_t a, foc_q15_t b);

/**
 * @brief Apply the amplitude-invariant Clarke transform.
 *
 * @param current Three-phase current in Q1.15.
 * @return Stationary alpha-beta current in Q1.15.
 *
 * Uses alpha = a and beta = (a + 2*b) / sqrt(3). The input is expected to
 * represent a balanced three-phase system.
 */
Foc_AlphaBeta Foc_Clarke(Foc_Abc current);

/**
 * @brief Rotate an alpha-beta vector into the electrical rotor frame.
 *
 * @param stationary Alpha-beta vector in Q1.15.
 * @param angle Electrical-angle sine and cosine in Q1.15.
 * @return Rotor-frame d-q vector in Q1.15.
 */
Foc_Dq Foc_Park(Foc_AlphaBeta stationary, Foc_SinCos angle);

/**
 * @brief Rotate a d-q vector back into the stationary frame.
 *
 * @param rotating Rotor-frame d-q vector in Q1.15.
 * @param angle Electrical-angle sine and cosine in Q1.15.
 * @return Stationary alpha-beta vector in Q1.15.
 */
Foc_AlphaBeta Foc_InversePark(Foc_Dq rotating, Foc_SinCos angle);

/**
 * @brief Limit a d-q vector without changing its direction.
 *
 * @param input Requested vector in Q1.15.
 * @param magnitude_limit Maximum vector magnitude in Q1.15.
 * @param output Original or scaled vector.
 * @return `true` when scaling was applied.
 */
bool Foc_LimitVector(Foc_Dq input, foc_q15_t magnitude_limit, Foc_Dq *output);

#endif
