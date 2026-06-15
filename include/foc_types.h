#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Normalized current, voltage and trigonometric values use signed Q1.15. */
typedef int16_t foc_q15_t;

/* PI gains use signed Q16.16. */
typedef int32_t foc_gain_q16_t;

/* Normalized PWM duty: 0 is 0%, UINT16_MAX is 100%. */
typedef uint16_t foc_duty_uq16_t;

typedef struct
{
    foc_q15_t alpha;
    foc_q15_t beta;
} Foc_AlphaBeta;

typedef struct
{
    foc_q15_t d;
    foc_q15_t q;
} Foc_Dq;

typedef struct
{
    foc_q15_t a;
    foc_q15_t b;
    foc_q15_t c;
} Foc_Abc;

typedef struct
{
    foc_q15_t sin;
    foc_q15_t cos;
} Foc_SinCos;

typedef struct
{
    foc_duty_uq16_t phase_a;
    foc_duty_uq16_t phase_b;
    foc_duty_uq16_t phase_c;

    /* Zero for the zero vector; otherwise one through six. */
    uint8_t sector;
} Foc_SvpwmOutput;

#endif
