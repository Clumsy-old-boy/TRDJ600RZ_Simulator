/**
  ******************************************************************************
  * @file    KnobInput.h
  * @brief   Control knob: Bourns ENA1J-B28-L00100L optical encoder read by
  *          TIM4 in encoder mode, providing the speed and direction reference
  *          for the spindle simulator
  * @version 1.0
  * @date    2026-09-26
  * @hardware NUCLEO-G431KB (STM32G431KBTx, LQFP32) @ 170 MHz
  * @compiler ARM Compiler 6 (Keil uVision AC6)
  ******************************************************************************
  *
  * DESCRIPTION
  * ----------------------------------------------------------------------------
  *   The encoder is a 100 cycles/revolution optical model (Bourns EN series,
  *   ENA1J-B28-L00100L) powered at 5 V, with the A and B outputs in quadrature on
  *   PB6 (TIM4_CH1) and PB7 (TIM4_CH2). TIM4 runs in Encoder Mode TI1/TI2 and
  *   counts 4 increments per cycle: 400 increments per knob revolution. The knob
  *   has no detents and clockwise rotation increases the count (bench verified).
  *
  *   The module keeps a SIGNED POSITION from -16800 to +16800, updated with the
  *   displacement read at every call and multiplied by KNOB_DELTA_SCALE. The
  *   position is the speed and direction reference:
  *
  *       position  0    -> simulator stopped
  *       position > 0   -> normal direction, speed proportional to the value
  *       position < 0   -> reverse direction, speed proportional to the value
  *
  *   The TIM4 counter is never written: it only measures the displacement, so it
  *   may wrap freely with no side effect.
  *
  ******************************************************************************
  */

#ifndef KNOB_INPUT_H
#define KNOB_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Counter increments for one full knob revolution:
   100 cycles/rev * 4 increments per cycle (TI1/TI2 reading) = 400 */
#define KNOB_COUNTS_PER_REV     400U

/* Scale factor applied to the displacement read at every call.
   With 14, one knob revolution shifts the position by 5600 units; the three
   revolutions covering the whole speed range therefore equal KNOB_POS_MAX. */
#define KNOB_DELTA_SCALE        14

/* Position limits: the sign selects the rotation direction.
   Three knob revolutions cover the whole speed range:
   KNOB_DELTA_SCALE * KNOB_COUNTS_PER_REV * 3 = 14 * 400 * 3 = 16800, i.e.
   1.29 rpm per TIM4 count. */
#define KNOB_POS_MAX            16800
#define KNOB_POS_MIN            (-16800)

/**
 * @brief  Enables the TIM4 counter and clears the internal position
 * @retval None
 * @note   Call it AFTER MX_TIM4_Init() and before the main loop. In encoder
 *         mode the peripheral initialization does not start the counter:
 *         without HAL_TIM_Encoder_Start() TIM4 stays at zero. The position
 *         starts from 0, i.e. simulator stopped, regardless of where the knob
 *         physically is at power-up.
 */
void KnobInput_Init(void);

/**
 * @brief  Reads the knob displacement and updates speed and direction
 * @retval Current position, from -16800 to +16800
 * @note   Call it cyclically from the main loop: every call reads the counter,
 *         accumulates the displacement and, when needed, updates the
 *         simulator. It uses no interrupts and introduces no waiting.
 */
int32_t KnobInput_Poll(void);

/**
 * @brief  Reads the current knob position
 * @retval Position from -16800 (maximum speed in reverse) to +16800 (maximum
 *         speed in the normal direction); 0 = stopped
 */
 
int32_t KnobInput_GetPosition(void);


#ifdef __cplusplus
}
#endif

#endif /* KNOB_INPUT_H */