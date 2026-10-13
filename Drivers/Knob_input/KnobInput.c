/**
  ******************************************************************************
  * @file    KnobInput.c
  * @brief   Control knob implementation
  * @version 1.0
  * @date    2026-09-26
  * @hardware NUCLEO-G431KB (STM32G431KBTx, LQFP32) @ 170 MHz
  * @compiler ARM Compiler 6 (Keil uVision AC6)
  ******************************************************************************
  *
  * DESIGN NOTES
  * ----------------------------------------------------------------------------
  *   - The TIM4 counter wraps at 65535: the difference between two readings must
  *     therefore be interpreted as a signed integer, otherwise a zero crossing
  *     would become a +65535 jump.
  *   - The counter is never written: the position lives in software only, so
  *     saturation at the limits creates no dead zone and the movement resumes
  *     immediately when turning back.
  *   - The speed is applied at most every KNOB_APPLY_MS: every
  *     SpindleSim_SetRpm() resets the TIM3 counter, so a continuous update would
  *     introduce phase discontinuities in the generated signal.
  *
  ******************************************************************************
  */

#include "KnobInput.h"
#include "SpindleSim.h"

/* TIM4 handle, defined in main.c and initialized by MX_TIM4_Init() */
extern TIM_HandleTypeDef htim4;

/* Minimum interval between two simulator updates [ms] */
#define KNOB_APPLY_MS           50U

/* Last count read from the counter */
static uint16_t knob_last = 0U;

/* Knob position, from KNOB_POS_MIN to KNOB_POS_MAX */
static int32_t knob_pos = 0;

/* Last position applied to the simulator and instant of application */
static int32_t  knob_applied = 0;
static uint32_t knob_tick    = 0U;

/**
  * @brief  Converts the knob position into speed and direction
  * @param  pos: position from KNOB_POS_MIN to KNOB_POS_MAX
  * @retval None
  * @note   At position 0 the simulator is stopped; for non-zero positions the
  *         direction comes from the sign and the speed grows linearly with the
  *         absolute value between SPINDLE_SIM_RPM_MIN and SPINDLE_SIM_RPM_MAX.
  */
static void KnobInput_ApplyPosition(int32_t pos)
{
    uint32_t modulo;
    float    rpm;

    if (pos == 0) {
        /* Centre position: no simulated rotation */
        if (SpindleSim_IsRunning()) {
            SpindleSim_Stop();
        }
        return;
    }

    /* The sign of the position selects the rotation direction */
    (void)SpindleSim_SetDirection(pos < 0);

    /* Speed proportional to the absolute value of the position */
    modulo = (uint32_t)((pos < 0) ? -pos : pos);
    rpm = SPINDLE_SIM_RPM_MIN
        + ((float)modulo / (float)KNOB_POS_MAX)
        * (SPINDLE_SIM_RPM_MAX - SPINDLE_SIM_RPM_MIN);
    (void)SpindleSim_SetRpm(rpm);

    /* The outputs are enabled only after setting geometry and speed: the start
       therefore generates no spurious edge */
    if (!SpindleSim_IsRunning()) {
        SpindleSim_Start();
    }
}

void KnobInput_Init(void)
{
    /* In encoder mode the peripheral is configured but the counter is stopped:
       it must be enabled explicitly */
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* Initial position 0: at power-up the knob commands from standstill,
       whatever its physical position */
    knob_last    = (uint16_t)TIM4->CNT;
    knob_pos     = 0;
    knob_applied = 0;
    knob_tick    = HAL_GetTick();
}

int32_t KnobInput_Poll(void)
{
    uint16_t now = (uint16_t)TIM4->CNT;
    int16_t  delta;
    int32_t  pos;

    /* Signed difference: the counter wraps at 65535 and an unsigned reading
       would become a +65535 jump */
    delta = (int16_t)(now - knob_last);
    knob_last = now;

    if (delta != 0) {
        /* The displacement is amplified to reduce the knob turns needed to
           cover the whole range */
        pos = knob_pos + ((int32_t)delta * KNOB_DELTA_SCALE);

        /* Saturation at the limits: beyond full scale the position does not grow,
           but the counter keeps being read, so the movement resumes immediately
           when turning back */
        if (pos > KNOB_POS_MAX) { pos = KNOB_POS_MAX; }
        if (pos < KNOB_POS_MIN) { pos = KNOB_POS_MIN; }
        knob_pos = pos;

        /* Speed and direction are updated only if the position changed and the
           minimum interval has elapsed; the last movement is applied anyway,
           because the position stays different from the applied one */
        if ((knob_pos != knob_applied)
            && ((HAL_GetTick() - knob_tick) >= KNOB_APPLY_MS)) {
            knob_tick    = HAL_GetTick();
            knob_applied = knob_pos;
            KnobInput_ApplyPosition(knob_pos);
        }
    }

    return knob_pos;
}

int32_t KnobInput_GetPosition(void)
{
    return knob_pos;
}