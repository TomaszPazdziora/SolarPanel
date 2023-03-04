#include "servo.h"
#include "tim.h"

void setAngle(uint16_t ang, uint8_t servoIdentifier)
{
	uint16_t val;

	if(ang > ANGLE_MAX)
	{
		ang = ANGLE_MAX;
	}
	else if (ang < ANGLE_MIN)
	{
		ang = ANGLE_MIN;
	}

	val = PWM_MAX - ((ang - ANGLE_MIN) * STEP) / 1000;
	// Set choosen servo motor

	if (servoIdentifier == X_SERVO)
	__HAL_TIM_SET_COMPARE(&TIM_X_SERVO, TIM_CH_X_SERVO, val);

	else if (servoIdentifier == Y_SERVO)
	__HAL_TIM_SET_COMPARE(&TIM_Y_SERVO, TIM_CH_Y_SERVO, val);

	else if (servoIdentifier == MEAS_SERVO)
	__HAL_TIM_SET_COMPARE(&TIM_MEAS_SERVO, TIM_CH_MEAS_SERVO, val);
}
