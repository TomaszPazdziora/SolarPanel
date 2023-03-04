/* The MIT License
 *
 * Copyright (c) 2022 Tomasz Paździora
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include <stdint.h>

/*=====================================*/

#define X_SERVO 1
#define TIM_X_SERVO htim10
#define TIM_CH_X_SERVO TIM_CHANNEL_1

/*=====================================*/

#define Y_SERVO 2
#define TIM_Y_SERVO htim4
#define TIM_CH_Y_SERVO TIM_CHANNEL_2

/*=====================================*/

#define MEAS_SERVO 3
#define TIM_MEAS_SERVO htim1
#define TIM_CH_MEAS_SERVO TIM_CHANNEL_3

/*=====================================*/

/* zakresy katowe pracy serwomechanizmu */
#define ANGLE_MIN 0
#define ANGLE_MAX 1800
/* zakres PWM */
#define PWM_MIN 500
#define PWM_MAX 2500

#define STEP ((1000 * (PWM_MAX - PWM_MIN)) / (ANGLE_MAX - ANGLE_MIN))

void setAngle(uint16_t ang, uint8_t servoIdentifier);

#endif /* INC_SERVO_H_ */
