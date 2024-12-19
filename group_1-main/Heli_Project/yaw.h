/*
 * yaw.h
 *
 *  Created on: 23/04/2024
 *      Author: jmi145, jed96
 */

#ifndef YAW_H_
#define YAW_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "buttons4.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

#define YAW_PERIPH SYSCTL_PERIPH_GPIOB
#define YAW_PORT_BASE GPIO_PORTB_BASE
#define YAW_A GPIO_PIN_0
#define YAW_B GPIO_PIN_1

#define YAW_REF_PORT_BASE GPIO_PORTC_BASE
#define YAW_REF_PIN GPIO_PIN_4
#define YAW_REF_PERIPH SYSCTL_PERIPH_GPIOC

extern volatile bool yawCalibrated;
static int8_t prev_phase;
static int8_t cur_phase;
static int32_t yaw;
static int32_t degConversion = 321;

void quadratureIntHandler(void);
void calculateYaw(int32_t prev_phase, int32_t cur_phase);
void refYawIntHandler(void);
void initYaw(void);
void disableYawRefInt(bool disabled);
int32_t calcYawDegrees(void);



#endif /* YAW_H_ */
