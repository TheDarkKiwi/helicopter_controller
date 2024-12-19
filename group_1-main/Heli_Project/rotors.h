/*
 * rotors.h
 *
 *  Created on: 7/05/2024
 *      Author: jmi145, jed96
 */

#ifndef ROTORS_H_
#define ROTORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons4.h"
#include "utils/ustdlib.h"
#include "stdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"

// Frequency
#define PWM_RATE_HZ 50

// Duty Cycle
#define PWM_START_DUTY_PER  0
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        4

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  ---Tail Rotor PWM: PF1
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

// duty cycle limits
#define PWM_MIN 2
#define PWM_MAX 98

// main rotor gains
#define KP_MAIN 100
#define KI_MAIN 5
#define KD_MAIN 20

// tail rotor gains
#define KP_TAIL 100
#define KI_TAIL 5
#define KD_TAIL 20

// button goal step sizes
#define YAW_STEP 15
#define ALT_STEP 10

// Alt boundaries for Flying state
#define ALT_UPPER_BOUND 90
#define ALT_LOWER_BOUND 10

// circle constants and PID scale factor
#define PID_SCALE_FACTOR 10
#define FULL_ROTATION 360
#define HALF_ROTATION 180

// global variables for PID controllers
static int32_t mainError = 0;
static int32_t prevMainError = 0;
static int32_t mainControl = 0;
static int32_t mainI = 0;

static int32_t tailError = 0;
static int32_t prevTailError = 0;
static int32_t tailControl = 0;
static int32_t tailI = 0;

// functions
void initialisePWMMain (void);
void initialisePWMTail (void);
void setPWMMain (uint32_t ui32Duty);
void setPWMTail (uint32_t ui32Duty);
int32_t controlTailGoal (int32_t targetYawDeg);
int32_t controlMainGoal (int32_t targetAlt);
int32_t pidMainRotor (int32_t targetAlt, int32_t percentAlt, uint8_t deltaTime);
int32_t pidTailRotor (int32_t targetYawDeg, int32_t degrees, uint8_t deltaTime);
void turnRotorsOff(void);
void turnRotorsOn(void);

#endif /* ROTORS_H_ */
