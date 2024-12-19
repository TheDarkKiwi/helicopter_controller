/*
 * rotors.c
 *
 *  Created on: 7/05/2024
 *      Author: jmi145, jed96
 */

#include "rotors.h"

void
setPWMMain (uint32_t ui32DutyMain)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32DutyMain / 100);
}

void setPWMTail (uint32_t ui32DutyTail) {
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * ui32DutyTail / 100);
}

void turnRotorsOff (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

void turnRotorsOn (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

void initialisePWMMain (void) {
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);
    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    setPWMMain(PWM_START_DUTY_PER);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
}

void
initialisePWMTail (void) //Initialize and make tail rotor pwm
{
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_GPIO);
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);
    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    setPWMTail (PWM_START_DUTY_PER);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

}

int32_t controlTailGoal (int32_t targetYawDeg) {
    //Background task: Up and down buttons alter duty cycle by 5% respectively
    if (checkButton (RIGHT) == PUSHED)
    {
        targetYawDeg += YAW_STEP;
        if (targetYawDeg > HALF_ROTATION) {
            targetYawDeg -= FULL_ROTATION;
        }
    }
    if (checkButton (LEFT) == PUSHED)
    {
        targetYawDeg -= YAW_STEP;
        if (targetYawDeg <= (-1 * HALF_ROTATION)) {
            targetYawDeg += FULL_ROTATION;
        }
    }
    return targetYawDeg;
}

int32_t controlMainGoal (int32_t targetAlt) {
    // Returns an array containing targetAlt, whether or not a statechange is required, and what state it should be in depending on the input.
    if ((checkButton(UP) == PUSHED) && (targetAlt <= ALT_UPPER_BOUND)) {
        targetAlt += ALT_STEP;
    } else if ((checkButton(DOWN) == PUSHED) && (targetAlt >= ALT_LOWER_BOUND)) {
        targetAlt -= ALT_STEP;
    }
    return targetAlt;
}

int32_t pidMainRotor(int32_t targetAlt, int32_t percentAlt, uint8_t deltaTime) {
    mainI = 0;
    // finds the error in the main rotor position
    mainError = targetAlt - percentAlt;

    // takes main error and turn into control output
    int32_t mainP = KP_MAIN * mainError;
    int32_t mainDi = (KI_MAIN * mainError) * deltaTime;
    int32_t mainD = KD_MAIN * (mainError - prevMainError) / deltaTime;
    mainControl  = (mainP + mainI + mainDi + mainD);
    prevMainError = mainError;

    // main rotor duty cycle limits
    if (mainControl > PWM_MAX) {
        mainControl = PWM_MAX;
    } else if (mainControl < PWM_MIN) {
        mainControl = PWM_MIN;
    } else {
        mainI += mainDi;
    }
    return mainControl * PID_SCALE_FACTOR; // return the control times a scale factor
}

int32_t pidTailRotor(int32_t targetYawDeg, int32_t degrees, uint8_t deltaTime) {
    tailI = 0;
    // finds the error in the tail rotor rotation
    tailError = targetYawDeg - degrees;

    // checks for if the tail rotor error is above half circle limit
    if (tailError > HALF_ROTATION) {
        tailError -= FULL_ROTATION;
    } else if (tailError <= (-1 * HALF_ROTATION)) {
        tailError += FULL_ROTATION;
    }

    // takes tail error and turn into control output
    int32_t tailP = KP_TAIL * tailError;
    int32_t tailDi = (KI_TAIL * tailError) * deltaTime;
    int32_t tailD = KD_TAIL * (tailError - prevTailError) / deltaTime;
    tailControl  = (tailP + tailI + tailDi + tailD);
    tailI += tailDi;
    prevTailError = tailError;

    // tail rotor duty cycle limits
    if (tailControl > PWM_MAX) {
        tailControl = PWM_MAX;
    } else if (tailControl < PWM_MIN) {
        tailControl = PWM_MIN;
    } else {
        tailI += tailDi;
    }
    return tailControl * PID_SCALE_FACTOR; // return the control times a scale factor
}




