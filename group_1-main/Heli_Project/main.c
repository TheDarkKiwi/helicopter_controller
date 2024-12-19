//********************************************************
//
// main.c - main project code for ENCE361 helicopter project
//
// Link with modules:  buttons2, OrbitOLEDInterface, circBufT
//
// Author:  P.J. Bones  UCECE
// Last modified:   16.4.2018
//

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
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "circBufT.h"
#include "yaw.h"
#include "rotors.h"
#include "uart.h"


//********************************************************
// Constants
//********************************************************
#define SYSTICK_RATE_HZ 100
#define SLOWTICK_RATE_HZ 2
#define MAX_STR_LEN 17
#define UART_STR_LENGTH 110
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX
#define BUF_SIZE 12
#define SAMPLE_RATE_HZ 48


//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
static int32_t heliLandedAlt = 1240; //Value for the reading at heli landed alitutude
static int32_t mainControl;
static int32_t tailControl;

static bool slowTick = false;
static int32_t percentAlt = 0;
static int32_t degrees = 0;
static int32_t controlDeg = 0;
static int32_t targetAlt = 0;
static int32_t targetYawDeg = 0;


typedef enum {
    LANDED = 0,
    TAKEOFF,
    FLYING,
    LANDING
} States;

static States currentState = LANDED;
static int32_t deltaTime = 10;


// The interrupt handler for the for SysTick interrupt.
void SysTickIntHandler(void) {
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / SLOWTICK_RATE_HZ;
    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }
}

void ADCIntHandler(void) {
    // Get the single sample from ADC0, place it in a circular buffer and clear the interupt.
    uint32_t ulValue;
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    writeCircBuf (&g_inBuffer, ulValue);
    ADCIntClear(ADC0_BASE, 3);
}

void initADC (void) {
    //Initialize the ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
    ADCIntEnable(ADC0_BASE, 3);
}

void initClock (void) {
    // Set the clock rate to 20 MHz, set up the period for the timer and enable the service and interupt
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
    SysTickEnable();
}

void initDisplay (void) {
    OLEDInitialise ();
}

void display(int32_t percentVal, int32_t degrees, int32_t altTarget, int32_t targetYawDeg, int32_t MainDuty, int32_t TailDuty, States currentState) {
    char string[MAX_STR_LEN];
    char uartStr[UART_STR_LENGTH];
    char stateString[MAX_STR_LEN];

    //Extract integer and decimal parts from the degrees calculation, and then display the altitude percentage and the yaw value.
    int32_t integerPart = degrees / 100;
    int32_t decimalPart = abs(degrees % 100);

    // Display the altitude percentage
    usnprintf (string, sizeof(string), "Altitude = %3d%%", percentVal);
    OLEDStringDraw (string, 0, 0);

    /* Displays the yaw value in degrees, and checks to see if the decimal part 
    is less than 10 (0.1 in actual terms) and if it is, adds a 0 in front of it, so it is displayed correctly. */
    if (decimalPart < 10) {
        usnprintf (string, sizeof(string), "Yaw = %3d.0%1d deg", integerPart, decimalPart);
        OLEDStringDraw (string, 0, 1);
    } else {
        usnprintf (string, sizeof(string), "Yaw = %3d.%2d deg", integerPart, decimalPart);
        OLEDStringDraw (string, 0, 1);
    }

    //Display current state of heli
    switch (currentState) {
        case LANDED:
            usnprintf(stateString, sizeof(stateString), "LANDED ");
            OLEDStringDraw (stateString, 0, 2);
            break;
        case TAKEOFF:
            usnprintf(stateString, sizeof(stateString), "TAKEOFF");
            break;
        case FLYING:
            usnprintf(stateString, sizeof(stateString), "FLYING ");
            break;
        case LANDING:
            usnprintf(stateString, sizeof(stateString), "LANDING");
            break;
    }
    usnprintf (string, sizeof(string), "State: %s", stateString);
    OLEDStringDraw (string, 0, 2);

    // creates the string for the UART message
    usnprintf (uartStr, sizeof(uartStr), "Alt: R = %3d%%, T = %3d%%, "
            "Deg: R = %3d.%2d deg, T = %3d deg, "
            "PWM: M = %3d%%, T = %3d%%, "
            "State: %s \r\n"
               , percentVal, altTarget, integerPart, decimalPart, targetYawDeg, MainDuty, TailDuty, stateString);
    // sends UART message
    UARTSend(uartStr);
}

int32_t calcAvgAlt(void) {
    // Gets avg PWM value
    int16_t i;
    int32_t sum;
    sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf (&g_inBuffer);
    // Calculate and display the rounded mean of the buffer contents
    return (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}

int calculateAltitudePercentage(int32_t adcValue, int32_t landedAltitude) {
    // Calculates the percentage of altitude the heli is currently at.
    int32_t altitudePercentage = (((heliLandedAlt - adcValue) * 10) / 124);
    return altitudePercentage;
}


void updateLandedState(void) {
    // Control the heli when it is in LANDED state
    turnRotorsOff();
    if (checkButton(SWITCH1) == PUSHED) {
        currentState = TAKEOFF;
        yawCalibrated = false;
    }
}

void updateTakeoffState(void) {
    // Control the heli when it is in TAKEOFF state
    turnRotorsOn();

    targetAlt = 10;
    targetAlt = controlMainGoal(targetAlt);

    if ((percentAlt >= 8) && yawCalibrated == true) {
        //Once the yaw has been calibrated (i.e once the heli is the right spot) it can move to FLYING state
        targetYawDeg = 0;
        currentState = FLYING;
    }

    if (yawCalibrated == false) {
        //Moves the heli to the reference point. This involves making sure it is at the correct height and orientation.
        targetYawDeg = controlDeg + 10;
        mainControl = pidMainRotor(targetAlt, percentAlt, deltaTime);
        tailControl = pidTailRotor(targetYawDeg, controlDeg, deltaTime);
        setPWMMain(mainControl);
        setPWMTail(tailControl);
    }
}

void updateFlyingState(void) {
    // Control the heli when it is in FLYING state
    // Sets target altitude and yaw based on input from the buttons
    targetYawDeg = controlTailGoal(targetYawDeg);
    targetAlt = controlMainGoal(targetAlt);

    // checks if going in for landing from down button
    if (targetAlt < 10) {
        targetAlt = 10;
        currentState = LANDING;
    }

    // Controls PWM signals for the rotors while in flight
    mainControl = pidMainRotor(targetAlt, percentAlt, deltaTime);
    tailControl = pidTailRotor(targetYawDeg, controlDeg, deltaTime);
    setPWMMain(mainControl);
    setPWMTail(tailControl);

    // checks if going in for landing from switch1 release
    if (checkButton(SWITCH1) == RELEASED) {
        currentState = LANDING;
        targetAlt = 10;
    }
}

void updateLandingState(void) {
    // Control the heli when in LANDING state
    targetYawDeg = -3;
    mainControl = pidMainRotor(targetAlt, percentAlt, deltaTime);
    tailControl = pidTailRotor(targetYawDeg, controlDeg, deltaTime);
    setPWMMain(mainControl);
    setPWMTail(tailControl);

    if ((percentAlt <= 1) && (controlDeg <= 2 && controlDeg >= -2)) {
        //Once at correct height and orientation, lower the heli and land
        currentState = LANDED;
    } else if ((controlDeg <= -1 && controlDeg >= -3) && (percentAlt <= 12 && percentAlt >= 8)) {
        targetAlt = 0;
    }
}

void initMain(void) {
    //Initialize everything
    initClock ();
    initADC ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initialisePWMMain ();
    initialisePWMTail();
    turnRotorsOn();
    initialiseUSB_UART();
    initYaw();
    initDisplay ();
    initButtons ();
    initResetInt ();
    IntMasterEnable();
    SysCtlDelay (SysCtlClockGet() / 6);
}

int main(void) {

    //Clears the buttons/switches
    SysCtlPeripheralReset (UP_BUT_PERIPH);
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);
    SysCtlPeripheralReset (SWITCH_BUT_PERIPH);

    initMain();
    // Get altitude reading to determine landed altitude
    heliLandedAlt = calcAvgAlt();
    display(percentAlt, 0, 0, 0, 0, 0, LANDED);

    while (1)

    {

        updateButtons ();
        percentAlt = calcAvgAlt();
        percentAlt = calculateAltitudePercentage(percentAlt, heliLandedAlt);
        //get yaw in degrees
        degrees = calcYawDegrees();
        // degrees is a factor of 100 off, for sub-degree precision, and as such, must be normalized before being used by control funcitons
        controlDeg = degrees / 100;


        //Checks what state the heli is in and calls the appropriate control function
        if (currentState == LANDED) {
            updateLandedState();

        } else if (currentState == TAKEOFF) {
            updateTakeoffState();

        } else if (currentState == FLYING) {
            updateFlyingState();

        } else if (currentState == LANDING) {
            updateLandingState();
        }

        if (slowTick) {
            slowTick = false;
            // Form and send a status message to the console as well as display relevant readings on the OLED display
            display(percentAlt, degrees, targetAlt, targetYawDeg, mainControl, tailControl, currentState);
        }
    }



}



