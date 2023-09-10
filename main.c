// Name: Fariha Rabbi
// ID: 1001749931
// Course: Embedded Systems I
// Lab 9

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for snprintf)

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "adc0.h"
#include "tm4c123gh6pm.h"

#define THUMB_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
//#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define DATA         (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define PD_CLK       (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define SPEAKER_ON   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))


// PortC masks
#define FREQ_IN_MASK 64

// PortD masks
#define DATA_LINE_MASK 4
#define CLOCK_LINE_MASK 8
#define SPEAKER_ON_MASK 2

// PortE masks
#define THUMB_LED_MASK 16
#define AIN8_MASK 32

// PortF masks
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool timeMode = false;
bool pulseActive = false;
bool speakerEnable = false;
bool calculatePulse = false;
bool pulse_alarm = false;
bool respiration_alarm = false;
uint32_t frequency = 0;
uint32_t time = 0;
uint32_t pulse_time = 0;
uint32_t breath_time = 0;
uint32_t pulse_min = 0;
uint32_t pulse_max = 0;
uint32_t respiration_min = 0;
uint32_t respiration_max = 0;
uint32_t breath_status = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void enableFingerDetection()
{
    // Configure Wide Timer 0 as the time base
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    WTIMER0_CFG_R = 4;                                // configure as 32-bit timer (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    WTIMER0_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    WTIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN2_R |= 1 << (INT_WTIMER0A-16-64);          // turn-on interrupt 110 (WTIMER0A)
}

void disableFingerDetection()
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer
    NVIC_EN2_R &= ~(1 << (INT_WTIMER0A-16-64));      // turn-off interrupt 110 (WTIMER0A)
}
void pulseTimer()
{
    // Configure Wide Timer 1 to count time between pulses
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)
}

void respirationTimer()
{
    // Configure Wide Timer 2 as the timer for respiration
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    WTIMER2_CFG_R = 4;                                // configure as 32-bit timer (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    WTIMER2_TAILR_R = 200000000;                      // set load value to 200e6 for 1 interrupt every 5 seconds
    WTIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN3_R |= 1 << (INT_WTIMER2A-16-96);          // turn-on interrupt 114 (WTIMER2A)
}


void fingerDetectionIsr()
{
    int32_t vol1;
    int32_t vol2;
    int32_t difference;

    // Measures the voltage on the collector of the phototransistor
    vol1 = readAdc0Ss3();

    // Turns on the red pulse LED for a short period of time (try 100 microseconds and see how low you can go)
    THUMB_LED = 1;
    waitMicrosecond(100);

    // Measures the voltage on the collector of the phototransistor again
    vol2 = readAdc0Ss3();

    // Turns off the red pulse LED
    THUMB_LED = 0;

    // difference between voltage 1 and voltage 2
    difference = vol2 - vol1;

    // detect if finger is present
    if (difference > 25)
    {
        THUMB_LED = 1;
        pulseActive = true;
        // If pulse_active is set, disable this timer
        disableFingerDetection();
    }

    else if (difference < 25)
    {
        THUMB_LED = 0;
        pulseActive = false;
        //enableFingerDetection();
    }

    // clear interrupt flag
    WTIMER0_ICR_R = TIMER_ICR_TATOCINT;
}

void pulseIsr()
{
    time = WTIMER1_TAV_R;                            // read counter input
    WTIMER1_TAV_R = 0;                               // zero counter for next edge

    static uint16_t raw;
    static uint16_t x[3];
    static uint8_t index = 0;
    static uint16_t sum = 0;

    raw = (60000000)/(time/40);

    //sliding average filter for bpm
    if ((raw<120) && raw>50)
    {
        sum -= x[index];
        sum += raw;
        x[index] = raw;
        index = (index + 1) % 3;
        pulse_time = sum/3;
    }

    enableFingerDetection();

    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void GPIOrespIsr()
{

    static uint32_t new;
    uint32_t i = 0;
    static  int32_t difference;
    uint32_t new_difference;
    uint32_t total_breath;
    static uint32_t inhale;
    static uint32_t exhale;
    static uint32_t prev;
    static uint32_t data;
    static uint32_t timer_val;


    timer_val = WTIMER2_TAV_R;
    char str[40];
    prev = data;

    PD_CLK = 0;

//    while (true)
//    {
        //prev = data;
        //prev_difference = new - prev;
        //while (DATA);
        data = 0;

        for (i=0;i<24;i++)
        {
            PD_CLK = 1;

            waitMicrosecond(40);
            data |= DATA;
            PD_CLK = 0;
            waitMicrosecond(40);
            data <<= 1;
        }

        new = data;
        new_difference = new - prev;
        difference = prev - new;

        if ((new_difference < 0) && breath_status == 0)
        {
            inhale++;
            //breath_status = 1;
        }

        else if ((new_difference > 0) && breath_status == 0)
        {
            exhale++;
            breath_status = 1;
        }

        if ((new_difference > 0) && breath_status == 1)
        {
            exhale++;

        }

        if (difference>2000)
        {
            breath_status = 2;
        }

        if (breath_status == 2)
        {
            breath_time = 2*((4000000*prev)/(timer_val));
            GREEN_LED ^= 1;
            WTIMER2_TAV_R = 0;
        }


//        total_breath = inhale + exhale;
//
//        if ((inhale>5) && (exhale >6))
//        {
//            breath_time = breath_time + 1;
//            GREEN_LED ^= 1;
//            inhale = 0;
//            exhale = 0;
//        }

        //breath_time = 600/total_breath;

//        if (breath > 12)
//        {
//            breath = 0;
//        }

        //PD_CLK = 1;
        //PD_CLK = 0;

//        snprintf(str, sizeof(str)," %7"PRIu32" \n", data);
//        putsUart0(str);
//    }
        GPIO_PORTD_ICR_R = DATA_LINE_MASK;              // turn-on interrupt 19 (GPIOD)
}



void respirationIsr()
{
    //breath_time = 0;
    WTIMER2_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}


// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTE_DIR_R |= THUMB_LED_MASK;
    //GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DR2R_R |= THUMB_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK;
    GPIO_PORTE_DEN_R |= THUMB_LED_MASK;
                                                         // enable LEDs and pushbuttons
    //GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    // Configure the clock line and the data line
    GPIO_PORTD_DIR_R |= CLOCK_LINE_MASK;                 // clock line as output
    GPIO_PORTD_DIR_R &= ~DATA_LINE_MASK;                 // data line as input
    GPIO_PORTD_DR2R_R |= CLOCK_LINE_MASK;                // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R |= CLOCK_LINE_MASK | DATA_LINE_MASK;
                                                         // enable data line and clock

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input

    // Configure AIN8 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN8_MASK;                 // select alternative functions for AN8 (PE5)
    GPIO_PORTE_DEN_R &= ~AIN8_MASK;                  // turn off digital operation on pin PE5
    GPIO_PORTE_AMSEL_R |= AIN8_MASK;                 // turn on analog operation on pin PE5

    // Configure SPEAKER
    GPIO_PORTD_DEN_R |= SPEAKER_ON_MASK;             // enable bit 1 for digital input
    GPIO_PORTD_AFSEL_R |= SPEAKER_ON_MASK;           // select alternative functions for PD1
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD1_M;           // map alt fns to PD1
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD1_M1PWM1;

    // Configure PWM module 1 to drive SPEAKER
    // SPEAKER on M1PWM1 (PD1), M1PWM0b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_0_CTL_R = 0;                                // turn-off PWM1 generator 0 (drives outs 0 and 1)
    PWM1_0_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;  // output 1 on PWM1, gen 0b, cmpb
    PWM1_0_LOAD_R = 2000;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
                                                     // (internal counter counts down from load value to zero)
    PWM1_0_CMPB_R = 0;                               // SPEAKER off (0=always low, 1023=always high)
    PWM1_0_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 0
    PWM1_ENABLE_R = PWM_ENABLE_PWM1EN;               // enable outputs

    // Configure rising edge interrupts on DATA inputs
    // (edge mode, single edge, rising edge, turn on interrupts)
    GPIO_PORTD_IM_R &= ~DATA_LINE_MASK;
    GPIO_PORTD_IS_R &= ~DATA_LINE_MASK;
    GPIO_PORTD_IBE_R &= ~DATA_LINE_MASK;
    GPIO_PORTD_RIS_R &= ~DATA_LINE_MASK;
    GPIO_PORTD_IEV_R |= DATA_LINE_MASK;
    GPIO_PORTD_IM_R |= DATA_LINE_MASK;
    NVIC_EN0_R = 1 << (INT_GPIOD-16-0);              // turn-on interrupt 19 (GPIOD)
}

void setSpeaker(uint16_t load, uint16_t compare)
{
    PWM1_0_LOAD_R = load;
    PWM1_0_CMPB_R = compare;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    // Initialize hardware
    initHw();
    initUart0();
    initAdc0Ss3();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Use AIN8 input with N=4 hardware sampling
    setAdc0Ss3Mux(8);
    setAdc0Ss3Log2AverageCount(2);


    USER_DATA data;
    char str[40];

    pulseTimer();
    respirationTimer();


    // enter an endless loop that executes the UART0 interface from Labs 4 and 5 only
    while (true)
    {
        getsUart0(&data);
        parseFields(&data);

        bool valid = false;

        // UART0 command “pulse” that returns the last calculated value of the pulse rate in beats/minute.
        if (isCommand(&data, "pulse", 0))
        {
            valid = true;

            // If a pulse has not been detected for some time (pulse_active is false), display “(not detected)”
            if (pulseActive == false)
            {
                putsUart0("not detected\n");
            }
            else if (pulseActive == true)
            {
                snprintf(str, sizeof(str), " %7"PRIu32" (beats/minute)\n", pulse_time);
                putsUart0(str);

                // if pulse is outside set range and alarm is off
                if (((pulse_time > pulse_max) || (pulse_time < pulse_min)) && (pulse_alarm == false))
                {
                    RED_LED = 0;
                    setSpeaker(5000,0);
                }

                // pulse outside range, alarm ON, speaker OFF
                else if (((pulse_time > pulse_max) || (pulse_time < pulse_min)) && (speakerEnable == false) && (pulse_alarm == true))
                {
                    setSpeaker(5000,0);
                    RED_LED = 1;
                    putsUart0("Pulse is outside set range!");
                }

                // pulse outside range, alarm ON, speaker ON
                else if (((pulse_time > pulse_max) || (pulse_time < pulse_min)) && (speakerEnable == true) && (pulse_alarm == true))
                {
                    RED_LED = 1;
                    putsUart0("Pulse is outside set range!");
                    setSpeaker(5000, 2500);
                    waitMicrosecond(2000000);
                    setSpeaker(5000,0);
                }

                // If the range returns to the allowed range, disable the sound and turn off the red LED
                else if ((pulse_time <= pulse_max) || (pulse_time >= pulse_min))
                {
                    RED_LED = 0;
                    setSpeaker(5000,0);
                }
            }
        }

        // Add support for a UART0 command “respiration” that returns the last calculated value of the respiration rate in breaths/minute.
        // If a breathing cycle has not been detected for 5 seconds (breath_time is 0), display “(not detected)”
        else if (isCommand(&data, "respiration", 0))
        {
            if(breath_time == 0)
            {
                putsUart0("not detected\n");
            }

            else
            {
                snprintf(str, sizeof(str), " %7"PRIu32" (breaths/minute)\n", breath_time);
                putsUart0(str);

                // if breath time is outside set range and alarm is off
                if (((breath_time > respiration_max) || (breath_time < respiration_min)) && (respiration_alarm == false))
                {
                    BLUE_LED = 0;
                    setSpeaker(5000,0);
                }

                // breathing outside range, alarm ON, speaker OFF
                else if (((breath_time > respiration_max) || (breath_time < respiration_min)) && (speakerEnable == false) && (respiration_alarm == true))
                {
                    setSpeaker(5000,0);
                    BLUE_LED = 1;
                    putsUart0("Breathing rate is outside set range!");
                }

                // breathing rate outside range, alarm ON, speaker ON
                else if (((breath_time > respiration_max) || (breath_time < respiration_min)) && (speakerEnable == true) && (respiration_alarm == true))
                {
                    BLUE_LED = 1;
                    putsUart0("Breathing rate is outside set range!");
                    setSpeaker(5000, 2500);
                    waitMicrosecond(1000000);
                    setSpeaker(5000,0);
                    waitMicrosecond(500000);
                    setSpeaker(5000, 2500);
                    waitMicrosecond(1000000);
                    setSpeaker(5000,0);
                }

                // If the range returns to the allowed range, disable the sound and turn off the red LED
                else if ((breath_time <= respiration_max) || (breath_time >= respiration_min))
                {
                    //BLUE_LED = 0;
                    setSpeaker(5000,0);
                }
            }
            valid = true;
        }

        // UART0 command “alarm pulse min max” that sets the minimum and maximum pulse limits
        else if (isCommand(&data, "alarm pulse", 3))
        {
            pulse_min = getFieldInteger(&data, 2);
            pulse_max = getFieldInteger(&data, 3);
            pulse_alarm = true;
            valid = true;
        }

        // UART0 command “alarm respiration min max” that sets the minimum and maximum respiration limits
        else if (isCommand(&data, "alarm respiration", 3))
        {
            respiration_min = getFieldInteger(&data, 2);
            respiration_max = getFieldInteger(&data, 3);
            respiration_alarm = true;
            valid = true;
        }

        // UART0 command “speaker ON|OFF” that controls the alarm feature for pulse and respiration rate
        // The LEDs would continue to show the alarm status.
        else if (isCommand(&data, "speaker", 1))
        {
            char* str = getFieldString(&data, 1);

            if (!strcmp(str, "ON"))
            {
                speakerEnable = true;
            }

            else if (!strcmp(str, "OFF"))
            {
                speakerEnable = false;
            }
            valid = true;
        }

        // Look for error
        else if (!valid)
        {
            putsUart0("Invalid command\n");
        }
    }
}

