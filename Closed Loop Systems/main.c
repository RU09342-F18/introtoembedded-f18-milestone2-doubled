#include <msp430.h> 
#include <math.h>
/**
 * main.c
 *
 * Authors: Dylan Dancel, David Russo
 * Created: 11/05/2018
 * Last Edited: 12/02/2018
 *
 * MILESTONE 2
 * -----------
 *
 * This code is part of a temperature regulation system. This code allows the user to read temperature using a thermistor
 * and to set a desired temperature. The MSP430F5529 uses this information to automate a PWM signal that drives a fan and
 * regulates the temperature.
 *
 */

/*-------------------FUNCTIONS--------------------*/

void ADC12setup();                                  // configures the ADC12 peripheral
void UARTsetup();                                   // configures UART in order to interface with Realterm
void portSetup();                                   // configures the ports to read the thermistor
void timerA0setup();                                // configures the Timer A0 for the fan PWM
unsigned char binaryToTemperature(double in);       // function that converts an ADC12 binary value to temperature
void PIDcontrol();                                  // function that controls the fan PWM

/*----------------GLOBAL VARIABLES----------------*/

unsigned int q;                                     // used in ADC12ISR to store the value of ADC12MEM0
unsigned char target = 60;                          // used in USCI_A1_interrupt and PIDcontrol to track target temperature
unsigned char temp;                                 // used in ADC12ISR and PIDcontrol to keep track of current temperature
unsigned char Q ;                                   // used in ADC12ISR to store the output of binaryToTemperature
double prevError = 0;                               // used in PIDcontrol to calculate D and I for PID control
double error;                                       // used in PIDcontrol to calculate closed loop correction
unsigned int watch;                                 // used in ADC12ISR to monitor the value of TA0CCR1 for debugging
int pwm;                                            // used in PIDcontrol to calculate the PWM of the fan
double qin;                                         // used in binaryToTemperature to convert from binary to a temperature
double I = 0;                                       // used in PIDcontol to calculate "integral" in PID control
double D = 0;                                       // used in PIDcontrol to calculate "derivative" in PID control
double P = 0;                                       // used in PIDcontrol to calculate "proportion" in PID control

/*------------------MAIN FUNCTION-----------------*/

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                       // stop watchdog timer

    ADC12setup();
    UARTsetup();
    portSetup();
    timerA0setup();
    __bis_SR_register(GIE);                         // if LPM0 used, ADC12_ISR will force exit
    while (1)
    {
        PIDcontrol();
        ADC12CTL0 |= ADC12SC;                       // start conversion, enable conversion
    }
}

/*-----------------SETUP FUNCTIONS----------------*/

// ADC12 Setup
void ADC12setup()
{
    ADC12CTL0 = ADC12ON+ADC12SHT0_15+ADC12SHT1_15;  // ADC12 Control reg 0 on, sampling hold time 64 ADC12clk cycles
    ADC12CTL1 = ADC12SHP+ADC12CONSEQ_0;             // ADC12 Sampling and hold pulse set to sample input signal
                                                    // single-channel, single conversion
    ADC12MCTL0 = ADC12INCH_0 +ADC12EOS ;            // ADC12 Mem0 control reg, input channel A0, last conversion in sequence
    ADC12CTL0 |= ADC12ENC;                          // ADC12 enable conversion
    ADC12CTL0 |= ADC12SC;                           // Start sample and conversion
    ADC12IE = 0x01;                                 // interrupt enabled
}

// Port Setup to Read Thermistor
void portSetup()
{
    P6SEL = BIT0;                                   // GPIO
    P6DIR &= ~BIT0;                                 // input direction
}

// UART Setup to interface with Realterm
void UARTsetup()
{
    P3SEL |= BIT3 + BIT4;                           // P3.3,4 = USCI_A0 TXD/RXD
    P4SEL |= BIT4 + BIT5;                           // P4.4,4.5 =USCI_A1 TXD/RXD

    // USB-to-pin UART setup
    UCA1CTL1 |= UCSWRST;                            // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                           // SMCLK
    UCA1BR0 = 104;                                  // 1MHz, 9600 Baud Rate
    UCA1BR1 = 0;                                    // 1MHz 9600 Baud Rate
    UCA1MCTL |= UCBRF_0 + UCBRS_1;                  // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                           // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                               // Enable USCI_A1 RX interrupt
}

// Timer A0 Setup to Control Fan PWM
void timerA0setup()
{
    // hardware PWM implemented
    TA0CTL = TASSEL_2 | MC_1;                       // Timer A Control set to: SMCLK, Up-mode, no internal divider
                                                    // Value stored in Timer A register initialized to zero
    TA0CCR0 = 255;                                  // TimerA0 CCR0  initialized at 255
    TA0CCTL1 = OUTMOD_7;                            // Timer A0 set to reset/set mode
    P1SEL |= BIT2;                                  // configures P1.2 as hardware PWM register
    P1DIR |= BIT2;                                  // output to PWM the fan
}

/*----------------INTERRUPT VECTORS---------------*/

// ADC12 Interrupt Service Routine
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    switch(__even_in_range(ADC12IV,34))
    {
        case  0: break;                             // Vector  0:  No interrupt
        case  2: break;                             // Vector  2:  ADC overflow
        case  4: break;                             // Vector  4:  ADC timing overflow
        case  6:                                    // Vector  6:  ADC12IFG0
            q = ADC12MEM0;                          // write register value to an unsigned int
            Q = binaryToTemperature(q);             // converts binary value to temperature in unsigned char
            if (Q >= 100)                           // band-aid code: fixing unwanted high values for
                temp = Q - 128;                     // temperature readings on Realterm
            else
                temp = Q;                           // normal conditions
            UCA1TXBUF = temp;                       // transfer the temperature to send info over UART
            watch = TA0CCR1;                        // used for debugging. Unnecessary for working code
            break;                                  // allows programmer to track the value of TA0CCR1 (PWM)
        case  8: break;                             // Vector  8:  ADC12IFG1
        case 10: break;                             // Vector 10:  ADC12IFG2
        case 12: break;                             // Vector 12:  ADC12IFG3
        case 14: break;                             // Vector 14:  ADC12IFG4
        case 16: break;                             // Vector 16:  ADC12IFG5
        case 18: break;                             // Vector 18:  ADC12IFG6
        case 20: break;                             // Vector 20:  ADC12IFG7
        case 22: break;                             // Vector 22:  ADC12IFG8
        case 24: break;                             // Vector 24:  ADC12IFG9
        case 26: break;                             // Vector 26:  ADC12IFG10
        case 28: break;                             // Vector 28:  ADC12IFG11
        case 30: break;                             // Vector 30:  ADC12IFG12
        case 32: break;                             // Vector 32:  ADC12IFG13
        case 34: break;                             // Vector 34:  ADC12IFG14
        default: break;
    }
}

// USB-to-pin UART Interrupt Service Routine
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_interrupt(void)
{
    switch (__even_in_range(UCA1IV, 4))
    {
        case 0: break;                              // Vector 0 - no interrupt
        case 2:                                     // Vector 2 - RXIFG
            while (!(UCA1IFG & UCTXIFG));           // USCI_A1 TX buffer ready?
                target = UCA1RXBUF;                 // assigns target to the value from Realterm
            break;
        case 4: break;                              // Vector 4 - TXIFG
        default: break;
    }
}

/*---------------INTERNAL FUNCTIONS---------------*/

// Binary to Temperature Conversion
unsigned char binaryToTemperature(double in)
{
    qin = (in*5)/(4096);                            // binary ADC value to voltage
    qin = (15000*qin)/(5-qin);                      // voltage to resistance

    // piecewise linear equation modeling the Steinhart-Hart equation
    // converts resistance to temperature
    if (qin > 19872)
        return round(-0.00079*qin+25.67) + 20;
    else if(qin > 12448 && qin <= 19872)
        return round(-0.00135*qin+36.77) + 20;
    else if(qin > 8059 && qin <= 12448)
        return round(-0.00228*qin+48.36) + 20;
    else if(qin > 5330 && qin <= 8059 )
        return round(-0.00366*qin+59.53) + 20;
    else if(qin > 3605 && qin <= 5330)
        return round(-.0058*qin+70.90) + 20;
    else if(qin > 2490 && qin <= 3605)
        return round(-0.00897*qin+82.33) + 20;
    else if(qin > 1753 && qin <= 2490)
        return round(-0.01357*qin+93.79) + 20;
    else if(qin > 1256 && qin <= 1753)
        return round(-0.02012*qin+105.27) + 20;
    else if(qin > 915 && qin <= 1256)
        return round(-0.02936*qin+116.88) + 20;
    else
        return round(-0.042*qin+128.45) + 20;
}

// PID Control Function
void PIDcontrol()
{
    prevError = error;                              // save the error from the previous iteration
    error = target - temp;                          // update the value of error
    I = prevError +  error;                         // integral control
    D = prevError - error;                          // derivative control
    P = error/target;                               // proportion control
    pwm = TA0CCR0*P + 5*D + 95*I;                   // PID control
    if(pwm < 0 )                                    // lower boundary condition
        TA0CCR1 = 255;
    else if(pwm > 0)                                // upper boundary condition
        TA0CCR1 = 0;
    else
      TA0CCR1  = pwm;                               // normal condition
}
