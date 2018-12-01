#include <msp430.h> 
#include <math.h>
/**
 * main.c
 *
 * Thermistor Configuration
 */

void ADC12setup();
void UARTsetup();
void portSetup(); // for the thermistor

void timerA0setup();
unsigned char binaryToTemperature(double in);
//int in = 5;
unsigned int q;
unsigned char target = 60;
unsigned char temp;
unsigned char Q ;
double prevError = 0;
double error;
unsigned int watch;
int RANGE = 256;
int pwm;
double qin;
double I = 0;
double D = 0;
double P = 0;

/*------------------MAIN FUNCTION-----------------*/

int main(void)
{

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    //target = 50; // for now
   // error = 0; // initialize error
    ADC12setup();
    UARTsetup();
    portSetup();
    timerA0setup();



    __bis_SR_register(GIE);      // LPM0, ADC12_ISR will force exit
    while (1)
    {
              prevError = error;
              error = target - temp;
              I = prevError +  error;
              D = prevError - error;
              P = error/target;
              pwm = TA0CCR0*P+D*5+95*I;
              if(pwm < 0 )
                  TA0CCR1 = 255;
              else if(pwm > 0)
                  TA0CCR1 = 0;
              else
                  TA0CCR1  = pwm ;


       // ADC12CTL0 |= ADC12ENC;                  // Start sampling/conversion
        ADC12CTL0 |= ADC12SC;         //Start conversion, enable conversion


       //
        //__no_operation();
    }

}

/*-----------------ADC12 Setup----------------------*/


void ADC12setup()
{
    ADC12CTL0 = ADC12ON  + ADC12SHT0_15+ ADC12SHT1_15; //ADC12 Control Register 0 on, sampling hold time 64 ADC12clk cycles
    ADC12CTL1 = ADC12SHP+ADC12CONSEQ_0; //ADC12 Sampling and hold pulse set to sample input signal, single-channel, single conversion
    ADC12MCTL0 = ADC12INCH_0 +ADC12EOS ; //ADC12 Memory 0 control register, input channel A0,Last conversion in sequence.
    ADC12CTL0 |= ADC12ENC;              //ADC12 enable conversion
    ADC12CTL0 |= ADC12SC;
    ADC12IE = 0x01;
}
void portSetup()
{
    // code
    P6SEL = BIT0; // GPIO
    P6DIR &= ~BIT0; // input direction

}


void UARTsetup()
{
    P3SEL |= BIT3 + BIT4;                               // P3.3,4 = USCI_A0 TXD/RXD
    P4SEL |= BIT4 + BIT5;                               // P4.4,4.5 =USCI_A1 TXD/RXD

    // pin-to-pin UART setup
    UCA0CTL1 |= UCSWRST;                                // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                               // SMCLK
    UCA0BR0 = 104;                                      // 1MHz, 9600 Baud Rate
    UCA0BR1 = 0;                                        // 1MHz, 9600 Baud Rate
    UCA0MCTL |= UCBRF_0 + UCBRS_1;                      // Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                               // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                                   // Enable USCI_A0 RX interrupt

    // USB-to-pin UART setup
    UCA1CTL1 |= UCSWRST;                                // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                               // SMCLK
    UCA1BR0 = 104;                                      // 1MHz, 9600 Baud Rate
    UCA1BR1 = 0;                                        // 1MHz 9600 Baud Rate
    UCA1MCTL |= UCBRF_0 + UCBRS_1;                      // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                               // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                                   // Enable USCI_A1 RX interrupt
}

void timerA0setup()
{
    //TA0CCTL0 = CCIE;                                    // TA0CCR0 enabled
    //TA0CCTL1 = CCIE;                                    // TA0CCR1 enabled

    TA0CTL = TASSEL_2 | MC_1;                           // Timer A Control set to: SMCLK, Up-mode, no internal divider
                                                        // Value stored in Timer A register initialized to zero

    TA0CCR0 = 255;                                   //TimerA0 CCR0  initialized at 1000
    //TA0CCR1 = 127;                                   //TimerA0 CCR1 initialized
    TA0CCTL1 = OUTMOD_7;
    P1SEL |= BIT2;
    P1DIR |= BIT2;
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)

{

  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow

      case  6:                                  // Vector  6:  ADC12IFG0
      q = ADC12MEM0;
      Q = binaryToTemperature(q);
      if (Q >= 100)
          temp = Q - 128;
      else
          temp = Q;
      UCA1TXBUF = temp;
      //UCA1TXBUF = TA0CCR1 & 0x00FF; // to read PWM
     /* prevError = error;
      error = target - temp;
      I = prevError  +  error;
      D = prevError - error;
      P = error/target;
      pwm = (int)TA0CCR1-50*P;//;//-(10*I);
      if(pwm < 0 )
          TA0CCR1 = 0;
      else if(pwm > 255)
          TA0CCR1 = 255;
      else
          TA0CCR1  = pwm ;
    */
      //if (pwn > 0)
       //   TA0CCR1 = TA0CCR1 +error*100+I*200+D*50 ;
      //else if (pwm < 0)
        //  TA0CCR1 = 255;
      //else


          //TA0CCR1 = TA0CCR1 + error*100;



     // if (error > prevError && temp > target && TA0CCR1 < 0xFFFF)
          //TA0CCR1 = 0;

      /*if(target - temp < 0 ) // colder than it should be
      {
              TA0CCR1 = 0;
      }
      else if(target - temp > 0) // warmer than it should be
      {
              TA0CCR1 = 255;
      }*/
      watch = TA0CCR1;
    break;

  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}

/*
#pragma vector=TIMER0_A0_VECTOR                         // interrupt service routine for Timer 0 CCR0
__interrupt void TIMER0_A0(void)
{
    P2OUT |= BIT0;                                      // turns PWM-LED (D1) ON
}

#pragma vector=TIMER0_A1_VECTOR                         // interrupt service routine for Timer 0 CCR1
__interrupt void TIMER0_A1(void)
{
    // int dummy = TA0IV;
    switch(TA0IV)
    {
        case TA0IV_TA0CCR1:
        {
            P2OUT &= ~BIT0;                             // turns PWM-LED (D1) OFF
            break;
        }
        default:
            break;
    }
}*/

// USB-to-pin UART interrupt service routine
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_interrupt(void)
{
    switch (__even_in_range(UCA1IV, 4))
    {
        case 0:
            break;                                          // Vector 0 - no interrupt
        case 2:                                             // Vector 2 - RXIFG
            while (!(UCA1IFG & UCTXIFG));                   // USCI_A1 TX buffer ready?
                target = UCA1RXBUF;                             // turns PWM-LED (D1) OFF
            // WE WANT TO SET THE TARGET TEMPERATURE IN HERE

            break;
        case 4:
            break;                                          // Vector 4 - TXIFG
        default:
            break;
    }
}

unsigned char binaryToTemperature(double in)
{

    qin = (in*5)/(4096);
    qin = (15000*qin)/(5-qin);

    //if(qin > 19872 && qin <= 32554)
    if (qin > 19872)
    {
        return round(-0.00079*qin+25.67) + 20;
    }
    else if(qin > 12448 && qin <= 19872)
    {
        return round(-0.00135*qin+36.77) + 20;
    }
    else if(qin > 8059 && qin <= 12448)
    {
        return round(-0.00228*qin+48.36) + 20;
    }
    else if(qin > 5330 && qin <= 8059 )
    {
        return round(-0.00366*qin+59.53) + 20;
    }
    else if(qin > 3605 && qin <= 5330)
     {
        return round(-.0058*qin+70.90) + 20;
     }
    else if(qin > 2490 && qin <= 3605)
    {
        return round(-0.00897*qin+82.33) + 20;
    }
    else if(qin > 1753 && qin <= 2490)
     {
        return round(-0.01357*qin+93.79) + 20;
     }
    else if(qin > 1256 && qin <= 1753)
    {
        return round(-0.02012*qin+105.27) + 20;
    }
    else if(qin > 915 && qin <= 1256)
    {
        return round(-0.02936*qin+116.88) + 20;
    }
    //else if(qin > 677 && qin <= 915)
    else
    {
        return round(-0.042*qin+128.45) + 20;
    }
}
