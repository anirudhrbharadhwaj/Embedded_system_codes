Wireless Home Automation using Internet of Things (IoT)

About:
•	Accomplished modelling a solar tracker and moisture monitoring system which uses application program interface to send and receive data. 
•	Latitude and longitude of a place were used in a complex mathematical model which was coded to TM4C microcontroller to steer a solar panel tracking the sun and report information on solar intensity.  
•	Built a soil sensor using two 3-1/2 inch nail by calculating the potential difference between them. Implemented rules for the system to determine when to use the 9 Volt water pump.
Code:
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"


#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define GMT -5
#define LATITUDE  33
#define LONGITUDE -97
#define TURNLEFT 2500       //counter clockwise
#define TURNCENTER 3750     //90 degree
#define TURNRIGHT 5000      //Clockwise
#define pi 3.141592654


double longitude =LONGITUDE;
double latitude =LATITUDE;
double local_time=0;
uint16_t hour=20;
uint16_t minute=00;
uint32_t year=2018;
uint8_t month=5;
uint8_t day=2;

double LSTM=0;
double GTOffset=-5;
double EOT=0;

uint16_t days_since_start =0;
double B=0;
double Bd=0;

double TC_Factor=0;
double LS_Time=0;
double hour_Angle=0;
double sunsettime=0;
double sunrisetime=0;
double declination=0;
double zenith=0;
double elevation=0;
double azimuth=0;

char str[55]={0};
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void initHw()
{
      // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
               SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)| SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_16;

               // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
               SYSCTL_GPIOHBCTL_R = 0;

               // Enable GPIO port F peripherals
               SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB |SYSCTL_RCGC2_GPIOE;

               // Configure LED and pushbutton pins
               GPIO_PORTF_DIR_R |= 0x04;  // bits 1 and 2 are outputs, other pins are inputs
               GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
               GPIO_PORTF_DEN_R |= 0x14;  // enable LEDs and pushbuttons
               GPIO_PORTF_PUR_R |= 0x10;  // enable internal pull-up for push button


              /*// Configure external LED and Pushbutton pins
                   GPIO_PORTA_DIR_R |= 0xE0;  // bits 1 and 2 are outputs, other pins are inputs
                  GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
                  GPIO_PORTA_DEN_R |= 0xFC;  // enable LEDs and pushbuttons
                  GPIO_PORTA_PUR_R |= 0x1C;  // enable internal pull-up for push button

                  GPIO_PORTB_DIR_R |= 0x10;  // bits 1 and 2 are outputs, other pins are inputs
                     GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
                     GPIO_PORTB_DEN_R |= 0x50;  // enable LEDs and pushbuttons
                     GPIO_PORTB_PUR_R |= 0x40;   // enable internal pull-up for push button*/

                     // Configure UART0 pins
            SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0  ;         // turn-on UART0, leave other uarts in same status
            GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
            GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
            GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

            // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
               UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
               UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
               UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
               UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
               UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
               UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    /*           SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
                   WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
                   WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
                   WTIMER5_TAMR_R =  TIMER_TAMR_TAMR_PERIOD; // configure for edge time mode, count up
                   WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
                   WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
                   WTIMER5_TAV_R = 0xffffffff;                             // zero counter for first period
                   WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    */

               GPIO_PORTE_DIR_R  = 0x30;
                     GPIO_PORTE_DR2R_R |= 0x30;
                     GPIO_PORTE_DEN_R |= 0x30;
                     GPIO_PORTE_AFSEL_R |= 0x30;
                     GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5 ;
               SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
               waitMicrosecond(2500);
                        SYSCTL_RCGCPWM_R |=SYSCTL_RCGCPWM_R0;
                            __asm(" NOP");                                   // wait 3 clocks
                            __asm(" NOP");
                            __asm(" NOP");
                            SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM1 module
                            SYSCTL_SRPWM_R = 0;                              // leave reset state
                            PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2

                            PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;

                            PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;

                            PWM0_2_LOAD_R = 50000;                            // set period to 40 MHz sys clock / 16 / 50000 = 50Hz
                            PWM0_INVERT_R = PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;
                        PWM0_2_CMPA_R = 0;                                  // invert outputs for duty cycle increases with increasing compare values
                         PWM0_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)

                            PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2

                            PWM0_ENABLE_R = PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;

}

//Blocking function that writes a serial character when the UART buffer is not full  --DONE
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

//Blocking function that writes a string when the UART buffer is not full --DONE
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}



uint16_t daySinceStart()
{
    uint8_t year_offset =0;
    if(year % 4 ==0)
        year_offset=1;
    else
        year_offset=2;

    uint16_t hold=30*(month-1) -year_offset + ceil(month/2);

    return day+hold;
}
void calcTimeinHour()
{
    local_time=hour+minute/60.0;
}
void calcSunriseSunset()
{
	sunrisetime=12-((acos(-tan(latitude*(pi/180))*tan(declination*(pi/180)))*(180/pi))/15)-(TC_Factor/60.0);
	sunsettime=12+((acos(-tan(latitude*(pi/180))*tan(declination*(pi/180)))*(180/pi))/15)-(TC_Factor/60.0);
}
void solarTrackerCalc()
{
    LSTM= 15* GTOffset;
    uint16_t daysincestart=daySinceStart();
    calcTimeinHour();
    calcSunriseSunset();
    B= ((360.0/365.0)*(daysincestart-81));
    Bd=B*(pi/180);
    EOT= 9.87*sin(2*Bd)-7.53*cos(Bd)-1.5*sin(Bd);
    TC_Factor= 4*(longitude-LSTM)+EOT;
    LS_Time = local_time+TC_Factor/60.0;
    hour_Angle= 15*(LS_Time-12);
    declination = 23.45*sin(Bd);
    elevation = asin(sin(declination*(pi/180))*sin(latitude*(pi/180))+cos(declination*(pi/180))*cos(latitude*(pi/180))*cos((360-hour_Angle)*(pi/180)))*(180/pi);
    zenith=90-elevation;
    if(hour_Angle>0)
    {
	elevation=90+(90-23.5-elevation);
    }
    else

    azimuth = acos((sin(declination*(pi/180))*cos(latitude*(pi/180))-cos(declination*(pi/180))*sin(latitude*(pi/180))*cos(hour_Angle*(pi/180)))/cos(elevation*(pi/180)))*(180/pi);

    if(hour_Angle>0)
        azimuth= 360-azimuth;


}


void positionReset()
{
    PWM0_2_CMPB_R=3125;
    PWM0_2_CMPA_R=3125;
    waitMicrosecond(6000000);

}
double positionToPulse(double inputAngle)
{
    //return .6+(inputAngle*1.4/180.0);        //in milliseconds
    return .5+(inputAngle*1.5/180.0);        //in milliseconds
}

double pulseConversion(double pulse)
{
    return ceil((pulse/20.0)*50000.0);        //in milliseconds
}


int main(void)
{
	uint8_t i;
  // Initialize hardware
  initHw();

  double pulsenum1=0,pulsenum2=0;
  positionReset();

  while(1)
  {
for(i=7;i<20;i++)
{
	hour=i;
waitMicrosecond(200000);
	  solarTrackerCalc();
	  if(local_time>=7 && local_time<=20 && elevation>=0)
	  {
      pulsenum1 = pulseConversion(positionToPulse(elevation+23.5));
      pulsenum2 = pulseConversion(positionToPulse(zenith+23.5));
      PWM0_2_CMPB_R=pulsenum1;
      waitMicrosecond(200000);
      PWM0_2_CMPA_R=pulsenum2;
      waitMicrosecond(200000);
        }
  }
  }
  }






