Peer-to-Peer communication system, supporting a Carrier Sense Multiple Access (CSMA) protocol
This project was done in my First semester of my masters in Electrical Engineering under Embedded Systems course. 
About:
•	Designed a communication system using Tiva ARM Cortex M4F (TM4C123GH6PMI microcontroller).
•	The construction phase required building a node capable of interfacing with a PC so that text
    commands can be entered by the user. Based on the commands, subsequent transmission on the 2-wire
     RS-485 bus to other nodes will was executed.
•	UART, SPI, Interrupts, Timers and Counter, PWM concept were used.
Code:
//Anirudh Ravi Shankar Bharadhwaj
//1001556502
// Final Project
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include<stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define DEN          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define BRED_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define BGREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define MAX_CHAR 80
#define MAX_MSGS 25
#define MAX_RETRYS 5


// Initializing Global   Variables
char str[MAX_CHAR+1]={0};
char str1[MAX_CHAR+1]={0};
char str2[MAX_CHAR+1]={0};
uint8_t destaddress[MAX_MSGS];
uint8_t srcaddress=002;
uint8_t sequenceID[MAX_MSGS]={0};
uint8_t command[MAX_MSGS];
uint8_t channel[MAX_MSGS];
uint8_t size[MAX_MSGS];
uint8_t checksum[MAX_MSGS];
uint8_t data[MAX_MSGS];
uint8_t retrans_count[MAX_MSGS];
uint8_t seqID=0;
uint8_t currentindex;
uint8_t currentphase;
uint8_t rxphase=0;
uint8_t rxdata[MAX_MSGS];
bool valid[MAX_MSGS];
bool ackreq[MAX_MSGS];
bool inprogress=false;
uint8_t retranscount[MAX_MSGS];
uint16_t retranstimeout[MAX_MSGS]={0};
uint8_t count=0;
uint16_t data_1;
uint32_t a=0,b=0,q=0,p,r,j,field=0,address=0,value=0,size1=0,address_1=0,command1=0,channel1=0,valid1=0,new_address=0;
int16_t str_pos[50];
char str_type[50];
bool cs_enable=false;
bool random_enable=false;
bool ack_enable;
char* variable;
int N=0;
uint8_t check_sum=0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,E,C and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;
       // enable LEDs

    // Board Red LED config
       GPIO_PORTE_DIR_R  = 0x08;
       GPIO_PORTE_DR2R_R |= 0x08;
       GPIO_PORTE_DEN_R |= 0x08;

       // Board Red LED config
             GPIO_PORTA_DIR_R  = 0x80;
             GPIO_PORTA_DR2R_R |= 0x80;
             GPIO_PORTA_DEN_R |= 0x80;

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

       // Configure UART1 pins
      SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
      GPIO_PORTC_DEN_R |= 0x70;                           // default, added for clarity
      GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
      GPIO_PORTC_DIR_R = 0x40;
      GPIO_PORTC_DR2R_R = 0x40;
      GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

//config UART1
         UART1_CTL_R = 0;
         UART1_CC_R = UART_CC_CS_SYSCLK;
         UART1_IBRD_R = 65;
         UART1_FBRD_R = 07;
         UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS;
         UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

         // Configure Timer 1 as the time base
                SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
                TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
                TIMER1_TAILR_R = 0x9C40;                      // set load value to 40e3 for 1000 Hz interrupt rate
                TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}


// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

uint8_t getnUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);
    return UART1_DR_R & 0xFF;
}

processpacket()
{
    uint8_t i,check_sum_1;
    check_sum = 0;
    for(i=0;i<7 ;i++)
    {
        check_sum=check_sum + rxdata[i];
    }
    check_sum_1=~check_sum;
if(check_sum_1==rxdata[7])
{
        if(rxdata[4]==1)
    {
                    if(rxdata[6])
                        GREEN_LED ^=1;
                    else GREEN_LED=0;
    }
}
else
BGREEN_LED=1;
if(rxdata[3]==0x70)
{

     for(i=0;i<25;i++)
     {
        if(valid[i]==1)
                {
            if(sequenceID[i]==rxdata[6])
            {
            valid[i]=0;
            putsUart0("\r\n Ack Recieved");
                        break;
            }

               }
     }

}


}
bool iscommand(char* str2,uint8_t minarguements)
{
   if((strcmp("set",str2)==0 && minarguements==3 ) || (strcmp("reset",str2)==0 && minarguements==1) || (strcmp("get",str2) ==0 && minarguements==2) || (strcmp("cs",str2) ==0 && minarguements==1) || (strcmp("random",str2) ==0 && minarguements==1) || (strcmp("ack",str2) ==0 && minarguements==1) || (strcmp("sa",str2) ==0 && minarguements==2))
   {
       if(field>minarguements)
           return 1;
       else return 0;
   }
   else if(strcmp("poll",str2)==0)
   {
       if(field==1)
           return 1;
       else return 0;
   }
   putsUart0("error\n\r");
   return 0;
}
char* getstring(uint16_t field_1)
{
    variable= &str1[str_pos[field_1]];
return variable;
}
int16_t getnumber(uint16_t field_1)
{
   address_1=  atoi(&str1[str_pos[field_1]]);
   return address_1;
}
sendpacket(uint8_t destadd,uint8_t cmd,uint8_t size1, uint8_t channel_1,uint8_t data_1)
{

    if(valid[a]==0)
    {
       destaddress[a]=destadd;
       if(ack_enable==1)
       {
           if(cmd!=0x70)
           {
           command[a]=cmd | 0x80;
           ackreq[a]=true;
              }
             else
           command[a]=cmd;
       }
           else command[a]=cmd;
       channel[a]=channel_1;
       size[a]=size1;
       data[a]=data_1;

       sequenceID[a]=seqID;
       seqID++;
       checksum[a]=~(destaddress[a]+srcaddress+command[a]+channel[a]+size[a]+data[a]+sequenceID[a]);
       valid[a]=1;


}
}
void Timer1Isr()
{
    if(!inprogress)
    {
           for(q=0;q<25;q++)
                        {
               if((valid[q]==1) && (retranstimeout[q]==0) )
               {
               inprogress=true;
               currentindex=q;
               currentphase=0;
               break;
               }

               }
    }
            if(inprogress)
        {
                if(cs_enable)
                {
                    if(rxphase==0)
                    {
                        goto tx1;
                    }
                    else goto rx1;
                }


    tx1:        if(currentphase==0)
            {
               UART1_LCRH_R =0xF2;
            DEN=1;
            BRED_LED =1;
               UART1_DR_R = destaddress[currentindex];
               currentphase++;
            }
            if(!(UART1_FR_R & UART_FR_BUSY))
            {
                if(currentphase==1)
            {
            UART1_LCRH_R =0xF6;
            putcUart1(srcaddress);
            currentphase++;
            putcUart1(sequenceID[currentindex]);
            currentphase++;
            putcUart1(command[currentindex]);
            currentphase++;
            putcUart1(channel[currentindex]);
            currentphase++;
            putcUart1(size[currentindex]);
            currentphase++;
            putcUart1(data[currentindex]);
            currentphase++;
            BRED_LED =0;
            putcUart1(checksum[currentindex]);
            currentphase++;
            }
            if(currentphase==7+size[currentindex])
            {
                        if((UART1_FR_R & UART_FR_BUSY)==0)
            {
                            inprogress=false;
          DEN=0;

          currentphase=0;
            if(!ackreq[currentindex])
            valid[currentindex]=0;
            else
                {
                retranscount[currentindex]++;
                putsUart0("\n \r");
                putsUart0("retransmitting");
                }
            if(retranscount[currentindex]>MAX_RETRYS)
            {
                N=0;
                valid[currentindex]=0;
            BRED_LED=1;
            retranstimeout[currentindex]=0;
            }
            else
                retranstimeout[currentindex]=500+100*(2^N);
            N++;
            }
            }
                    }
                    }

            for(b=0;b<25;b++)
        {
       if(retranstimeout[b]>0)
       retranstimeout[b]--;

        }
      //RED_LED ^=1;

       //Recieve program

   rx1:  if(currentphase==0)
   {
   while(!(UART1_FR_R & UART_FR_RXFE))
       {
          data_1=UART1_DR_R;

       if(data_1 & 0x200)
       {
           rxphase=0;
           BGREEN_LED=1;
           rxdata[rxphase]=data_1 & 0xFF;

           if(rxdata[rxphase]==srcaddress)
           {
           rxphase++;
            }
       }
       else if(rxphase!=0)
           {
               rxdata[rxphase]=data_1;
           rxphase++;
           }
       if(rxphase==8)
       {
           BGREEN_LED=0;
           processpacket();
           rxphase=0;
           if(rxdata[3] & 0x80)
          {
                 sendpacket(rxdata[1],0x70,0x01,0x00,rxdata[2]);
                 putsUart0("Ack Sent\r\n");
          }
       }

       }


    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
   }
   }

//
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    int i;

    for(i=0;i<MAX_MSGS;i++)
    {
        valid[i]=0;
    }
    // Initialize hardware
    initHw();
    // DISPLAY STATUS
     putsUart0("READY\r\n");

              for(i=0;i<1;i++)
              {
     BRED_LED = 1;
              waitMicrosecond(500000);
             BRED_LED = 0;
              waitMicrosecond(500000);
            }
while(1)
{
                  putsUart0("\n\r");
                  putsUart0("Enter String-");


                  while(1)
                  {
                      int i=0;
                 char     c = getcUart0();

                 //check if its a backspace
                  if(c == '\b')
                  {
                    if(count!=0)
                      {
                          count--;
                      }
                   }
                  // check if its an Enter
                  if(c=='\r')
                  {
                   str[count++]=' ';

                   break;
                  }
                  // check if its a Alphabet and number i.e Greater than space ASCII=20h
                  if(c>=' ')
                  {
                      str[count++] = tolower(c);
                      if(count>=MAX_CHAR)
                  {
                      str[count++]=0;
                      break;
                  }

                  }

                  }
                  // Entered String
                  putsUart0(str);

                  //Differentiate the input string as numbers and alphabets
                  // Calculate Field, String type, Sptring position
                  // Finally replace all delimitters by Null
                  for(i=0;i<strlen(str);i++)
                  {
                       if((str[i]>='0'&&str[i]<='9'))
                          {
                            str_pos[r]=i;
                             field=field+1;
                            str_type[r]='n';
                            while((str[i]>='0'&& str[i]<='9'))
                              {
                                str1[i]=str[i];
                                ++i;
                                }
                                --i;
                                r++;
                          }

                                else if((str[i]>='A'&&str[i]<='Z')||(str[i]>='a'&&str[i]<='z'))
                                    {

                                              field=field+1;
                                              str_pos[r]=i;
                                              str_type[r]='a';
                                              while((str[i]>='A'&&str[i]<='Z')||(str[i]>='a'&&str[i]<='z'))
                                              {
                                                  str1[i]=str[i];
                                                  str2[i]=str[i];
                                                  ++i;
                                              }
                                              r++;
                                              i--;
                                          }

                                            else
                                            {
                                                str1[i]='\0';
                                            }
                  }

                  putsUart0("\r\n");
                  putsUart0("parsed string\n\r");
                  putsUart0(str1);
                  putsUart0("\n\r");
                  putsUart0(str_type);

for(i=0;i<MAX_MSGS;i++)
{
if (iscommand(str2,field-1))
{
   if(strcmp("set",str2)==0)
        {
   //    UART1_LCRH_R |=0xF2;
       address=getnumber(1);
       //UART1_LCRH_R |=0xF6;
       channel1=getnumber(2);
       value=getnumber(3);
       command1=0x00;
       size1=0x01;
       valid1=1;
       sendpacket(address,command1,size1,channel1,value);
       putsUart0("Success\n\r");
       seqID=seqID+1;
break;
        }
   else if(strcmp("reset",str2)==0)
           {
          address=getnumber(1);
            }
   else if(strcmp("cs",str2)==0)
   {
       getstring(1);
       if(strcmp("on",variable)==0)
       {
           cs_enable=true;
           putcUart0("CS is Enabled");
   }
       else
           cs_enable=false;
   }
   else if (strcmp("get",str2)==0)
   {
       address=getnumber(1);
       channel1=getnumber(2);
   }
   else if(strcmp("random",str2)==0)
   {
       if(strcmp("on",variable)==0)
             {
                 random_enable=true;
         }
   }
   else if(strcmp("sa",str2)==0)
              {
             address=getnumber(1);
             new_address=getnumber(2);
               }
   else if(strcmp("ack",str2)==0)
     {
         getstring(1);
         if(strcmp("on",variable)==0)
         {
             ack_enable=true;
             break;
     }
         else if(strcmp("off",variable))
             ack_enable=false;
              }
 }

break;
}

for(i=0;i<MAX_CHAR;i++)
{
    str[i] = 0;
    str1[i]=0;
   str2[i]=0;
 //  str_type[i]=0;
  //  str_pos[i]=0;
}
variable=0;
count=0;
r=0;
field=0;

}
}








