# Embedded_system_codes
Project was done during 2nd Semester of my masters in Electrical Engineering Under Advanced Embedded Systems subject.
About:
Implemented a pre-emptive RTOS solution with support for semaphores, yielding, sleep, priority scheduling, priority inheritance, create thread, destroy thread, wait, post and a shell interface for an M4F controller. 
Adapted software internal interrupt systickISR() function to handle the sleep timing &amp; kernel functions and pendSCISR() function to switch task.
Code: 
// RTOS Framework - Spring 2018
// J Losh
// Student Name:Anirudh Ravi Shankar Bharadhwaj
// TO DO: Add your name on this line. Do not include your ID number.
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC: TM4C123GH6PM
// System Clock: 40 MHz
// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include<stdlib.h>
#include<ctype.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_nvic.h"
#include "hw_types.h"
// REQUIRED: correct these bitbanding references for the off-board LEDs
#define ONB_PUSH_BUTTON (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON_1 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON_2 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON_3 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON_4 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define ORANGE_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define YELLOW_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define MAX_CHAR 150
#define alpha 0.98
uint32_t
A=0,B=0,c=0,i=0,j=0,r=0,n=0,y=0,m=0,skip_count=0,w_count=0,count=0,count_3=0,field=0,
result=0;
uint32_t t1,t2;
uint32_t temp;
uint32_t total_time;
uint32_t value;
uint8_t buttons;
uint16_t code;
uint8_t temp_priority;
uint16_t address_1=0;
uint16_t str_pos[50];
bool priority_inheritance;
char* variable;
char attempt[100];
char str_type[50];
char str[MAX_CHAR+1]={0};
char str1[MAX_CHAR+1]={0};
char str2[MAX_CHAR+1]={0};
char str3[MAX_CHAR+1]={0};
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
// function pointer
typedef void (*_fn)();
// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
uint16_t count;
uint16_t queueSize;
uint32_t processQueue[MAX_QUEUE_SIZE];
char name[16];// store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;
struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
// task
#define STATE_INVALID 0 // no task
#define STATE_UNRUN 1 // task has never been run
#define STATE_READY 2 // has run, can resume at any time
#define STATE_BLOCKED 3 // has run, but now blocked by semaphore
#define STATE_DELAYED 4 // has run, but now awaiting timer
#define STATE_RUN 10
#define MAX_TASKS 10 // maximum number of valid tasks
uint8_t taskCurrent = 0; // index of last dispatched task
uint8_t taskCount = 0; // total number of valid tasks
uint16_t retranstimeout[MAX_TASKS]={0};
struct _tcb
{
uint8_t state; // see STATE_ values above
void *pid; // used to uniquely identify thread
void *sp; // location of stack pointer for thread
uint8_t priority; // 0=highest, 7=lowest
uint8_t skip_count;
uint8_t currentPriority; // used for priority inheritance
uint32_t CPU_time;
uint32_t time;
float bi;
uint32_t ticks; // ticks until sleep complete
char name[16]; // name of task used in ps command
void *semaphore; // pointer to the semaphore that is blocking the
thread
} tcb[MAX_TASKS];
uint32_t stack[MAX_TASKS][256]; // 1024 byte stack for each thread
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void rtosInit()
{
uint8_t i;
// no tasks running
taskCount = 0;
// clear out tcb records
for (i = 0; i < MAX_TASKS; i++)
{
tcb[i].state = STATE_INVALID;
tcb[i].pid = 0;
}
NVIC_ST_CTRL_R =0x00000007;
NVIC_ST_RELOAD_R =0x9C40;
}
uint32_t getSP()
{
__asm(" MOV R0,R13");
__asm(" BX LR");
}
void setSP(void* B)
{
__asm(" MOV R13,R0");
__asm(" BX LR");
}
void rtosStart()
{
_fn fn;
A =getSP();
taskCurrent = rtosScheduler();
setSP(tcb[taskCurrent].sp);
fn =(_fn)(1 | (uint32_t)(tcb[taskCurrent].pid));
(*fn)();
}
bool createThread(_fn fn, char name[], int priority)
{
bool ok = false;
uint8_t i = 0;
bool found = false;
if (taskCount < MAX_TASKS)
{
while (!found && (i < MAX_TASKS))
{
found = (tcb[i++].pid == fn);
}
if (!found)
{
i = 0;
while (tcb[i].state != STATE_INVALID) {i++;}
tcb[i].state = STATE_UNRUN;
tcb[i].pid = fn;
tcb[i].sp = &stack[i][255];
tcb[i].priority = priority;
tcb[i].currentPriority = priority;
for(j=0;j<16;j++)
{
tcb[i].name[j]=name[j];
}
// increment task count
taskCount++;
ok = true;
}
}
return ok;
}
// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
struct semaphore *pSemaphore = 0;
for(y=0;y<10;y++)
{
if(fn==tcb[y].pid)
{
if((tcb[y].state==STATE_READY)|tcb[y].state==STATE_DELAYED)
{
if(tcb[y].semaphore==0)
{
tcb[y].pid=0;
tcb[y].state=STATE_INVALID;
tcb[y].priority=0;
tcb[y].time=0;
tcb[y].CPU_time=0;
tcb[y].skip_count=0;
taskCount--;
break;
}
else
{
pSemaphore=tcb[y].semaphore;
pSemaphore->count++;
for(i=0;i<5;i++)
{
if(pSemaphore->processQueue[i]!=0)
for(j=0;j<10;j++)
{
if( pSemaphore-
>processQueue[i]==(uint32_t)tcb[j].pid)
{
pSemaphore->queueSize--;
pSemaphore->count--;
tcb[j].state =
STATE_READY;
pSemaphore-
>processQueue[i]=0;
break;
}
}
tcb[y].pid=0;
tcb[y].state=STATE_INVALID;
tcb[y].priority=0;
tcb[y].time=0;
tcb[y].CPU_time=0;
tcb[y].skip_count=0;
taskCount--;
break;
}
}
}
if(tcb[y].state==STATE_BLOCKED)
{
for(i=0;i<5;i++)
{
for(j=0;j<5;j++)
{
if(semaphores[i].processQueue[j]==(uint32_t)tcb[y].pid)
{
semaphores[i].processQueue[j]=0;
semaphores[i].queueSize--;
}
}
}
tcb[y].pid=0;
tcb[y].state=STATE_INVALID;
tcb[y].priority=0;
tcb[y].time=0;
taskCount--;
tcb[y].CPU_time=0;
tcb[y].skip_count=0;
}
}
}
}
void setThreadPriority(_fn fn, uint8_t priority)
{
for(i=0;i<10;i++)
{
if(tcb[i].pid==fn)
{
tcb[i].priority=priority;
tcb[i].currentPriority=priority;
break;
}
}
}
struct semaphore* createSemaphore(uint8_t count,char name[15])
{
struct semaphore *pSemaphore = 0;
if (semaphoreCount < MAX_SEMAPHORES)
{
pSemaphore = &semaphores[semaphoreCount++];
pSemaphore->count = count;
}
for(i=0;i<15;i++)
{
pSemaphore->name[i]=name[i];
}
return pSemaphore;
}
void yield()
{
tcb[taskCurrent].state = STATE_READY;
NVIC_INT_CTRL_R |= 0x10000000;
// push registers, call scheduler, pop registers, return to new function
}
// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
tcb[taskCurrent].ticks=tick;
tcb[taskCurrent].state = STATE_DELAYED;
NVIC_INT_CTRL_R |= 0x10000000; // push registers, set state to delayed, store
timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
}
// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
using pendsv
void wait(struct semaphore *pSemaphore)
{
if(pSemaphore->count>0)
{
pSemaphore->count--;
}
else
{
pSemaphore->processQueue[pSemaphore-
>queueSize]=(uint32_t)tcb[taskCurrent].pid;
pSemaphore->queueSize++;
tcb[taskCurrent].semaphore=pSemaphore;
tcb[taskCurrent].state = STATE_BLOCKED;
if(priority_inheritance==true)
{
if(pSemaphore->count==0)
{
for(j=0;j<MAX_TASKS;j++)
{
if((tcb[taskCurrent].semaphore==(void*)tcb[j].semaphore)&&(taskCurrent!=j))
{
if(
tcb[j].priority>tcb[taskCurrent].priority)
{
tcb[j].priority=tcb[taskCurrent].priority;
}
}
}
}
}
else
tcb[taskCurrent].priority=tcb[taskCurrent].currentPriority;
NVIC_INT_CTRL_R |= 0x10000000;
}
}
// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
pSemaphore->count++;
for(i=0;i<6;i++)
{
if(pSemaphore->processQueue[i]!=0)
{
for(j=0;j<11;j++)
{
if( pSemaphore->processQueue[i]==(uint32_t)tcb[j].pid)
{
tcb[taskCurrent].priority=tcb[taskCurrent].currentPriority;
pSemaphore->queueSize--;
pSemaphore->count--;
tcb[j].state = STATE_READY;
pSemaphore->processQueue[i]=0;
break;
}
}
}
else
break;
}
tcb[taskCurrent].semaphore=0x00;
tcb[taskCurrent].state = STATE_READY;
NVIC_INT_CTRL_R |= 0x10000000;
}
int rtosScheduler()
{
bool ok;
static uint8_t task = 0xFF;
ok = false;
while (!ok)
{
task++;
if (task >= MAX_TASKS)
task = 0;
if((tcb[task].skip_count>=tcb[task].priority)&&(tcb[task].pid!=0))
{
ok = (tcb[task].state == STATE_READY || tcb[task].state ==
STATE_UNRUN);
tcb[task].skip_count=0;
}
else
{
if(tcb[task].pid!=0)
tcb[task].skip_count++;
}
}
return task;
}
// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
value++;
for(c=0;c<11;c++)
{
if (tcb[c].ticks>0)
tcb[c].ticks--;
if((tcb[c].ticks==0) && (tcb[c].state==STATE_DELAYED)==1)
{
tcb[c].state = STATE_READY;
}
}
if(value==1000)
{
for(i=0;i<10;i++)
{
//tcb[i].CPU_time=0;
tcb[i].time=0;
tcb[i].bi=0;
}
value=0;
total_time=0;
}
}
// REQUIRED: in coop and preemptive, modify this function to add support for task
switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
__asm(" add sp,#16");
t2= WTIMER5_TAV_R;
if(t1>t2){
tcb[taskCurrent].time=tcb[taskCurrent].time+(t1-t2);
total_time =total_time+(t1-t2);
tcb[taskCurrent].bi=(alpha*(tcb[taskCurrent].bi)+((1-
alpha)*tcb[taskCurrent].time));
tcb[taskCurrent].CPU_time=((tcb[taskCurrent].bi*100)/total_time);
}
__asm(" PUSH{R4,R5,R6,R7,R8,R9,R10,R11}");
NVIC_INT_CTRL_R |= 0x08000000;
tcb[taskCurrent].sp=(void*)getSP();
setSP((void*)A);
taskCurrent = rtosScheduler();
if(tcb[taskCurrent].state == STATE_READY)
{
setSP(tcb[taskCurrent].sp);
__asm(" POP{R4,R5,R6,R7,R8,R9,R10,R11}");
tcb[taskCurrent].state = STATE_RUN;
}
if( tcb[taskCurrent].state == STATE_UNRUN )
{
setSP(tcb[taskCurrent].sp);
__asm(" sub sp,#8");
stack[taskCurrent][253]=0x41000200;
stack[taskCurrent][252]=((uint32_t)(tcb[taskCurrent].pid)-1);
stack[taskCurrent][251]=(1 | (uint32_t)(tcb[taskCurrent].pid));
__asm(" sub sp,#8");
__asm(" PUSH{R0,R1,R2,R3,R12} ");
tcb[taskCurrent].state = STATE_RUN;
}
WTIMER5_TAV_R = 0xFFFFFFFF;
t1= WTIMER5_TAV_R;
__asm(" mov R14, #0xffff");
__asm(" ROR R14,#16");
__asm(" mov R0, #0xfff9");
__asm(" add LR, R0");
__asm(" BX LR");
}
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN |
SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
// Set GPIO ports to use AP (not needed since default configuration -- for
clarity)
SYSCTL_GPIOHBCTL_R = 0;
// Enable GPIO port F peripherals
SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOA |
SYSCTL_RCGC2_GPIOB;
// Configure LED and pushbutton pins
GPIO_PORTF_DIR_R |= 0x04; // bits 1 and 2 are outputs, other pins are
inputs
GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since
default configuration -- for clarity)
GPIO_PORTF_DEN_R |= 0x14; // enable LEDs and pushbuttons
GPIO_PORTF_PUR_R |= 0x10; // enable internal pull-up for push button
// Configure external LED and Pushbutton pins
GPIO_PORTA_DIR_R |= 0xE0; // bits 1 and 2 are outputs, other pins are
inputs
GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed
since default configuration -- for clarity)
GPIO_PORTA_DEN_R |= 0xFC; // enable LEDs and pushbuttons
GPIO_PORTA_PUR_R |= 0x1C; // enable internal pull-up for push button
GPIO_PORTB_DIR_R |= 0x10; // bits 1 and 2 are outputs, other pins are
inputs
GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed
since default configuration -- for clarity)
GPIO_PORTB_DEN_R |= 0x50; // enable LEDs and pushbuttons
GPIO_PORTB_PUR_R |= 0x40; // enable internal pull-up for push
button
// Configure UART0 pins
SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0 ; // turn-on UART0, leave
other uarts in same status
GPIO_PORTA_DEN_R |= 3; // default, added for
clarity
GPIO_PORTA_AFSEL_R |= 3; // default, added for
clarity
GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock
enable and config writes)
UART0_CTL_R = 0; // turn-off UART0 to
allow safe programming
UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40
MHz)
UART0_IBRD_R = 21; // r = 40 MHz /
(Nx115.2kHz), set floor(r)=21, where N=16
UART0_FBRD_R = 45; // round(fract(r)*64)=45
UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/
16-level FIFO
UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX,
RX, and module
SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5; // turn-on timer
WTIMER5_CTL_R &= ~TIMER_CTL_TAEN; // turn-off counter
before reconfiguring
WTIMER5_CFG_R = 4; // configure as 32-
bit counter (A only)
WTIMER5_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for edge time
mode, count up
WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS; // measure time from
positive edge to positive edge
WTIMER5_IMR_R = TIMER_IMR_CAEIM; // turn-on interrupts
WTIMER5_TAV_R = 0xffffffff; // zero
counter for first period
WTIMER5_CTL_R |= TIMER_CTL_TAEN; // turn-on counter
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
// Approx clocks per us
__asm("WMS_LOOP0: MOV R1, #6"); // 1
__asm("WMS_LOOP1: SUB R1, #1"); // 6
__asm(" CBZ R1, WMS_DONE1"); // 5+1*3
__asm(" NOP"); // 5
__asm(" B WMS_LOOP1"); // 5*3
__asm("WMS_DONE1: SUB R0, #1"); // 1
__asm(" CBZ R0, WMS_DONE0"); // 1
__asm(" B WMS_LOOP0"); // 1*3
__asm("WMS_DONE0:"); // ---
// 40 clocks/us + error
}
// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are
pressed
uint8_t readPbs()
{
temp=0;
if(ONB_PUSH_BUTTON==0)
{
temp |=1;
}
if(PUSH_BUTTON_1==0)
{
temp |=8;
}
if(PUSH_BUTTON_2==0)
{
temp |=4;
}
if(PUSH_BUTTON_3==0)
{
temp |=2;
}
if(PUSH_BUTTON_4==0)
{
temp |=16;
}
return temp;
}
// ------------------------------------------------------------------------------
// Task functions
// ------------------------------------------------------------------------------
// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
__asm(" add sp,#8");
while(true)
{
ORANGE_LED = 1;
waitMicrosecond(1000);
ORANGE_LED = 0;
yield();
}
}
void flash4Hz()
{
__asm(" add sp,#8");
while(true)
{
GREEN_LED ^= 1;
sleep(125);
}
}
void oneshot()
{
while(true)
{
wait(flashReq);
YELLOW_LED = 1;
sleep(1000);
YELLOW_LED = 0;
}
}
void partOfLengthyFn()
{
// represent some lengthy operation
waitMicrosecond(1000);
// give another process a chance to run
yield();
}
void lengthyFn()
{
uint16_t i;
while(true)
{
wait(resource);
for (i = 0; i < 4000; i++)
{
partOfLengthyFn();
}
RED_LED ^= 1;
post(resource);
}
}
void readKeys()
{
while(true)
{
wait(keyReleased);
buttons = 0;
while (buttons == 0)
{
buttons = readPbs();
yield();
}
post(keyPressed);
if ((buttons & 1) != 0)
{
YELLOW_LED ^= 1;
RED_LED = 1;
}
if ((buttons & 2) != 0)
{
post(flashReq);
RED_LED = 0;
}
if ((buttons & 4) != 0)
{
createThread(flash4Hz, "Flash4Hz", 0);
}
if ((buttons & 8) != 0)
{
destroyThread(flash4Hz);
}
if ((buttons & 16) != 0)
{
setThreadPriority(lengthyFn, 4);
}
yield();
}
}
void debounce()
{
uint8_t count;
while(true)
{
wait(keyPressed);
count = 10;
while (count != 0)
{
sleep(10);
if (readPbs() == 0)
count--;
else
count = 10;
}
post(keyReleased);
}
}
void uncooperative()
{
while(true)
{
while (readPbs() == 8)
{
}
yield();
}
}
void important()
{
while(true)
{
wait(resource);
BLUE_LED = 1;
sleep(1000);
BLUE_LED = 0;
post(resource);
}
}
void putcUart0(char c)
{
while (UART0_FR_R & UART_FR_TXFF);
UART0_DR_R = c;
}
void putsUart0(char* str)
{
uint8_t i;
for (i = 0; i < strlen(str); i++)
putcUart0(str[i]);
}
void putnUart0(uint32_t number)
{
uint32_t count_1=0,i,temp,temp2[10]={0};
temp=number;
for(i=0;i<9;i++)
{
if((temp%10!=0)|(temp!=0))
{
count_1++;
temp=temp/10;
}
else
break;
}
for(i=count_1;i>0;i--)
{
if(number!=0)
{
temp2[i]=number%10+'0';
number=number/10;
}
}
for(i=0;i<=count_1;i++)
{
putcUart0(temp2[i]);
}
if(count_1==0)
{
putsUart0("0");
}
}
void putfUart0(uint32_t number)
{
uint32_t count_1=0,i,temp,temp2[10]={0};
temp=number;
for(i=0;i<9;i++)
{
if((temp%10!=0)|(temp!=0))
{
count_1++;
temp=temp/10;
}
else
break;
}
for(i=count_1;i>0;i--)
{
if(number!=0)
{
temp2[i]=number%10+'0';
number=number/10;
}
}
for(i=0;i<=count_1;i++)
{
putcUart0(temp2[i]);
}
if(count_1==0)
{
putsUart0("0");
}
}
char* getstring(uint16_t field_1)
{
variable= &str3[str_pos[field_1]];
return variable;
}
int32_t getnumber(uint32_t field_1)
{
address_1= atoi(&str1[str_pos[field_1]]);
return address_1;
}
uint8_t stringlen(char name[100])
{
uint8_t j=0,count_2=0;
while(name[j]!='\0')
{
j++;
count_2++;
}
return count_2;
}
bool numstringcmp(char name[100],char name1[100] )
{
uint8_t i,j,count_2=0;
for(j=0;j<15;j++)
{
if(name[j]!='\0')
{
count_2++;
}
else break;
}
for(i=0;i<count_2;i++)
{
if(name[i]==name1[i])
{
}
else
break;
}
if(i==count_2)
{
return 0;
}
else return 1;
}
char getcUart0()
{
while (UART0_FR_R & UART_FR_RXFE)
{
yield();
}
return UART0_DR_R & 0xFF;
}
bool iscommand(char* str2,uint8_t minarguements)
{
if(numstringcmp("ps",str2)==0)
{
if(field==1)
return 1;
}
return 0;
}
void shell()
{
struct semaphore *pSemaphore=0;
while (true)
{
while(1)
{
i=0;
char c = getcUart0();
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
str[count]=' ';
str3[count]=' ';
count++;
break;
}
// check if its a Alphabet and number i.e Greater than space
ASCII=20h
if((c>=' ')|(c=='&'))
{
str3[count]= c;
str[count] = tolower(c);
count++;
if(count>=MAX_CHAR)
{
str[count]=0;
str3[count]=0;
count++;
break;
}
}
}
//Differentiate the input string as numbers and alphabets
// Calculate Field, String type, Sptring position
// Finally replace all delimitters by Null
for(i=0;i< stringlen(str);i++)
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
-- i;
r++;
}
else if((str[i]>='a'&&str[i]<='z'))
{
field=field+1;
str_pos[r]=i;
str_type[r]='a';
while((str[i]>='a'&&str[i]<='z'))
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
putsUart0(str);
putsUart0("\n\r");
if(iscommand(str2,field-1))
{
if(numstringcmp("ps",str2)==0)
{
putsUart0("\n \r PID\t\t Name\t\t CPU%\t\t State\t\t");
for(n=0;n<11;n++)
{
if((tcb[n].pid!=0)&(tcb[n].state!=STATE_INVALID))
{
putsUart0("\n \r");
putnUart0((uint32_t)tcb[n].pid);
putsUart0("\t\t");
putsUart0(&tcb[n].name[0]);
for(i=0;i<20-stringlen(tcb[n].name);i++)
{
putsUart0(" ");
}
putnUart0((uint32_t)tcb[n].CPU_time);
putsUart0("\t\t");
if(tcb[n].state==1)
{
putsUart0("UNRUN");
}
else if(tcb[n].state==2)
{
putsUart0("READY");
}
else if(tcb[n].state==3)
{
putsUart0("BLOCKED");
}
else if(tcb[n].state==4)
{
putsUart0("DELAYED");
}
else if(tcb[n].state==10)
{
putsUart0("RUN");
}
// putsUart0("\n \r");
}
}
}
}
if(numstringcmp("pidof",str2)==0)
{
variable=getstring(1);
for(y=0;y<10;y++)
{
if(numstringcmp(&(tcb[y].name[0]),variable)==0)
{
putnUart0((uint32_t)tcb[y].pid);
break;
}
}
}
if(numstringcmp("reboot",str2)==0)
{
putsUart0("Reset Successful");
waitMicrosecond(100000);
HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}
if(numstringcmp("ipcs",str2)==0)
{
putsUart0("\n \r Semaphore Name\t Count\t PID in Semaphore
queue\t Semaphore ID\t\n\r");
for(n=0;n<4;n++)
{
putsUart0("\n \r");
putsUart0("\n \r ");
putsUart0(&(semaphores[n].name[0]));
for(i=0;i<20-strlen(semaphores[n].name);i++)
{
putsUart0(" ");
}
putnUart0(semaphores[n].count);
putsUart0("\t\t ");
count_3=0;
for(m=0;m<4;m++)
{
if(semaphores[n].processQueue[m]!=0)
{
count_3++;
putnUart0((semaphores[n].processQueue[m]));
}
else
{
if( count_3==0)
{
putnUart0(0);
putnUart0(0);
putnUart0(0);
putnUart0(0);
break;
}
}
}
for(i=0;i<16;i++)
{
putsUart0(" ");
}
putnUart0((pSemaphore=&semaphores[n]));
pSemaphore=0;
// putsUart0("\n \r");
}
}
if(numstringcmp("Idle&",str3)==0)
{
createThread(idle, "Idle", 2);
}
if(numstringcmp("Flash4Hz&",str3)==0)
{
createThread(flash4Hz, "Flash4Hz", 2);
}
else if(numstringcmp("LengthyFn&",str3)==0)
{
createThread(lengthyFn, "LengthyFn", 6);
}
else if(numstringcmp("OneShot&",str3)==0)
{
createThread(oneshot, "OneShot", 2);
}
else if(numstringcmp("ReadKeys&",str3)==0)
{
createThread(readKeys, "ReadKeys", 6);
}
else if(numstringcmp("Debounce&",str3)==0)
{
createThread(debounce, "Debounce", 6);
}
else if(numstringcmp("Important&",str3)==0)
{
createThread(important, "Important", 1);
}
if(numstringcmp("kill",str2)==0)
{
destroyThread(getnumber(1));
}
if(numstringcmp("pi",str2)==0)
{
variable= getstring(1);
if(numstringcmp("on ",variable)==0)
{
priority_inheritance=true;
}
else if(numstringcmp("off ",variable)==0)
{
priority_inheritance=false;
}
}
for(i=0;i<MAX_CHAR;i++)
{
str[i] = 0;
str1[i]=0;
str2[i]=0;
str3[i]=0;
}
count=0;
r=0;
field=0;
}
}
//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
bool ok;
// Initialize hardware
initHw();
rtosInit();
// Power-up flash
GREEN_LED = 1;
waitMicrosecond(250000);
GREEN_LED = 0;
waitMicrosecond(250000);
// Initialize semaphores
keyPressed = createSemaphore(1,"keyPressed");
keyReleased = createSemaphore(0,"keyReleased");
flashReq = createSemaphore(5,"flashReq");
resource = createSemaphore(1,"resource");
// Add required idle process
ok = createThread(idle, "Idle", 7);
// Add other processes
ok &= createThread(lengthyFn, "LengthyFn", 6);
ok &= createThread(flash4Hz, "Flash4Hz", 2);
ok &= createThread(oneshot, "OneShot", 2);
ok &= createThread(readKeys, "ReadKeys", 6);
ok &= createThread(debounce, "Debounce", 6);
ok &= createThread(important, "Important", 0);
ok &= createThread(uncooperative, "Uncoop", 5);
ok &= createThread(shell, "Shell", 4);
// Start up RTOS
if (ok)
rtosStart(); // never returns
else
RED_LED = 1;
return 0;
}
