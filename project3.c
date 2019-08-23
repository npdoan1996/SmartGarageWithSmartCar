// project3.c
// Runs on LM4F120 or TM4C123
// Use the SysTick timer interrupt, Port B and F interrupts
// to detect object and control the mode for two stepper
// motors. Port C and D is utilized to drive the stepper
// motors
// Nguyen Doan
// May 3, 2019

	
// Constant declarations to access port registers using 
// symbolic names instead of addresses
#include "systick.h"

// Port F and its interrupts
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define LEDS 										(*((volatile unsigned long *)0x40025038))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control

//Port C and its interrupt
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  
#define NVIC_PRI0_R             (*((volatile unsigned long *)0xE000E400))  
#define GPIO_PORTC_IS_R         (*((volatile unsigned long *)0x40006404))
#define GPIO_PORTC_IBE_R        (*((volatile unsigned long *)0x40006408))
#define GPIO_PORTC_IEV_R        (*((volatile unsigned long *)0x4000640C))
#define GPIO_PORTC_IM_R         (*((volatile unsigned long *)0x40006410))
#define GPIO_PORTC_RIS_R        (*((volatile unsigned long *)0x40006414))
#define GPIO_PORTC_ICR_R        (*((volatile unsigned long *)0x4000641C))
#define GPIO_PORTC_DATA_R       (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_PUR_R        (*((volatile unsigned long *)0x40006510))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_CR_R         (*((volatile unsigned long *)0x40006524))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile unsigned long *)0x4000652C))
#define GPIO_PORTC_DR8R_R       (*((volatile unsigned long *)0x40006508))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
//Port B and its interrupt
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI0_R             (*((volatile unsigned long *)0xE000E400))  // IRQ 13 to 15 Priority Register
#define GPIO_PORTB_IS_R         (*((volatile unsigned long *)0x40005404))
#define GPIO_PORTB_IBE_R        (*((volatile unsigned long *)0x40005408))
#define GPIO_PORTB_IEV_R        (*((volatile unsigned long *)0x4000540C))
#define GPIO_PORTB_IM_R         (*((volatile unsigned long *)0x40005410))
#define GPIO_PORTB_RIS_R        (*((volatile unsigned long *)0x40005414))
#define GPIO_PORTB_ICR_R        (*((volatile unsigned long *)0x4000541C))
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define SENSOR    					   	(*((volatile unsigned long *)0x40005040))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTB_DR8R_R       (*((volatile unsigned long *)0x40005508))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

// Port D 
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
	
// Systick interrupt 
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))

// color declaration
#define RED 										0x02
#define BLUE 										0x04
#define GREEN 									0x08
#define ON								 			1
#define OFF											0

struct State{
  unsigned char Out;     // Output
  unsigned char Next[2]; // CW/CCW
};
typedef const struct State StateType;

#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
StateType fsm[4]={
  { 0x03,{1,3}},
  { 0x06,{2,0}},
  {	0x0c,{3,1}},
  { 0x09,{0,2}}
};
unsigned char L;
unsigned char R; 

#define STEPPER1  (*((volatile unsigned long *)0x4000703C)) //PD0,1,2,3(left)
#define STEPPER2  (*((volatile unsigned long *)0x400063C0)) //PC4,5,6,7(right)
#define T1ms 16000 
	
// 	 Declarations Section
//   Global Variables
unsigned char mode;
unsigned char object;
unsigned int coefficient = 6;
unsigned int counter2 = 0;

//   Function Prototypes
void PortF_Init(void);
void PortB_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);  // low power mode
void ButtonInterrupt_Init(void);
void SensorInterrupt_Init(void);
//void SysTickI_Init(unsigned long period);
void Stepper_Init1(void);
void Stepper_Init2(void);
void Stepper_CW(unsigned long delay);
void Stepper_CCW(unsigned long delay);
void Forward(unsigned long delay);
void Backward(unsigned long delay);
void Left(unsigned long delay);
void Right(unsigned long delay);


	
// MAIN: Mandatory for a C Program to be executable
int main(void){  
  PortF_Init();       			
	PortB_Init();
	Stepper_Init1();
	Stepper_Init2();
	SysTick_Init();  
	ButtonInterrupt_Init();           // initialize GPIO Port F interrupt	
	SensorInterrupt_Init();						// initialize GPIO Port B interrupt
	EnableInterrupts();
  while(1){
		unsigned int i=0;	
		mode = 2;
		WaitForInterrupt();
		if (mode == 0){
			for (i=0;i<2048*2; i++) {
				Forward(coefficient*T1ms);   // output every 10ms
			}
			for (i=0;i<1024; i++) {
				Left(coefficient*T1ms);   // output every 10ms
			}
			while(object == 0){
				Forward(coefficient*T1ms);
			}
			while(object == 1);
			for (i=0;i<2048; i++) {
				Forward(coefficient*T1ms);   // output every 10ms
			}
		}else if (mode == 1) {
			for (i=0;i<2048; i++) {
				Backward(coefficient*T1ms);   // output every 10ms
			}
			for (i=0;i<1024; i++) {
				Right(coefficient*T1ms);   // output every 10ms
			}
			for (i=0;i<2048*2; i++) {
				Forward(coefficient*T1ms);   // output every 10ms
			}
		}
	}
}
// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortF_Init(void){volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R &= ~0x1F;      // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;         // PF3,PF2,PF1 outputs and PF4,PF0 inputs
  GPIO_PORTF_AFSEL_R &= ~0x1F;      // no alternate function    
  GPIO_PORTF_DEN_R |= 0x1F;         // enable digital pins PF4-PF0        
}

// Subroutine to initialize port B pins for input and output
// PB4 is input from the avoidance sensor
// Inputs: None
// Outputs: None
// 
void PortB_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     	// B clock
  delay = SYSCTL_RCGC2_R;           	// delay   
  GPIO_PORTB_CR_R |= 0x10;          	// allow changes to PB4       
  GPIO_PORTB_AMSEL_R &= ~0x10;      	// disable analog function
  GPIO_PORTB_PCTL_R &= ~0x000F0000; 	// GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R &= ~0x10;        	// PB4 input   
  GPIO_PORTB_AFSEL_R &= ~0x00010000;	// no alternate function
  GPIO_PORTB_PUR_R &= ~0x00010000;  	// disable all pullup resistors        
  GPIO_PORTB_DEN_R |= 0x10;         	// enable digital pins PB4       
}

// global variable visible in Watch window of debugger
// increments at least once per button press
void ButtonInterrupt_Init(void){                          
  SYSCTL_RCGC2_R |= 0x00000020; 		// (a) activate clock for port F
  GPIO_PORTF_DIR_R &= ~0x11;    		// (c) make PF4,PF0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  		//     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     		//     enable digital I/O on PF4,0   
  GPIO_PORTF_PCTL_R &= ~0x000F000F; // configure PF4,0 as GPIO
  GPIO_PORTF_PUR_R |= 0x11;    		 	//     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     		// (d) PF4,0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    		//     PF4,0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    		//     PF4,0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      		// (e) clear flag4,0
  GPIO_PORTF_IM_R |= 0x11;      		// (f) arm interrupt on PF4,0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00800000; // (g) priority 4
  NVIC_EN0_R |= 0x40000000;      		// (h) enable interrupt 30 in NVIC
}

// global variable visible in Watch window of debugger
// increments at least once per button press
void SensorInterrupt_Init(void){                          
	SYSCTL_RCGC2_R |= 0x00000002; 		// (a) activate clock for port B
  GPIO_PORTB_DIR_R &= ~0x10;    		// (c) make PB4 in
  GPIO_PORTB_AFSEL_R &= ~0x10;  		// 	disable alt funct on PB4
  GPIO_PORTB_DEN_R |= 0x10;     		//  enable digital I/O on PB4   
  GPIO_PORTB_PCTL_R &= ~0x000F0000; //  configure PB4 as GPIO
  GPIO_PORTB_PUR_R &= ~0x10;     		//  disnable weak pull-up on PB4
  GPIO_PORTB_IS_R &= ~0x10;     		// (d) PB4 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x10;    			//     PB4 is both edges
  GPIO_PORTB_IEV_R &= ~0x10;    		//     PB4 both edges event
  GPIO_PORTB_ICR_R = 0x10;      		// (e) clear flag4
  GPIO_PORTB_IM_R |= 0x10;      		// (f) arm interrupt on PB4
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF1FFF)|0x0000A000; // (g) priority 5
  NVIC_EN0_R |= 0x00000002;      		// (h) enable interrupt 2 in NVIC
}

// Subroutine to wait 0.25 sec
// Inputs: None
// Outputs: None
// Notes: ...
// **************SysTick_Init*********************
// Initialize SysTick periodic interrupts
// Input: interrupt period
//        Units of period are 62.5ns (assuming 16 MHz clock)
//        Maximum is 2^24-1
//        Minimum is determined by length of ISR
// Output: none
//void SysTickI_Init(unsigned long period){
//  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
//  NVIC_ST_RELOAD_R = period-1;// reload value
//  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
//  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // priority 2
//                              // enable SysTick with core clock and interrupts
//  NVIC_ST_CTRL_R = 0x07;
//}

// Interrupt service routine for portF
// Executed when SW1 or SW2 is pushed
void GPIOPortF_Handler(void){
		if ((GPIO_PORTF_RIS_R&0x10)==0x10){
			GPIO_PORTF_ICR_R = 0x10;		// acknowledge flag4 of portF
			mode = 0; 
		}
		else if ((GPIO_PORTF_RIS_R&0x01)==0x01){
			GPIO_PORTF_ICR_R = 0x01;		// acknowledge flag1 of portF
			mode = 1;
		}
}
	
// Interrupt service routine for port B
// Executed when avoidance sensor signal is detected
void GPIOPortB_Handler(void){
	unsigned long volatile time;
  time = 727240*200/910;  // 0.01sec for debouncing
  while(time){
		time--;
  }
	
	GPIO_PORTB_ICR_R = 0x10;  	// acknowledge flag4 of portC
		if (SENSOR==0x10){
			object = 0;
		}
		if (SENSOR==0x00){
			object = 1;
		}
}

// Interrupt service routine
// Executed every 10ms(period)
//void SysTick_Handler(void){ 
//counter1++;
//}

// Move 1.8 degrees clockwise, delay is the time to wait after each step
void Stepper_CW(unsigned long delay){
  L = fsm[L].Next[clockwise]; 
  STEPPER1 = fsm[L].Out; 
	SysTick_Wait(delay);
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void Stepper_CCW(unsigned long delay){
  L = fsm[L].Next[counterclockwise]; 
  STEPPER1 = fsm[L].Out;
	SysTick_Wait(delay);
}
// Moving forward function, delay is wait after each step
void Forward(unsigned long delay)
{
  R = fsm[R].Next[clockwise];
	STEPPER2 = fsm[R].Out << 4;
	L = fsm[L].Next[counterclockwise];
	STEPPER1 = fsm[L].Out;
	SysTick_Wait(delay);
}

// Moving backward function, delay is wait after each step
void Backward(unsigned long delay)
{
  R = fsm[R].Next[counterclockwise];
	STEPPER2 = fsm[R].Out << 4;
	L = fsm[L].Next[clockwise];
	STEPPER1 = fsm[L].Out;
	SysTick_Wait(delay);
}

// Turning left function, delay is wait after each step
void Left(unsigned long delay)
{
  R = fsm[R].Next[clockwise]; 
	STEPPER2 = fsm[R].Out << 4;
	L = fsm[L].Next[clockwise];
	STEPPER1 = fsm[L].Out;
	SysTick_Wait(delay);
}

// Turning right function, delay is wait after each step
void Right(unsigned long delay)
{
  R = fsm[R].Next[counterclockwise]; 
	STEPPER2 = fsm[R].Out << 4;
	L = fsm[L].Next[counterclockwise];
	STEPPER1 = fsm[L].Out;
	SysTick_Wait(delay);
}

// Initialize Stepper 1 interface
void Stepper_Init1(void){unsigned long volatile delay;
  SYSCTL_RCGC2_R  |= 0x08; 						// 1) activate port D
	delay = SYSCTL_RCGC2_R;
	L = 0;
  
	GPIO_PORTD_CR_R |= 0x0F;          	// 2) allow changes to PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;     		// 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; 	// 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   				// 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;				// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  				// enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   				// 7) enable digital I/O on PD3-0 
}

// Initialize Stepper 2 interface
void Stepper_Init2(void){unsigned long volatile delay;
	SYSCTL_RCGC2_R  |= 0x04;						// 1) activate port C
	delay = SYSCTL_RCGC2_R;
	R = 0;
	GPIO_PORTC_CR_R |= 0xF0;						// 2) allow changes to PC7-4
  GPIO_PORTC_AMSEL_R &= ~0xF0;      	// 3) disable analog functionality on PC7-4
  GPIO_PORTC_PCTL_R &= ~0xFFFF0000;	 	// 4) GPIO configure PC7-4 as GPIO
  GPIO_PORTC_DIR_R |= 0xF0;   				// 5) make PC7-4 out
  GPIO_PORTC_AFSEL_R &= ~0xF0;				// 6) disable alt funct on PC7-4
  GPIO_PORTC_DR8R_R |= 0xF0;  				// enable 8 mA drive
  GPIO_PORTC_DEN_R |= 0xF0;   				// 7) enable digital I/O on PC7-4 
}

