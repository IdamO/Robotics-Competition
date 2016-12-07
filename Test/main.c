//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles


//------------------------------------------
// TivaWare Header Files
//------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

//----------------------------------------
// Macros
//----------------------------------------

#define LF					0x0A
#define CR					0x0D

// Ultrasonic Sensor
#define USENSOR_PORT_BASE 	GPIO_PORTB_BASE
#define USENSOR_VCC 		GPIO_PIN_0
#define USENSOR_TRIGGER 	GPIO_PIN_6
#define USENSOR_ECHO		GPIO_PIN_7

// IR Sensor
#define IR_ADC_BASE 		ADC0_BASE
#define IR_ADC_PERIPH		SYSCTL_PERIPH_ADC0
#define IR_ADC_IN			ADC_CTL_CH0
#define IR_PORT_BASE 		GPIO_PORTE_BASE
#define IR_INPUT			GPIO_PIN_3
#define IR_PERIPH			SYSCTL_PERIPH_GPIOE

//Motor Controller Unit Right
#define R_MOTOR_PERIPH		SYSCTL_PERIPH_GPIOC
#define R_MOTOR_PORT_BASE	GPIO_PORTC_BASE
#define R_MOTOR_STEP		GPIO_PIN_4
#define	R_MOTOR_ENABLE		GPIO_PIN_5
#define	R_MOTOR_M0			GPIO_PIN_6

//Motor Controller Unit Left
#define L_MOTOR_PERIPH		SYSCTL_PERIPH_GPIOD
#define L_MOTOR_PORT_BASE	GPIO_PORTD_BASE
#define L_MOTOR_STEP		GPIO_PIN_0
#define	L_MOTOR_DIR			GPIO_PIN_1
#define	L_MOTOR_SLEEP		GPIO_PIN_2

//LED Display
#define LED_MATRIX_PERIPH	SYSCTL_PERIPH_GPIOA
#define LED_MATRIX_BASE		GPIO_PORTA_BASE
#define LED_MATRIX_DIN		GPIO_PIN_2
#define	LED_MATRIX_CIN		GPIO_PIN_3

//----------------------------------------
// Prototypes
//----------------------------------------
void hardware_init(void);
void command_line(void);
void UARTNextLine(void);
void delay(uint32_t time);
void motor_moveForward(uint32_t steps);
void motor_rotateRight_90(void);
void step_off_R(void);
void motor_rotateLeft_90(void);
void ir_sensor(void);
void move_forward_1ft(void);
void navigate_snake(void);
int avoid_obstacle(int currentGrid);
double uSensor(void);
void map_tunnels(void);
void timer_wait(int microseconds);
void led_control(void);

//---------------------------------------
// Globals
//---------------------------------------
volatile int16_t i16ToggleCount = 0;
int game_field[49];
int tunnel_map[49];
int grid_orientation = 0;


//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{

   hardware_init();							// init hardware via Xware
//   while (1) {
//   	   command_line();
//   }
   BIOS_start();

}

//---------------------------------------------------------------------------
// hardware_init()
//
// inits GPIO pins for toggling the LED
//---------------------------------------------------------------------------
void hardware_init(void)
{
	uint32_t ui32Period;

	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// ADD Tiva-C GPIO setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// enables port, sets trigger for output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(USENSOR_PORT_BASE, USENSOR_TRIGGER);
	// enables port, sets echo for input
	GPIOPinTypeGPIOInput(USENSOR_PORT_BASE, USENSOR_ECHO);

	// Turn on the Sensor
	//GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_VCC, 0);
	GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_TRIGGER, 1);

	// Timer 2 setup code
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);			// enable Timer 2 periph clks
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);		// cfg Timer 2 mode - periodic

	ui32Period = (SysCtlClockGet()/2);				// period = CPU clk div 2 (500ms)
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);			// set Timer 2 period

	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);		// enables Timer 2 to interrupt CPU

	TimerEnable(TIMER2_BASE, TIMER_A);						// enable Timer 2

	//////////////////////////UART//////////////////////////////////////////////
	//
	// Enable the peripherals used by this example.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
	                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                         UART_CONFIG_PAR_NONE));
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////MOTOR CONTROLLER////////////////////////////////////
	SysCtlPeripheralEnable(R_MOTOR_PERIPH);
	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_STEP);
//	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_M0);
//	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_ENABLE);
//	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_M1,0x00);
//	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_M0,0xFF);

	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_SLEEP);
	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_DIR);

	SysCtlPeripheralEnable(L_MOTOR_PERIPH);
	GPIOPinTypeGPIOOutput(L_MOTOR_PORT_BASE,L_MOTOR_STEP);
	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_SLEEP);
	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_DIR);
	/////////////////////////////////////////////////////////////////////////////

	////////////////////////IR SENSOR////////////////////////////////////////////
	SysCtlPeripheralEnable(IR_ADC_PERIPH);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(IR_PORT_BASE,IR_INPUT);
	ADCSequenceConfigure(IR_ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(IR_ADC_BASE, 1, 0, IR_ADC_IN);
	ADCSequenceStepConfigure(IR_ADC_BASE, 1, 1, IR_ADC_IN);
	ADCSequenceStepConfigure(IR_ADC_BASE, 1, 2, IR_ADC_IN);
	ADCSequenceStepConfigure(IR_ADC_BASE, 1, 3, IR_ADC_IN |ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(IR_ADC_BASE,1);

	///////////////////////LED DISPLAY///////////////////////////////////////////
	SysCtlPeripheralEnable(LED_MATRIX_PERIPH);
	GPIOPinTypeGPIOOutput(LED_MATRIX_BASE,LED_MATRIX_DIN);
	GPIOPinTypeGPIOOutput(LED_MATRIX_BASE,LED_MATRIX_CIN);

}


//---------------------------------------------------------------------------
// uSensor()
//
// toggles LED on Tiva-C LaunchPad
//---------------------------------------------------------------------------
double uSensor(void)
{
	uint32_t timeCount;
	double distance,testDistance;

	GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_TRIGGER, 0xFF);
	// wait 10us until disabling trigger
	delay(133333);
	uint32_t x = GPIOPinRead(USENSOR_PORT_BASE, USENSOR_TRIGGER);
	GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_TRIGGER, 0);
	// enable echo port to capture rising edge
	while (GPIOPinRead(USENSOR_PORT_BASE, USENSOR_ECHO) == 0) {}
	// start timer
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);			// must clear timer flag FROM timer
	timeCount = TimerValueGet(TIMER2_BASE, TIMER_A);
	while (GPIOPinRead(USENSOR_PORT_BASE, USENSOR_ECHO)!=0) {}
	timeCount = timeCount - TimerValueGet(TIMER2_BASE, TIMER_A);
	// capture falling edge

	// read time of counter = ultrasonic running time

	// distance = (time * ultrasonic spreading velocity in air)/2
	distance = timeCount * 340.0 / 100 / 2 / 4000;
	testDistance = distance*100;
	Log_info1("Distance is [%d] cm",testDistance);		// send toggle count to UIA
	return testDistance;
}


//---------------------------------------------------------------------------
// delay(uint32_t time)
//
// Creates a 500ms delay via TivaWare fxn
//---------------------------------------------------------------------------
void delay(uint32_t time)
{
	//500 ms = 6700000
	//1 ms = 13400
	//1 us = 14
	SysCtlDelay(time);		// creates ~500ms delay - TivaWare fxn

}

//---------------------------------------------------------------------------
// command_line()
//
// takes inputs from serial and runs corresponding functions
//---------------------------------------------------------------------------
void command_line()
{
	char command[2];

	command[0] = UARTCharGet(UART0_BASE);
	UARTCharPut(UART0_BASE,command[0]);
	command[1] = UARTCharGet(UART0_BASE);
	UARTCharPut(UART0_BASE,command[1]);
	UARTNextLine();


	if (command[0] == 'M' && command[1]=='F') {

		UARTCharPut(UART0_BASE,'Y');
		UARTNextLine();
		move_forward_1ft();
	}
	else if (command[0] == 'R' && command[1] == 'R') {
		//Log_info1("Hello test");
		UARTCharPut(UART0_BASE,'R');
		UARTNextLine();
		motor_rotateRight_90();
	}
	else if (command[0] == 'R' && command[1] == 'L') {
//		UARTCharPut(UART0_BASE,'L');
//		UARTNextLine();
		motor_rotateLeft_90();
	}
	else if (command[0] == 'D' && command[1] == 'C') {
		UARTCharPut(UART0_BASE,'U');
		UARTNextLine();
		int i = 0;
		for (i = 0; i<10; i++){
			uSensor();
		}
		UARTCharPut(UART0_BASE,'!');
		UARTNextLine();
	}
	else if (command[0] == 'I' && command[1] == 'R') {
			UARTCharPut(UART0_BASE,'I');
			UARTNextLine();
			int i = 0;
			for (i = 0; i<10; i++){
				ir_sensor();
			}
			UARTCharPut(UART0_BASE,'!');
			UARTNextLine();
	}
	else if (command[0] == 'O' && command[1] == 'N') {
			UARTCharPut(UART0_BASE,'O');
			UARTNextLine();
			led_control();
			UARTCharPut(UART0_BASE,'!');
			UARTNextLine();
	}
	else {
		//Log_info1("Test failed");
		UARTCharPut(UART0_BASE,'F');
		UARTNextLine();
	}
}

//---------------------------------------------------------------------------
// UARTNextLine()
//
// insert a new line in serial communication
//---------------------------------------------------------------------------
void UARTNextLine()
{
	UARTCharPut(UART0_BASE,LF);
	UARTCharPut(UART0_BASE,CR);
}


//---------------------------------------------------------------------------
// motor_moveForward()
//
// control of stepper motors
//---------------------------------------------------------------------------
void motor_moveForward(uint32_t steps)
{
	uint32_t currStep;
	for(currStep=0;currStep<steps;currStep++){
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0xFF);
		delay(295);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
		delay(13400);
	}
}

//---------------------------------------------------------------------------
// motor_rotateRight_90()
//
// rotate the robot to the right
//---------------------------------------------------------------------------
void motor_rotateRight_90()
{
	uint32_t steps;
	for(steps=0;steps<200;steps++){
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0xFF);
		delay(295);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
		delay(13400);
	}
}

//---------------------------------------------------------------------------
// motor_rotateLeft_90()
//
// rotate the robot to the left
//---------------------------------------------------------------------------
void motor_rotateLeft_90()
{
//	while (1){
//		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
//		delay(13400);
//		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
//		delay(13400);
//	}
	uint32_t steps;

	for(steps=0;steps<200;steps++){
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
		delay(295);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0);
		delay(13400);
	}
}

//---------------------------------------------------------------------------
// ir_sensor()
//
// poll data on ir sensor
//---------------------------------------------------------------------------
void ir_sensor()
{
	uint32_t sequenceData[4];
	uint32_t irData;
	ADCIntClear(IR_ADC_BASE, 1);
	ADCProcessorTrigger(IR_ADC_BASE,1);
	while(!ADCIntStatus(IR_ADC_BASE,1,false)){}
	ADCSequenceDataGet(IR_ADC_BASE,1,sequenceData);
	irData = (sequenceData[0]+sequenceData[1]+sequenceData[2]+sequenceData[3]+2)/4;
	Log_info1("Data is [%d]",irData);

}

//---------------------------------------------------------------------------
void move_forward_1ft()
{
	uint32_t steps;
	steps = 647;
	motor_moveForward(steps);
}

//---------------------------------------------------------------------------
void navigate_snake()
{
	int grid;
	for(grid=0;grid<49;grid++){
		move_forward_1ft();
		grid++;
		game_field[grid]=1;
		if (grid%7==0){
			if (grid_orientation==0){
				motor_rotateRight_90();
				move_forward_1ft();
				grid++;
				game_field[grid]=1;
				motor_rotateRight_90();
				grid_orientation=1;
			}
			else {
				motor_rotateLeft_90();
				move_forward_1ft();
				grid++;
				game_field[grid]=1;
				motor_rotateLeft_90();
				grid_orientation=0;
			}
		}
	}
}

//-------------------------------------------------------------------------
int avoid_obstacle(int currentGrid)
{
	int stair = 0;
	int grid = currentGrid;
	double dist_to_obst;
	dist_to_obst = uSensor();

	motor_rotateRight_90();
	move_forward_1ft();
	motor_rotateLeft_90();
	move_forward_1ft();
	dist_to_obst = uSensor();
	if (dist_to_obst<=6.5){
		grid = avoid_obstacle(grid);
		stair = 1;
	}
	move_forward_1ft();
	motor_rotateLeft_90();
	move_forward_1ft();
	motor_rotateRight_90();
	if(stair==1){
		motor_rotateLeft_90();
		move_forward_1ft();
		motor_rotateLeft_90();
		move_forward_1ft();
		motor_rotateRight_90();
		motor_rotateRight_90();
	}

	return grid;
}

//-------------------------------------------------------------------
void map_tunnels()
{
	int endpoint1 = -1, endpoint2 = -1;
	int i;
	for(i=0;i<7;i++){
		if (game_field[i]==1){
			if (endpoint1==-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
		}
	}
	for(i=42;i<49;i++){
		if (game_field[i]==1){
			if (endpoint1==-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
			break;
		}
	}
	for(i=7;i<42;i+7){
		if (game_field[i]==1){
			if (endpoint1==-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
			break;
		}
	}
	for(i=13;i<48;i+7){
		if (game_field[i]==1){
			if (endpoint1==-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
			break;
		}
	}
	int multpath = 0;
	int currentGrid = endpoint1;
	int paths;
	int noDeadEnd;
	while(!multpath){
		paths = 0;
		if (game_field[currentGrid+1]==1 && currentGrid%7!=6){
			paths++;
			currentGrid = currentGrid+1;
		}
		if (game_field[currentGrid-1]==1 && currentGrid%7!=0 && currentGrid!=0){
			paths++;
			currentGrid = currentGrid-1;
		}
		if (game_field[currentGrid+7]==1 && currentGrid<42){
			paths++;
			currentGrid = currentGrid+7;
		}
		if (game_field[currentGrid+7]==1 && currentGrid>6){
			paths++;
			currentGrid = currentGrid-7;
		}
		if (paths>1){
			multpath = 1;
		}
		else if (paths==0){
			noDeadEnd = 1;
			break;
		}
		else {
			game_field[currentGrid]=2;
		}
	}
	if (!noDeadEnd){
		multpath = 0;
		while (!multpath){
			paths = 0;
			if (game_field[currentGrid+1]==1 && currentGrid%7!=6){
				paths++;
				currentGrid = currentGrid+1;
			}
			if (game_field[currentGrid-1]==1 && currentGrid%7!=0 && currentGrid!=0){
				paths++;
				currentGrid = currentGrid-1;
			}
			if (game_field[currentGrid+7]==1 && currentGrid<42){
				paths++;
				currentGrid = currentGrid+7;
			}
			if (game_field[currentGrid+7]==1 && currentGrid>6){
				paths++;
				currentGrid = currentGrid-7;
			}
			if (paths>1){
				multpath = 1;
				game_field[currentGrid]=2;
			}
			else if (paths==0){
				noDeadEnd = 1;
				break;
			}
			else {
				game_field[currentGrid]=2;
			}
		}
	}

}

// ------------------------------------------------
// led_control()

void led_control()
{
	GPIOPinWrite(LED_MATRIX_BASE, LED_MATRIX_DIN, 0x01);
	GPIOPinWrite(LED_MATRIX_BASE, LED_MATRIX_CIN, 0x00);
	GPIOPinWrite(LED_MATRIX_BASE, LED_MATRIX_DIN, 0xFF);
	GPIOPinWrite(LED_MATRIX_BASE, LED_MATRIX_CIN, 0x00);
}






