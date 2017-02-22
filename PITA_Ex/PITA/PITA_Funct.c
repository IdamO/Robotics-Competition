#include <xdc/runtime/Log.h>				//needed for any Log_info() call
//------------------------------------------
// TivaWare Header Files
//------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "PITA_Funct.h"
#include "utils/uartstdio.c"

//----------------------------------------
// Macros
//----------------------------------------

#define LF					0x0A
#define CR					0x0D

// Ultrasonic Sensor
#define USENSOR_PORT_BASE 	GPIO_PORTD_BASE
#define USENSOR_VCC 		GPIO_PIN_0
#define USENSOR_TRIGGER 	GPIO_PIN_6
#define USENSOR_ECHO		GPIO_PIN_7
#define USENSOR_TRIGGER2	GPIO_PIN_0
#define USENSOR_ECHO2		GPIO_PIN_1

// IR Sensor
#define IR_ADC_BASE 		ADC0_BASE
#define IR_ADC_PERIPH		SYSCTL_PERIPH_ADC0
#define IR_ADC_IN			ADC_CTL_CH0
#define IR_PORT_BASE 		GPIO_PORTE_BASE
#define IR_INPUT			GPIO_PIN_3
#define IR_PERIPH			SYSCTL_PERIPH_GPIOE

// Capacitive Sensor
#define CAP_ADC_BASE		ADC1_BASE
#define CAP_ADC_PERIPH		SYSCTL_PERIPH_ADC1
#define CAP_ADC_IN			ADC_CTL_CH1
#define CAP_BASE			GPIO_PORTE_BASE
#define CAP_INPUT			GPIO_PIN_2
#define CAP_PERIPH			SYSCTL_PERIPH_GPIOE
#define CAP_VCC				GPIO_PIN_1

//Motor Controller Unit Right
#define R_MOTOR_PERIPH		SYSCTL_PERIPH_GPIOC
#define R_MOTOR_PORT_BASE	GPIO_PORTC_BASE
#define R_MOTOR_STEP		GPIO_PIN_7
#define	R_MOTOR_DIR			GPIO_PIN_6

//Motor Controller Unit Left
#define L_MOTOR_PERIPH		SYSCTL_PERIPH_GPIOC
#define L_MOTOR_PORT_BASE	GPIO_PORTC_BASE
#define L_MOTOR_STEP		GPIO_PIN_5
#define	L_MOTOR_DIR			GPIO_PIN_4
#define	L_MOTOR_SLEEP		GPIO_PIN_2

//Robot Arm
#define ARM_PERIPH			SYSCTL_PERIPH_GPIOB
#define ARM_BASE			GPIO_PORTB_BASE
#define ARM_ROTATE			GPIO_PIN_6
#define ARM_LOWER			GPIO_PIN_7
#define ARM_GRIP			GPIO_PIN_4


//LED Display
#define LED_MATRIX_PERIPH	SYSCTL_PERIPH_GPIOA
#define LED_MATRIX_BASE		GPIO_PORTA_BASE
#define LED_MATRIX_DIN		GPIO_PIN_2
#define	LED_MATRIX_CIN		GPIO_PIN_3
#define LED_RED				0xD102000F
#define LED_BLUE			0xD10FF000
#define LED_YELLOW			0xD1020F0F
#define LED_OFF				0xD1000000

//Raspberry Pi Comm
#define PI_COMM_PERIPH		SYSCTL_PERIPH_GPIOA
#define PI_COMM_BASE		GPIO_PORTA_BASE
#define PI_COMM_DETECT_OUT	GPIO_PIN_5
#define PI_COMM_DETECT_IN	GPIO_PIN_6
#define PI_COMM_DEGREE		GPIO_PIN_7


//---------------------------------------
// Globals
//---------------------------------------
volatile int16_t i16ToggleCount = 0;
int game_field[49] ={0,0,0,0,0,1,0,
					 0,0,1,1,0,1,0,
					 0,0,0,1,1,1,0,
					 1,1,0,0,0,1,0,
					 0,1,1,1,1,1,0,
					 0,0,0,0,0,0,0,
					 0,0,0,0,0,0,0};

int tunnel_map[49] = 	{0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0};
int grid_orientation = 0;
uint32_t ui32PWMClock;
uint32_t ui32Load;

//---------------------------------------------------------------------------
// hardware_init()
//
// inits hardware being used
//---------------------------------------------------------------------------
void hardware_init(void)
{
	uint32_t ui32Period;

	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

	// ADD Tiva-C GPIO setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

	//Permanent high
//	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_2);
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2,0xFF);

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
	UARTStdioConfig(0,9600,SysCtlClockGet());

	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////Ultrasonic Sensors//////////////////////////////////
	// enables port, sets trigger for output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(USENSOR_PORT_BASE, USENSOR_TRIGGER);
	GPIOPinTypeGPIOOutput(USENSOR_PORT_BASE, USENSOR_TRIGGER2);
	// enables port, sets echo for input
	GPIOPinTypeGPIOInput(USENSOR_PORT_BASE, USENSOR_ECHO);
	GPIOPinTypeGPIOInput(USENSOR_PORT_BASE, USENSOR_ECHO2);
	// Turn on the Sensor
	//GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_VCC, 0);
	//GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_TRIGGER, 1)
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////MOTOR CONTROLLER////////////////////////////////////
	SysCtlPeripheralEnable(R_MOTOR_PERIPH);
	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_STEP);
//	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_M0);
//	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_ENABLE);
//	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_M1,0x00);
//	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_M0,0xFF);

	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_SLEEP);
	GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_DIR);

	//SysCtlPeripheralEnable(L_MOTOR_PERIPH);
	GPIOPinTypeGPIOOutput(L_MOTOR_PORT_BASE,L_MOTOR_STEP);
	//GPIOPinTypeGPIOOutput(R_MOTOR_PORT_BASE,R_MOTOR_SLEEP);
	GPIOPinTypeGPIOOutput(L_MOTOR_PORT_BASE,L_MOTOR_DIR);
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

	////////////////////////CAPACITIVE SENSOR////////////////////////////////////
	SysCtlPeripheralEnable(CAP_ADC_PERIPH);
	GPIOPinTypeADC(CAP_BASE,CAP_INPUT);
	//GPIOPinConfigure(GPIO_PE2_AIN1);
	ADCSequenceConfigure(CAP_ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(CAP_ADC_BASE, 1, 0, CAP_ADC_IN);
	ADCSequenceStepConfigure(CAP_ADC_BASE, 1, 1, CAP_ADC_IN);
	ADCSequenceStepConfigure(CAP_ADC_BASE, 1, 2, CAP_ADC_IN);
	ADCSequenceStepConfigure(CAP_ADC_BASE, 1, 3, CAP_ADC_IN |ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(CAP_ADC_BASE,1);
	GPIOPinTypeGPIOOutput(CAP_BASE,CAP_VCC);
	GPIOPinWrite(CAP_BASE,CAP_VCC,0xFF);


	///////////////////////LED DISPLAY///////////////////////////////////////////
	SysCtlPeripheralEnable(LED_MATRIX_PERIPH);
	GPIOPinTypeGPIOOutput(LED_MATRIX_BASE,LED_MATRIX_DIN);
	GPIOPinTypeGPIOOutput(LED_MATRIX_BASE,LED_MATRIX_CIN);

	///////////////////////RASPBERRY PI COMM/////////////////////////////////////
	GPIOPinTypeGPIOOutput(PI_COMM_BASE,PI_COMM_DETECT_OUT);
	GPIOPinTypeGPIOInput(PI_COMM_BASE,PI_COMM_DETECT_IN | PI_COMM_DEGREE);
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x00);

	//////////////////////ARM////////////////////////////////////////////////////
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(ARM_PERIPH);

	GPIOPinTypePWM(ARM_BASE,ARM_ROTATE|ARM_LOWER|ARM_GRIP);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN);
	ui32PWMClock = SysCtlPWMClockGet()/64;
	UARTprintf("%d\n",(int)ui32PWMClock);
	ui32Load = 50000;
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (10/100)*ui32Load);
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE,PWM_GEN_0);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (10/100)*ui32Load);
	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT, true);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (10/100)*ui32Load);
	PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE,PWM_GEN_1);

	UARTprintf("\n***Hardware setup complete.\n");
}


//---------------------------------------------------------------------------
// uSensor()
//
// Determine distance between ultrasnoic sensor and nearest obstacle (up to 4m)
//---------------------------------------------------------------------------
double uSensor(void)
{
	uint32_t timeCount;
	double distance,testDistance;
	UARTprintf("Collecting ultrasonic sensor data.\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	GPIOPinWrite(USENSOR_PORT_BASE, USENSOR_TRIGGER, 0xFF);
	// wait 10us until disabling trigger
	delay(134);
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
	UARTprintf("Distance is %d cm divided by 100.\n", testDistance);
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
	char command[3];

	UARTprintf("\nEnter command: ");
	UARTgets(command,3);

	if (strcmp(command,"MF")==0) {

		UARTprintf("\nMove forward command entered.\n");
		move_forward_1ft();
		UARTprintf("Move forward command completed.\n");
	}
	if (strcmp(command,"NS")==0) {
		UARTprintf("\nMove forward command entered.\n");
		navigate_snake();
		UARTprintf("Move forward command completed.\n");
	}
	else if (command[0] == 'R' && command[1] == 'R') {
		//Log_info1("Hello test");
		UARTprintf("\nRotate right command entered.\n");
		UARTNextLine();
		motor_rotateRight_90();
		UARTprintf("Rotate right command completed.\n");
	}
	else if (command[0] == 'R' && command[1] == 'L') {
		UARTprintf("\nRotate left command entered.\n");
		motor_rotateLeft_90();
		UARTprintf("Rotate left command completed.\n");
	}
	else if (command[0] == 'U' && command[1] == 'C') {
		UARTprintf("\nUltrasonic sensor command entered.\n");
		int i = 0;
//		for (i = 0; i<10; i++){
			uSensor();
			delay(13400);
//		}
		UARTprintf("Ultrasonic sensor command completed.\n");
	}
	else if (command[0] == 'I' && command[1] == 'R') {
		UARTprintf("\nIR sensor command entered.\n");
		ir_sensor();
		UARTprintf("IR sensor command completed.\n");
	}
	else if (command[0] == 'C' && command[1] == 'S') {
		UARTprintf("\nCapacitive sensor command entered.\n");
		int i = 0;
		for (i=0;i<5;i++){
			capacitive_sensor();
		}
		UARTprintf("Capacitive sensor command completed.\n");
	}
	else if (command[0] == 'O' && command[1] == 'N') {
		UARTprintf("\nLED display command entered.\n");
		map_tunnels();
		led_control();
		UARTprintf("LED display command completed.\n");
	}
	else if (command[0] == 'A' && command[1] == 'R') {
		UARTprintf("\nArm rotate command entered.\n");
		rotate_arm(180);
		delay(6700000*2);
		rotate_arm(0);
		UARTprintf("Arm rotate command completed.\n");
	}
	else if (command[0] == 'A' && command[1] == 'G') {
		UARTprintf("\nArm grip command entered.\n");
		arm_grip(1);
		delay(6700000*2);
		arm_grip(0);
		delay(6700000*2);
		UARTprintf("Arm grip command entered.\n");
	}
	else if (command[0] == 'A' && command[1] == 'C') {
		UARTprintf("\nArm control command entered.\n");
		move_arm(1);
		delay(6700000*2);
		move_arm(0);
		UARTprintf("Arm control command completed.\n");
	}
	else if (command[0] == 'G' && command[1] == 'A') {
		UARTprintf("Getting Data from Pi.\n");

		UARTprintf("Communication complete.\n");
	}
	else {
		UARTprintf("\nNot a valid command.\n");
		UARTprintf("Valid commands are: \nMF\nRR\nRL\nUC\nIR\nCS\nON\nAR\nAG\nAC\n");
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
// motor_move()
//
// rotate the stepper motors a certain amount of steps
//---------------------------------------------------------------------------
void motor_move(uint32_t steps,char continuous)
{
	uint32_t currStep;
	int rampDelay=1400;
	char alternate = 1;
	UARTprintf("Motors rotating %d steps.\n",steps);
	GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0xFF);
	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0xFF);
	for(currStep=0;currStep<steps;currStep++){
		rampDelay = motor_ramp(currStep,steps,7000,0,rampDelay);
//		if (currStep==steps-1){
//			alternate = 0;
//		}
		if (alternate==1){
			GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0xFF);
			GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
		}
		else {
			GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
			GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0xFF);
		}
		delay(295);
		if (alternate==1){
			GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
			GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
			alternate = 0;
		}
		else {
			GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
			GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
			alternate = 1;
		}
		delay(rampDelay);
	}
	UARTprintf("Steps completed.\n");
}

//---------------------------------------------------------------------------
// motor_rotateRight_90()
//
// rotate the robot to the right 90 degrees
//---------------------------------------------------------------------------
void motor_rotateRight_90()
{
	uint32_t steps;
	int maxSteps = 1400;
	int rampDelay;
	UARTprintf("Rotating to the right 90 degrees.\n");
	GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0x00);
	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0xFF);
	for(steps=0;steps<maxSteps;steps++){
		rampDelay = motor_ramp(steps,maxSteps,7000,0,rampDelay);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0xFF);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
		delay(295);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
		delay(rampDelay);
	}
	UARTprintf("Rotation complete.\n");
}

//---------------------------------------------------------------------------
// motor_rotateLeft_90()
//
// rotate the robot to the left
//---------------------------------------------------------------------------
void motor_rotateLeft_90()
{
	uint32_t steps;
	int maxSteps = 1400;
	int rampDelay;
	UARTprintf("Rotating to the left 90 degrees.\n");
	GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0xFF);
	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0x00);
	for(steps=0;steps<maxSteps;steps++){
		rampDelay = motor_ramp(steps, maxSteps, 7000,0,rampDelay);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0xFF);
		GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_STEP,0xFF);
		delay(295);
		GPIOPinWrite(R_MOTOR_PORT_BASE, R_MOTOR_STEP, 0x00);
		GPIOPinWrite(L_MOTOR_PORT_BASE, L_MOTOR_STEP, 0x00);
		delay(rampDelay);
	}
	UARTprintf("Rotation complete.\n");
}

int motor_ramp(int currStep, int maxSteps, int finalDelay, char continuous, int currDelay)
{
	int rampDelay;
	int risingEdge = maxSteps / 5;
	int fallingEdge = maxSteps / 2;
	int maxDelay = finalDelay*3;
	int incrementRise = (maxDelay-finalDelay)/risingEdge;
	int incrementFall = (maxDelay-finalDelay)/(maxSteps-fallingEdge);

	if (currStep==0){
		rampDelay = maxDelay;
	}
	else if (currStep < risingEdge){
		rampDelay = currDelay - incrementRise;
	}
	else if (risingEdge <= currStep && currStep < fallingEdge){
		rampDelay = finalDelay;
	}
	else if (continuous==0){
		rampDelay = currDelay + incrementFall;
	}

	return rampDelay;
}

//---------------------------------------------------------------------------
// ir_sensor()
//
// poll data on ir sensor
//---------------------------------------------------------------------------
uint32_t ir_sensor()
{
	uint32_t sequenceData[4];
	uint32_t irData;
	UARTprintf("Collecting IR data.\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	ADCIntClear(IR_ADC_BASE, 1);
	ADCProcessorTrigger(IR_ADC_BASE,1);
	while(!ADCIntStatus(IR_ADC_BASE,1,false)){}
	ADCSequenceDataGet(IR_ADC_BASE,1,sequenceData);
	irData = (sequenceData[0]+sequenceData[1]+sequenceData[2]+sequenceData[3]+2)/4;
	if (irData > 700){
		// Turn on the LED
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
	}
	Log_info1("Data is [%d]",irData);
	UARTprintf("Data is %d.\n",irData);
	return irData;

}

//---------------------------------------------------------------------------
void move_forward_1ft()
{
	uint32_t steps;
	steps = 2588;
	GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0x00);
	GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0xFF);
	UARTprintf("Moving forward 1 ft.\n");
	motor_move(steps,0);
	UARTprintf("Movement complete.\n");
}

//---------------------------------------------------------------------------
void move(int inches,char dir,char continuous){
	uint32_t steps = 2588 / 12 * inches;
	if (dir){
		UARTprintf("Moving forward %d inches.\n",inches);
		GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0x00);
		GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0xFF);
		motor_move(steps,continuous);
	}
	else {
		UARTprintf("Moving backward %d inches.\n",inches);
		GPIOPinWrite(L_MOTOR_PORT_BASE,L_MOTOR_DIR,0xFF);
		GPIOPinWrite(R_MOTOR_PORT_BASE,R_MOTOR_DIR,0x00);
		motor_move(steps,continuous);
	}
	UARTprintf("Movement completed.\n");
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
	UARTprintf("Running algorithm to map tunnels.\n");
	int endpoint1 = -1, endpoint2 = -1;
	int i;
	for(i=0;i<7;i++){
		if (game_field[i]==1){
			if (endpoint1!=-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
		}
	}
	for(i=42;i<49;i++){
		if (game_field[i]==1){
			if (endpoint1!=-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
			break;
		}
	}
	for(i=7;i<42;i=i+7){
		if (game_field[i]==1){
			if (endpoint1!=-1){
				endpoint2=i;
			}
			else {
				endpoint1=i;
			}
			break;
		}
	}
	for(i=13;i<48;i=i+7){
		if (game_field[i]==1){
			if (endpoint1!=-1){
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
	int paths=0;
	int noDeadEnd = 0;
	int nextGrid = 0;
	while(!multpath){
		if (game_field[currentGrid+1]==1 && currentGrid%7!=6){
			paths++;
			nextGrid = currentGrid+1;
		}
		if (game_field[currentGrid-1]==1 && currentGrid%7!=0 && currentGrid!=0){
			paths++;
			nextGrid = currentGrid-1;
		}
		if (game_field[currentGrid+7]==1 && currentGrid<42){
			paths++;
			nextGrid = currentGrid+7;
		}
		if (game_field[currentGrid-7]==1 && currentGrid>6){
			paths++;
			nextGrid = currentGrid-7;
		}
		if (paths>1){
			multpath = 1;
			game_field[currentGrid]=2;
		}
		else if (paths==0){
			noDeadEnd = 1;
			game_field[currentGrid]=2;
			break;
		}
		else {
			game_field[currentGrid]=2;
			currentGrid = nextGrid;
			paths = 0;
		}
	}
	if (!noDeadEnd){
		multpath = 0;
		currentGrid = endpoint2;
		paths=0;
		while (!multpath){
			if (game_field[currentGrid+1]==1 && currentGrid%7!=6){
				paths++;
				nextGrid = currentGrid+1;
			}
			if (game_field[currentGrid-1]==1 && currentGrid%7!=0 && currentGrid!=0){
				paths++;
				nextGrid = currentGrid-1;
			}
			if (game_field[currentGrid+7]==1 && currentGrid<42){
				paths++;
				nextGrid = currentGrid+7;
			}
			if (game_field[currentGrid-7]==1 && currentGrid>6){
				paths++;
				nextGrid = currentGrid-7;
			}
			if (paths>1){
				multpath = 1;
				game_field[currentGrid]=2;
			}
			else if (paths==0){
				noDeadEnd = 1;
				game_field[currentGrid]=2;
				break;
			}
			else {
				game_field[currentGrid]=2;
				currentGrid = nextGrid;
				paths = 0;
			}
		}

	}
	UARTprintf("Tunnel mapping algorithm completed.\n");

}

// ------------------------------------------------
// led_control()

void led_control()
{
	UARTprintf("Updating and turning on LEDs on matrix.\n");
	int LED_count = 49;
	int bit_count;
	int LED_frames;
	char active_LED = 0;
	int num_disabled = 8;
	int disabledLEDs[]={0,15,16,31,32,47,48,63};
	int index = 0;
	char snake_dir = 0;
	char edge = 0;
	int grid = 0;
	char grid_inbounds = 1;
	// brightness - 12
	// BGR - 255,0,0
	uint32_t LED_bitstream = 0xD1FF0000;
	// Send the start frame
	for (bit_count = 0; bit_count < 32; bit_count++){
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,0x00);
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0xFF);
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0x00);
	}
	// Send the LED frames
	for (LED_frames = 0; LED_frames < 64; LED_frames++){
		for (index=0;index<num_disabled;index++){
			if (LED_frames==disabledLEDs[index]){
				active_LED = 0;
			}
		}
		if (active_LED && grid_inbounds && LED_frames>7){
			if (grid==48){
				grid_inbounds = 0;
			}
			if (grid==42){
				LED_bitstream = LED_YELLOW;
			}
			else if (tunnel_map[grid]==1){
				LED_bitstream = LED_BLUE;
			}
			else if (tunnel_map[grid]==2){
				LED_bitstream = LED_RED;
			}
			else {
				LED_bitstream = LED_OFF;
			}

			if (grid==0){
				grid = grid+7;
			}
			else if (grid<7 || grid > 41){
				if (edge){
					edge=0;
					if (snake_dir)
						grid = grid-7;
					else
						grid = grid+7;
				}
				else{
					grid = grid+1;
					edge=1;
					if (snake_dir)
						snake_dir=0;
					else
						snake_dir=1;
				}
			}
			else if (snake_dir==0){
				grid = grid+7;
			}
			else if (snake_dir==1){
				grid = grid-7;
			}
		}
		for (bit_count = 0; bit_count <32; bit_count++){
			if (active_LED && (LED_frames>7)){
				if (bit_count < 24)	{
					GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,LED_bitstream >> (24-bit_count));
				}
				else {
					GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,LED_bitstream << (bit_count-24));
				}
			}
			else {
				if (bit_count < 25)	{
					GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,LED_OFF >> (24-bit_count));
				}
				else {
					GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,LED_OFF << (bit_count-24));
				}
			}
			GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0xFF);
			GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0x00);
		}
		active_LED = 1;
	}
	// Send the end frame
	for (bit_count = 0; bit_count < 32; bit_count++){
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_DIN,0x00);
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0xFF);
		GPIOPinWrite(LED_MATRIX_BASE,LED_MATRIX_CIN,0x00);
	}
	UARTprintf("LEDs have been updated and turned on.\n");
}

void rotate_arm(int degrees){
	UARTprintf("Rotating arm %d degrees\n", degrees);
	uint32_t time_for_delay = (600 + 10*degrees)*13.4;
	uint32_t duty = degrees/20 + 3;
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,(duty/100.0)*ui32Load);
	UARTprintf("Rotation complete.\n");
}
void arm_grip(char open){
	uint32_t open_time_for_delay = 1800 * 13.4;
	uint32_t close_time_for_delay = 2200 * 13.4;
	if (open==0) {
		UARTprintf("Opening arm grip.\n");
		PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,(8.0/100.0)*ui32Load);
		UARTprintf("Arm opened.\n");
	}
	else {
		UARTprintf("Closing arm grip.\n");
		PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,(10.0/100.0)*ui32Load);
		UARTprintf("Arm closed.\n");
	}
}

void move_arm(char dir){
	double time_sec;
	uint32_t time_for_delay;
	int increment = 0;
	if (dir==1){
		time_sec = 1.75;
		time_for_delay = time_sec * 13400 * 100;
		UARTprintf("Lowering arm.\n");
		PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT, true);
		PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,(10.0/100.0)*ui32Load);
		delay(6700000*4);
		PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT, false);
		UARTprintf("Arm lowered.\n");
	}
	else {
		time_sec = 1;
		time_for_delay = time_sec * 13400 * 100;
		UARTprintf("Raising arm.\n");
		PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT, true);
		PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,(5.0/100.0)*ui32Load);
		delay(6700000*4);
		PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT, false);
		UARTprintf("Arm raised.\n");
	}
}

uint32_t capacitive_sensor()
{
	uint32_t sequenceData[4];
	uint32_t capData;
	UARTprintf("Collecting capacitive sensor data.\n");
	//GPIOPinWrite(CAP_BASE,CAP_VCC,0xFF);
	//delay(1000);
	ADCIntClear(CAP_ADC_BASE, 1);
	ADCProcessorTrigger(CAP_ADC_BASE,1);
	while(!ADCIntStatus(CAP_ADC_BASE,1,false)){}
	ADCSequenceDataGet(CAP_ADC_BASE,1,sequenceData);
	capData = (sequenceData[0]+sequenceData[1]+sequenceData[2]+sequenceData[3])/4;
	Log_info1("Data is [%d]",capData);
	UARTprintf("Data is %d.\n",capData);
	//GPIOPinWrite(CAP_BASE,CAP_VCC,0x00);
	return capData;
}

int getData() {
	uint16_t data = 0b00000000;
	uint16_t temp;
	int bitcount = 0;
	for (bitcount=0;bitcount<9;bitcount++){
		while (GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN)==0){}
		temp = GPIOPinRead(PI_COMM_BASE,PI_COMM_DEGREE);
		temp = temp << (7-bitcount);
		data = data | temp;
		GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x01);
		while (GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN)==1){}
		GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x00);
	}
	return data;
}

int detect_line() {
	int angle;
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x10);
	while (GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN)==0){}
	angle = GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN);
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0xFF);
	return angle;
}

uint32_t locate_cache(){
	uint32_t distance;
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x20);
	while (GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN)==0){}
	distance = GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN);
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0xFF);
	return distance;
}

uint8_t find_pips(){
	uint8_t pips;
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0x30);
	while (GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN)==0){}
	pips = GPIOPinRead(PI_COMM_BASE,PI_COMM_DETECT_IN);
	GPIOPinWrite(PI_COMM_BASE,PI_COMM_DETECT_OUT,0xFF);
	return pips;
}




