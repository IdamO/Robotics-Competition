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
#include "PITA_Funct.h"

//----------------------------------------
// Macros
//----------------------------------------
//Start Button
#define START_BASE			GPIO_PORTD_BASE
#define START_PIN			GPIO_PIN_0

//----------------------------------------
// Prototypes
//----------------------------------------
void PITA_standby(void);
void game_nav(void);
char obstacle_check(int currentGrid);
char tunnel_check(int currentGrid);
char cache_check(void);
char die_check(void);
void return_to_start(void);
int grid_change(int grid,char grid_orientation);

//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{
   hardware_init();							// init hardware via Xware
   BIOS_start();

}

//---------------------------------------------------------------------------
// PITA_standby()
// wait for the start button to be pushed
// when start button is pushed trigger interrupt and begin movement
//---------------------------------------------------------------------------
void PITA_standby()
{
	char test_mode = 1;
	if (test_mode){
		while (1) {
		   command_line();
		}
	}
	else {
		// wait for the start button to be pushed
		while(GPIOPinRead(START_BASE,START_PIN)==0x00){
		}

		// post interrupt to begin game
		Swi_post(Field_Navigation);
	}
}

void game_nav()
{
	int grid = 0;
	char tunnel;
	while (grid!=6){
		tunnel = tunnel_check(grid);
		//if there is a tunnel and it is an edge of the field, look for cache
		if (tunnel){
			if (grid%7==0 || grid%7==6){
				cache_check();
			}
		}
		obstacle_check(grid);
		move_forward_1ft();
		grid = grid_change(grid,grid_orientation);
		game_field[grid]=1;
		// if the robot has reached the edge of the field
		if (grid<=6 || grid>=42){
			if (grid_orientation==0){
				tunnel = tunnel_check(grid);
				if (tunnel){
					cache_check();
				}
				motor_rotateRight_90();
				obstacle_check(grid);
				move_forward_1ft();
				grid = grid_change(grid,grid_orientation);
				game_field[grid]=1;
				motor_rotateRight_90();
				grid_orientation=1;
			}
			else {
				tunnel = tunnel_check(grid);
				if (tunnel){
					cache_check();
				}
				motor_rotateLeft_90();
				obstacle_check(grid);
				move_forward_1ft();
				grid = grid_change(grid,grid_orientation);
				game_field[grid]=1;
				motor_rotateLeft_90();
				grid_orientation=0;
			}
		}
	}
	return_to_start();
}

int grid_change(int grid,char grid_orientation)
{
	int new_grid = grid;
	if (grid<=6 || grid >= 42){
		new_grid++;
	}
	else if (grid_orientation==0) {
		new_grid = new_grid-7;
	}
	else if (grid_orientation==1) {
		new_grid = new_grid+7;
	}
	return new_grid;
}

void return_to_start()
{
	int grid = 6;
	motor_rotateLeft_90();
	while (grid != 42){
		if (grid==0){
			motor_rotateLeft_90();
		}
		else if (grid<7){
			obstacle_check(grid);
			move_forward_1ft();
			grid--;
		}
		else if (grid%7==0){
			obstacle_check(grid);
			move_forward_1ft();
			grid = grid+7;
		}
		else {
			obstacle_check(grid);
			move_forward_1ft();
			grid--;
			if (grid%7==0) {
				motor_rotateLeft_90();
			}
		}
	}

}

//---------------------------------------------------------------------------
// obstacle_check()
//
// poll data on ir sensor
//---------------------------------------------------------------------------
char obstacle_check(int currentGrid)
{
	uint32_t distData = ir_sensor();
	if (distData > 700){
		// Turn on the LED
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
		avoid_obstacle(currentGrid);
		return 1;
	}
	return 0;
}

//---------------------------------------------------------------------------
// tunnel_check()
//
// use appropriate sensor to detect if there is a tunnel at the location
//---------------------------------------------------------------------------
char tunnel_check(int currentGrid)
{
	return 0;
}

//---------------------------------------------------------------------------
// cache_check()
//
// call to raspberry pi to check for cache
//---------------------------------------------------------------------------
char cache_check()
{
	// locate cache
	// align gripper with cache
	// remove the lid
	if (die_check()){
		//call to pi to perform the die functions
		//pi will display on the 7-seg
	}
	return 0;
}

//---------------------------------------------------------------------------
// die_check()
//
// call to raspberry pi to check for die
//---------------------------------------------------------------------------
char die_check()
{
	return 0;
}
