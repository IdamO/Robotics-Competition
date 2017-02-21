/*
 * PITA_Funct.h
 *
 *  Created on: Jan 4, 2017
 *      Author: tevin
 */

#ifndef PITA_FUNCT_H_
#define PITA_FUNCT_H_

extern int game_field[49];
extern int tunnel_map[49];
extern int grid_orientation;

////////////////////////////////////////////////
// Functions								  //
////////////////////////////////////////////////

//---------------------------------------------------------------------------
// hardware_init()
//
// inits hardware being used
//---------------------------------------------------------------------------
void hardware_init(void);


void command_line(void);
void UARTNextLine(void);
void delay(uint32_t time);
void step_off_R(void);

//---------------------------------------------------------------------------
// uSensor()
//
// Determine distance between ultrasnoic sensor and nearest obstacle (up to 4m)
//---------------------------------------------------------------------------
double uSensor(void);


uint32_t ir_sensor(void);

//---------------------------------------------------------------------------
// motor_move()
//
// rotate the stepper motors a certain amount of steps
//---------------------------------------------------------------------------
void motor_move(uint32_t steps,char continuous);

void motor_rotate(int degrees,char dir);
//---------------------------------------------------------------------------
// motor_rotateRight_90()
//
// rotate the robot to the right 90 degrees
//---------------------------------------------------------------------------
void motor_rotateRight_90(void);

//---------------------------------------------------------------------------
// motor_rotateLeft_90()
//
// rotate the robot to the left 90 degrees
//---------------------------------------------------------------------------
void motor_rotateLeft_90(void);
int motor_ramp(int currStep,int maxSteps,int finalDelay,char continuous, int currDelay);
void step_off_R(void);
void move_forward_1ft(void);
void move(int inches,char dir,char continuous);
void navigate_snake(void);
int avoid_obstacle(int currentGrid);
void map_tunnels(void);
void timer_wait(int microseconds);
void led_control(void);
void cache_align(void);
uint32_t capacitive_sensor(void);
void rotate_arm(int degrees);
void move_arm(char dir);
void arm_grip(char open);
int detect_line(void);
int getData(void);
uint32_t locate_cache(void);
uint8_t find_pips(void);

#endif /* PITA_FUNCT_H_ */
