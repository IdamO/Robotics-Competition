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
void hardware_init(void);
void command_line(void);
void UARTNextLine(void);
void delay(uint32_t time);
void motor_moveForward(uint32_t steps);
void motor_rotateRight_90(void);
void step_off_R(void);
void motor_rotateLeft_90(void);
uint32_t ir_sensor(void);
void move_forward_1ft(void);
void navigate_snake(void);
int avoid_obstacle(int currentGrid);
double uSensor(void);
void map_tunnels(void);
void timer_wait(int microseconds);
void led_control(void);
void cache_align(void);
uint32_t capacitive_sensor(void);
void arm_control(void);
void rotate_arm(int degrees);
void move_arm(char dir);
void arm_grip(char open);

#endif /* PITA_FUNCT_H_ */
