/*
 * chassis_kinematics.h
 *
 *  Created on: Apr 21, 2025
 *      Author: root
 */

#ifndef INC_CHASSIS_KINEMATICS_H_
#define INC_CHASSIS_KINEMATICS_H_

#include "main.h"
#include <math.h>
#include <stdlib.h>  // For abs() function

extern TIM_HandleTypeDef htim3;  // Add this line after the includes

/* Robot Configuration */
#define ROBOT_CONFIG_VERSION "1.0"

/* Physical Dimensions */
#define WHEEL_BASE 0.5f    // Distance between front and rear wheels (m)
#define TRACK_WIDTH 0.5f   // Distance between left and right wheels (m)

/* Control Parameters */
#define MAX_PWM 300       // Maximum PWM value
#define MIN_PWM -300      // Minimum PWM value
#define JOYSTICK_DEADZONE 0.2f  // Joystick deadzone threshold
#define BRAKE_THRESHOLD 0.2f   // Threshold for applying brake

// In chassis_kinematics.h
#define MOVEMENT_SCALE 0.6f    // Reduce overall movement speed 0.8 bolgoj uzeh
#define ROTATION_SCALE 0.5f    // Reduce rotation speed

// Structure to hold wheel PWM values
typedef struct {
    int16_t front_left;
    int16_t front_right;
    int16_t rear_left;
    int16_t rear_right;
} WheelPWM;

// Structure to hold robot velocity
typedef struct {
    float vx;     // Linear velocity in x direction [-1, 1]
    float vy;     // Linear velocity in y direction [-1, 1]
    float omega;  // Angular velocity [-1, 1]
} RobotVelocity;

// Function prototypes
void calculateWheelPWM(RobotVelocity* robot_vel, WheelPWM* wheel_pwm);
void setMotorPWM(WheelPWM* wheel_pwm);
void applyBrake(void);

#endif /* INC_CHASSIS_KINEMATICS_H_ */
