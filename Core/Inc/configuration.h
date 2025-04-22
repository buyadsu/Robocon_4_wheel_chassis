/*
 * configuration.h
 *
 *  Created on: Apr 21, 2025
 *      Author: root
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

/* Robot Configuration */
#define ROBOT_CONFIG_VERSION "1.0"

/* Physical Dimensions */
#define WHEEL_BASE 0.5f    // Distance between front and rear wheels (m)
#define TRACK_WIDTH 0.5f   // Distance between left and right wheels (m)
#define WHEEL_RADIUS 0.1f  // Wheel radius (m)

/* Performance Limits */
#define MAX_LINEAR_VELOCITY 2.0f    // Maximum linear velocity (m/s)
#define MAX_ANGULAR_VELOCITY 2.0f   // Maximum angular velocity (rad/s)
#define MAX_WHEEL_SPEED 1000.0f     // Maximum wheel speed (RPM)
#define MIN_WHEEL_SPEED -1000.0f    // Minimum wheel speed (RPM)

/* Control Parameters */
#define JOYSTICK_TIMEOUT_MS 500
#define PWM_FREQUENCY 5000          // PWM frequency (Hz)
#define PWM_PERIOD 200              // Timer period for PWM
#define PWM_PRESCALER 169           // Timer prescaler for PWM

#endif /* INC_CONFIGURATION_H_ */
