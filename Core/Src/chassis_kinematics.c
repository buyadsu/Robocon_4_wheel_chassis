/*
 * chassis_kinematics.c
 *
 *  Created on: Apr 21, 2025
 *      Author: root
 */


#include "chassis_kinematics.h"
#include "main.h"

// Calculate wheel PWM values based on robot velocity
void calculateWheelPWM(RobotVelocity* robot_vel, WheelPWM* wheel_pwm) {
    if (fabsf(robot_vel->vy) < BRAKE_THRESHOLD && fabsf(robot_vel->vx) < BRAKE_THRESHOLD && fabsf(robot_vel->omega) < BRAKE_THRESHOLD) {
        applyBrake();

        // Clear wheel speeds when braking
        wheel_pwm->front_left = 0;
        wheel_pwm->front_right = 0;
        wheel_pwm->rear_left = 0;
        wheel_pwm->rear_right = 0;
        return;
    }
    // Scale down the movements
    float scaled_vx = robot_vel->vx * MOVEMENT_SCALE;
    float scaled_vy = robot_vel->vy * MOVEMENT_SCALE;
    float scaled_omega = robot_vel->omega * ROTATION_SCALE;

    // Calculate wheel speeds with scaled values
    wheel_pwm->front_left = (int16_t)((scaled_vx + scaled_vy + scaled_omega) * MAX_PWM);
    wheel_pwm->front_right = (int16_t)((scaled_vx - scaled_vy - scaled_omega) * MAX_PWM);
    wheel_pwm->rear_left = (int16_t)((scaled_vx - scaled_vy + scaled_omega) * MAX_PWM);
    wheel_pwm->rear_right = (int16_t)((scaled_vx + scaled_vy - scaled_omega) * MAX_PWM);

    setMotorPWM(wheel_pwm);
}

// Set motor PWM values
void setMotorPWM(WheelPWM* wheel_pwm) {
    // Front right motor
    if (wheel_pwm->front_right >= 0) {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)abs(wheel_pwm->front_right));

    // Front left motor
    if (wheel_pwm->front_left >= 0) {
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)abs(wheel_pwm->front_left));

    // Rear right motor
    if (wheel_pwm->rear_right >= 0) {
        HAL_GPIO_WritePin(INA3_GPIO_Port, INA3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB3_GPIO_Port, INB3_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(INA3_GPIO_Port, INA3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB3_GPIO_Port, INB3_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)abs(wheel_pwm->rear_right));

    // Rear left motor
    if (wheel_pwm->rear_left >= 0) {
        HAL_GPIO_WritePin(INA4_GPIO_Port, INA4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB4_GPIO_Port, INB4_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(INA4_GPIO_Port, INA4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB4_GPIO_Port, INB4_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)abs(wheel_pwm->rear_left));
}

// Apply brake to all motors
void applyBrake(void) {
    // Set PWM to 0 for all channels
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    // Set brake mode for all motors (INA and INB high)
    // Front left
    HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_SET);

    // Front right
    HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);

    // Rear left
    HAL_GPIO_WritePin(INA3_GPIO_Port, INA3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB3_GPIO_Port, INB3_Pin, GPIO_PIN_SET);

    // Rear right
    HAL_GPIO_WritePin(INA4_GPIO_Port, INA4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB4_GPIO_Port, INB4_Pin, GPIO_PIN_SET);
}
