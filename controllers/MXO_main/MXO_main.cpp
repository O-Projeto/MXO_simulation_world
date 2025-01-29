#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <cmath>
#include "odom.h"

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_RADIUS 0.03
#define WHEEL_TRACK 0.3  // Distância entre rodas

using namespace webots;

// PID Gains
double Kp = 1.5;
double Ki = 0.01;
double Kd = 0.1;
double integral = 0;
double prev_error = 0;

robot_position robot_pos;

struct Waypoint {
    double x;
    double y;
};

std::vector<Waypoint> waypoints = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
size_t current_waypoint = 0;

double pid_control(double error) {
    integral += error * (TIME_STEP / 1000.0);
    double derivative = (error - prev_error) / (TIME_STEP / 1000.0);
    prev_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("motor_left");
    Motor *rightMotor = robot->getMotor("motor_direita");
    
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    
    PositionSensor *encoderL = robot->getPositionSensor("encoder_left");
    PositionSensor *encoderR = robot->getPositionSensor("encoder_right");
    encoderL->enable(TIME_STEP);
    encoderR->enable(TIME_STEP);
    
    InertialUnit *imu = robot->getInertialUnit("IMU");
    imu->enable(TIME_STEP);
    
    double target_linear_velocity = 0.3;

    while (robot->step(TIME_STEP) != -1) {
        double encoderLeft = encoderL->getValue();
        double encoderRight = encoderR->getValue();
        const double *imu_read = imu->getRollPitchYaw();
        
        robot_pos = odom(encoderLeft, encoderRight, imu_read[2]);
        
        if (current_waypoint < waypoints.size()) {
            double target_x = waypoints[current_waypoint].x;
            double target_y = waypoints[current_waypoint].y;
            double error_x = target_x - robot_pos.x;
            double error_y = target_y - robot_pos.y;
            double distance = sqrt(error_x * error_x + error_y * error_y);
            
            if (distance < 0.05) {
                current_waypoint++;
                if (current_waypoint >= waypoints.size()) break;
            }
            
            double desired_theta = atan2(error_y, error_x);
            double error_theta = desired_theta - robot_pos.theta;
            double omega_correction = pid_control(error_theta);
            
            double v_left = target_linear_velocity - (omega_correction * WHEEL_TRACK / 2);
            double v_right = target_linear_velocity + (omega_correction * WHEEL_TRACK / 2);
            
            v_left = std::max(-MAX_SPEED, std::min(MAX_SPEED, v_left));
            v_right = std::max(-MAX_SPEED, std::min(MAX_SPEED, v_right));
            
            leftMotor->setVelocity(v_left);
            rightMotor->setVelocity(v_right);
        } else {
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
        }
        
        std::cout << "X: " << robot_pos.x << " | Y: " << robot_pos.y;
        std::cout << " | Th: " << robot_pos.theta;
        std::cout << " | Target X: " << waypoints[current_waypoint].x;
        std::cout << " | Target Y: " << waypoints[current_waypoint].y << std::endl;
    }

    delete robot;
    return 0;
}