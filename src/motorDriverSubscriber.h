#pragma once

#include "motor.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define MOTOR_A_PIN1 13
#define MOTOR_A_PIN2 19
#define MOTOR_B_PIN1 12
#define MOTOR_B_PIN2 18
#define SLEEP_PIN 16
#define LASER_PIN 26
#define GREEN_LED_PIN 24
#define RED_LED_PIN 23

class MotorDriverSubscriber : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    Motor motorsA;
    Motor motorsB;
    int pi;

    /**
     * @brief Functor for processing data published to cmd_vel
     *
     * @param msg Twist message published by the controllerNode
     */
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    /**
     * @brief Construct ROS subscriber Node for Motor Driver
     *
     */
    MotorDriverSubscriber() : Node("MotorDriverSubscriber");

    /**
     * @brief Destruct ROS Motor Driver subscriber Node
     */
    ~MotorDriverSubscriber();
};