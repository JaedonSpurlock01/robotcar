#include "motorDriverSubscriber.h"

/**
 * @brief Functor for processing data publsihed to cmd_vel ros topic
 * determine direction bsed on liner sign
 * determine dutyCycle from linear.x
 * simulte turning by  multiplying turn side dutyCyle by fctor of ngulr
 *
 * @param msg Twist message published by the controllerNode
 */
MotorDriverSubscriber::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Process received Twist message
    RCLCPP_INFO(this->get_logger(), "linear.x=%f, angular.z=%f, linear.z=%f",
                msg->linear.x, msg->angular.z, msg->linear.z);

    int direction = (msg->linear.x > 0) ? FORWARD : REVERSE;
    float leftDutyCycle = (msg->linear.x == 0) ? 0 : std::abs(msg->linear.x) * RANGE;
    float rightDutyCycle = (msg->linear.x == 0) ? 0 : std::abs(msg->linear.x) * RANGE;

    if (msg.linear.z)
    {
        gpio_write(this->pi, LASER_PIN, HIGH);
    }
    else
    {
        gpio_write(this->pi, LASER_PIN, LOW);
    }

    if (msg->angular.z > 0.1)
    {
        leftDutyCycle = std::abs(msg->angular.z) * 0.1 * leftDutyCycle;
        RCLCPP_INFO(get_logger(), "turning left with duty cycle of %f", leftDutyCycle);
    }
    else if (msg->angular.z < -0.1)
    {
        rightDutyCycle = std::abs(msg->angular.z) * 0.1 * rightDutyCycle;
        RCLCPP_INFO(get_logger(), "turning right with duty cycle of %f", rightDutyCycle);
    }

    motorsA.update(this->pi, direction, leftDutyCycle);
    motorsB.update(this->pi, direction, rightDutyCycle);
}

/**
 * @brief Destruct ROS Motor Driver subscriber Node
 */
MotorDriverSubscriber::~MotorDriverSubscriber()
{
    this->motorsA.close(this->pi);
    this->motorsB.close(this->pi);
    pigpio_stop(this->pi);
}

/**
 * @brief Construct ROS subscriber Node for motor driver
 * Iniialize motor
 */
MotorDriverSubscriber::MotorDriverSubscriber() : Node("MotorDriverSubscriber")
{

    this->pi = pigpio_start(NULL, NULL); // connect process to pigpiod
    if (this->pi < 0)
    {
        throw std::runtime_error("Failed to initialize pigpio");
    }

    // Subscriber object sucribe to cmd_vel topic, buffer queue size 10, functor twistCllbck
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotorDriverSubscriber::twistCallback, this, std::placeholders::_1));

    // Initialize motors (pin out in heder file)
    this->motorsA = Motor(this->pi, MOTOR_A_PIN1, MOTOR_A_PIN2);
    this->motorsB = Motor(this->pi, MOTOR_B_PIN1, MOTOR_B_PIN2);
    set_mode(this->pi, SLEEP_PIN, PI_OUTPUT);
    gpio_write(this->pi, SLEEP_PIN, HIGH);

    set_mode(this->pi, LASER_PIN, PI_OUTPUT);
    set_mode(this->pi, GREEN_LED_PIN, PI_OUTPUT);
    set_mode(this->pi, RED_LED_PIN, PI_OUTPUT);
}