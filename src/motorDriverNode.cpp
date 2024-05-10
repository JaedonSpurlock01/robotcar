#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <pigpiod_if2.h>
#include <cstdlib>

#define MOTOR_A_PIN1 13
#define MOTOR_A_PIN2 19
#define MOTOR_B_PIN1 12
#define MOTOR_B_PIN2 18
#define SLEEP_PIN 16
#define LASER_PIN 26
#define GREEN_LED_PIN 24
#define RED_LED_PIN 23
#define FORWARD 1
#define REVERSE 2
#define HIGH 1
#define LOW 0
#define FREQUENCY 1000
#define RANGE 255

class Motor
{
private:
    int outPin1;
    int outPin2;

public:
    Motor() {}
    Motor(int board, int pin1, int pin2)
    {
        this->outPin1 = pin1;
        this->outPin2 = pin2;
        set_mode(board, this->outPin1, PI_OUTPUT);
        set_mode(board, this->outPin2, PI_OUTPUT);

        set_PWM_frequency(board, this->outPin1, FREQUENCY);
        set_PWM_frequency(board, this->outPin2, FREQUENCY);
        set_PWM_range(board, this->outPin1, RANGE);
        set_PWM_range(board, this->outPin2, RANGE);

        set_PWM_dutycycle(board, this->outPin1, LOW); // blank signal
        set_PWM_dutycycle(board, this->outPin2, LOW); // blank signal
    }
    void update(int board, int direction, int dutyCycle)
    {
        if (direction == FORWARD)
        {
            set_PWM_dutycycle(board, this->outPin1, dutyCycle);
            gpio_write(board, this->outPin2, LOW);
        }
        else if (direction == REVERSE)
        {
            gpio_write(board, this->outPin1, LOW);
            set_PWM_dutycycle(board, this->outPin2, dutyCycle);
        }
    }
    void close(int board)
    {
        gpio_write(board, this->outPin1, LOW);
        gpio_write(board, this->outPin2, LOW);
    }
};
class MotorDriverSubscriber : public rclcpp::Node
{
public:
    MotorDriverSubscriber() : Node("MotorDriverSubscriber")
    {
        this->pi = pigpio_start(NULL, NULL);
        if (this->pi < 0)
        {
            throw std::runtime_error("Failed to initialize pigpio");
        }
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorDriverSubscriber::twistCallback, this, std::placeholders::_1));
        this->motorsA = Motor(this->pi, MOTOR_A_PIN1, MOTOR_A_PIN2);
        this->motorsB = Motor(this->pi, MOTOR_B_PIN1, MOTOR_B_PIN2);
        set_mode(this->pi, SLEEP_PIN, PI_OUTPUT);
        gpio_write(this->pi, SLEEP_PIN, HIGH);
    }

    ~MotorDriverSubscriber()
    {
        this->motorsA.close(this->pi);
        this->motorsB.close(this->pi);
        pigpio_stop(this->pi);
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Process received Twist message
        RCLCPP_INFO(this->get_logger(), "linear.x=%f, angular.z=%f, linear.z=%f",
                    msg->linear.x, msg->angular.z, msg->linear.z);

        int direction = (msg->linear.x > 0) ? FORWARD : REVERSE;
        float leftDutyCycle = (msg->linear.x == 0) ? 0 : std::abs(msg->linear.x) * RANGE;
        float rightDutyCycle = (msg->linear.x == 0) ? 0 : std::abs(msg->linear.x) * RANGE;
        if (direction == FORWARD)
        {
            gpio_write(this->pi, RED_LED_PIN, HIGH);
            gpio_write(this->pi, GREEN_LED_PIN, LOW);
        }
        else if (direction == REVERSE)
        {
            gpio_write(this->pi, RED_LED_PIN, LOW);
            gpio_write(this->pi, GREEN_LED_PIN, HIGH);
        }

        if (msg->linear.z)
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    Motor motorsA;
    Motor motorsB;
    int pi;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriverSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}