#include "motor.h"


Motor::Motor(){};

/**
 * @brief Motor custom constrtuctor
 * 
 * @param int board the gpiochip to progam the motor to
 * @param int pin1 first gpio pin to program
 * @param int pin2 second gpio pin to program
*/
Motor::Motor(int board, int pin1, int pin2){
    this->outPin1 = pin1;
    this->outPin2 = pin2;
    set_mode(board, this->outPin1, PI_OUTPUT);
    set_mode(board, this->outPin2, PI_OUTPUT);

    set_PWM_frequency(board,this->outPin1, FREQUENCY);
    set_PWM_frequency(board, this->outPin2, FREQUENCY);
    set_PWM_range(board, this->outPin1, RANGE);
    set_PWM_range(board, this->outPin2, RANGE);

    set_PWM_dutycycle(board, this->outPin1, LOW); // blank signal
    set_PWM_dutycycle(board, this->outPin2, LOW); // blank signal
};

/**
 * @brief Update the direction and dutyCycle of the Motor
 * 
 * @param int board the gpioochip to target
 * @param int direction (Forward or Back) 
 * @param dutyCycle the dutyCycle to output on the motor
*/
void Motor::update(int board, int direction, int dutyCycle){
    if(direction == FORWARD)
    {
        set_PWM_dutycycle(board, this->outPin1, dutyCycle);
        gpio_write(board, this->outPin2, LOW);
    }
    else if(direction == REVERSE)
    {
        gpio_write(board, this->outPin1, LOW);
        set_PWM_dutycycle(board, this->outPin2, dutyCycle);
    }
};


/**
 * @brief Close set the GPIO pins to low before terminating program - used in MotorDriverSubscriber destructor
 * 
 * @param int board the GPIO chip to target
*/
void Motor::close(int board){
    gpio_write(board, this->outPin1, LOW);
    gpio_write(board, this->outPin2, LOW);
}