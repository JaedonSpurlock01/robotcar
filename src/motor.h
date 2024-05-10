#pragma once

#include <pigpiod_if2.h>
#include <cstdlib>

#define HIGH          1
#define LOW           0
#define FREQUENCY     1000 // bandwidth of pwm pin
#define RANGE         255 // DutyCycle range (0-255)
#define FORWARD       1 
#define REVERSE       2

class Motor{
    private:
        int outPin1;
        int outPin2;

    public:
    /**
     * @brief Motor default constructor
    */
        Motor(); 

        /**
         * @brief Motor custom constrtuctor
         * 
         * @param int board the gpiochip to progam the motor to
         * @param int pin1 first gpio pin to program
         * @param int pin2 second gpio pin to program
        */
       Motor(int board, int pin1, int pin2);

       ~Motor(); // destructor does nothing since using static data types


        /**
         * @brief Update the direction and dutyCycle of the Motor
         * 
         * @param int board the gpioochip to target
         * @param int direction (Forward or Back) 
         * @param dutyCycle the dutyCycle to output on the motor
        */
       void update(int board, int direction, int dutyCycle);

        /**
         * @brief Close set the GPIO pins to low before terminating program - used in MotorDriverSubscriber destructor
         * 
         * @param int board the GPIO chip to target
        */
       void close (int board);
};