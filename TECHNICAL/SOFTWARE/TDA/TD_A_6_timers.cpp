#include "mbed.h"   //Imports mbed libraries
#include "C12832.h" //Imports the library for the LCD screen
#include "math.h"
#include "QEI.h"

C12832 lcd(D11, D13, D12, D7, D10); // Initializes the LCD screen
InterruptIn FirePressed(D4);        // Initializes the interrupt for the fire button

class RGBLed
{
public:
    PwmOut red;
    PwmOut green;
    PwmOut blue;
    RGBLed(PinName redPin, PinName greenPin, PinName bluePin) : red(redPin), green(greenPin), blue(bluePin)
    {
        red.period(0.01);
        green.period(0.01);
        blue.period(0.01);
    }
    void color(float r, float g, float b)
    {
        red = r;
        green = g;
        blue = b;
    }
};

class unipolarmotor // Class for the unipolar motor
{
public:
    PwmOut pwm;     // Pulse width modulation output
    DigitalOut dir; // Direction of the motor
    // potentiometer pot;
    unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin)
    {
        pwm.period(0.00001);
        dir = 0;
    }
    void changeDirection()
    {
        dir = !dir;
    }
};

unipolarmotor rightMotor(D9, D2);   // D9 is the pwm pin, D2 is the direction pin
unipolarmotor leftMotor(D5, PA_13); // D5 is the pwm pin, D7 is the direction pin

DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

float _90turntime = 0.5; // Time for the 90 degree turn
float _180turntime = 1;  // Time for the 180 degree turn
float _1mforwardtime = 2; // Time for the forward movement of 1 m

void enableMotorDriverBoard() // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

Timeout foward1mTimeout; // Timeout for the forward movement of 1 m
Timeout rightTimeout;  // Timeout for the right movement
Timeout leftTimeout;   // Timeout for the left movement
Timeout _180Timeout;   // Timeout for the 180 degree turn

void stop() // Function to stop the robot
{
    rightMotor.dir = 1;
    leftMotor.dir = 0;
    rightMotor.pwm = 0;
    leftMotor.pwm = 0;
}


void forward1m() // Function to move the robot forward by 1 m
{
    rightMotor.pwm = 0.5;
    leftMotor.pwm = 0.5;
    foward1mTimeout.attach(&stop, _1mforwardtime);
}

void right() // Function to move the robot right
{
    rightMotor.pwm = 0;
    leftMotor.pwm = 0.5;
    rightTimeout.attach(&stop, _90turntime);
}

void left() // Function to move the robot left
{
    rightMotor.pwm = 0.5;
    leftMotor.pwm = 0;
    leftTimeout.attach(&stop, _90turntime);
}

void _180() // Function to turn the robot 180 degrees
{
    rightMotor.pwm = 0.5;
    leftMotor.pwm = 0.5;
    rightMotor.dir = 1;
    leftMotor.dir = 1;
    _180Timeout.attach(&stop, _180turntime);
}


int main()
{
    rightMotor.dir = 1;
    leftMotor.dir = 0;
    EnableMDB = 0; // Disables the motor driver board
    FirePressed.rise(&enableMotorDriverBoard); // Interrupt for the fire button
    lcd.cls();                                 // Clears the LCD screen
    lcd.locate(0, 0);                          // Sets the cursor to the top left corner of the LCD screen
    lcd.printf("Press the fire button to start"); // Prints the message on the LCD screen
    while (1)
    {
        if (EnableMDB == 1) // If the motor driver board is enabled
        {
            lcd.cls(); // Clears the LCD screen
            for(int i = 0; i < 4; i++)
            {
                forward1m();
                wait();
                right();
                wait();
            }
        }
        else
        {
            lcd.locate(0, 0); // Sets the cursor to the top left corner of the LCD screen
            lcd.printf("Press the fire button to start"); // Prints the message on the LCD screen
        }
    }
}
