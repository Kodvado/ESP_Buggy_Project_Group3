#include "mbed.h"   //Imports mbed libraries
#include "C12832.h" //Imports the library for the LCD screen
#include "math.h"

C12832 lcd(D11, D13, D12, D7, D10); // Initializes the LCD screen

InterruptIn downpressed(A3); // Initializes the interrupt for the down button
InterruptIn uppressed(A2);   // Initializes the interrupt for the up button
InterruptIn FirePressed(D4); // Initializes the interrupt for the fire button

class LCD
{
public:
    void print(int x, int y, char *text)
    {
        lcd.locate(x, y);
        lcd.printf(text);
    }
};

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

class potentiometer
{
public:
    AnalogIn pot;
    potentiometer(PinName pin) : pot(pin) {}
    float read()
    {
        return pot;
    }
};

class unipolarmotor // Class for the unipolar motor()
{
public:
    PwmOut pwm;     // Pulse width modulation output
    DigitalOut dir; // Direction of the motor
    potentiometer pot;
    unipolarmotor(PinName pwmPin, PinName dirPin, potentiometer potNo) : pwm(pwmPin), dir(dirPin), pot(potNo)
    {
        pwm.period(0.0002);
        dir = 0;
    }
    void update()
    {
        pwm = pot.read();
    }
    void changeDirection()
    {
        dir = !dir;
    }
};

potentiometer leftPot(A0);
potentiometer rightPot(A1);
unipolarmotor rightMotor(D9, D2, rightPot);  //D9 is the pwm pin, D2 is the direction pin
unipolarmotor leftMotor(D5, PA_13, leftPot);  //D5 is the pwm pin, D7 is the direction pin

DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

void enableMotorDriverBoard()   // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
}   // Function to enable the motor driver board

void uppressedint() 
{
    rightMotor.changeDirection();
}

void downpressedint()
{
    leftMotor.changeDirection();
}

int main() // Main function
{
    EnableMDB = 0; // Enable the motor driver board
    uppressed.rise(&uppressedint);
    downpressed.rise(&downpressedint);
    FirePressed.rise(&enableMotorDriverBoard);

    //Set motors to unipoar mode
    Bipolar1 = 0;    // Set the bipolar 1 to 0
    Bipolar2 = 0;    // Set the bipolar 2 to 0  

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 0;  // Set the direction of the left motor to 0
    rightMotor.dir = 1; // Set the direction of the right motor to 1

    while (1)
    {
        lcd.locate(0, 0);
        lcd.printf("RM: %f    dir: %d", rightPot.read(), rightMotor.dir.read());
        lcd.locate(0, 10);
        lcd.printf("LM: %f    dir: %d", leftPot.read(), leftMotor.dir.read());
        rightMotor.update();
        leftMotor.update();
        wait(0.1);
    }
}
