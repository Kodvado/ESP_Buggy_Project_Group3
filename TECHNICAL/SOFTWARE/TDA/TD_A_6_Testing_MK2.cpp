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

void enableMotorDriverBoard() // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

int Pulse1 = 0;
int Pulse2 = 0;

// Set the number of pulses of each encoder to perfrom certain actions
int meter = 4656;     // Number of pulses of each encoder to move forward by 1 meter
int _90degree = 374;  // Number of pulses of the moving tyre to turn 90 degrees
int _180degree = 374; // Number of pulses of each encoder to turn 180 degrees

/*Encoder encoder1(PC_8, PC_6, 1024);
Encoder encoder2(PA_10, PB_3, 1024);*/
QEI encoder1(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);
QEI encoder2(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);

void readEncoders() // Function to read the encoders
{
    Pulse1 = encoder1.getPulses();
    Pulse2 = encoder2.getPulses();
}

void moveForward() // Function to move forward
{
    encoder1.reset();
    encoder2.reset();
    readEncoders();
    leftMotor.pwm = 0.5;                       // Set the PWM to 1
    rightMotor.pwm = 0.5;                      // Set the PWM to 1
    while (Pulse1 <= meter && Pulse2 <= meter) // 1164 is the number of pulses to move forward
    {
        readEncoders();
    }
}

void turnRight() // Function to turn right
{
    encoder1.reset();
    encoder2.reset();
    leftMotor.pwm = 0.25;       // Set the PWM to 0.5
    rightMotor.pwm = 0;         // Set the PWM to 1
    while (Pulse1 <= _90degree) // 374 is the number of pulses to turn right
    {
        readEncoders();
    }
}

void turnLeft() // Function to turn left
{
    encoder1.reset();
    encoder2.reset();
    leftMotor.pwm = 0;          // Set the PWM to 0
    rightMotor.pwm = 0.25;      // Set the PWM to 0.5
    while (Pulse2 <= _90degree) // 374 is the number of pulses to turn left
    {
        readEncoders();
    }
}

void turnAround() // Function to turn around
{
    encoder1.reset();
    encoder2.reset();
    leftMotor.pwm = 0.25;                              // Set the PWM to 0.5
    rightMotor.pwm = 0.25;                             // Set the PWM to 0.5
    while (Pulse1 <= _90degree && Pulse2 <= _90degree) // 374 is the number of pulses to turn around
    {
        readEncoders();
    }
}
void motorStop() // Function to stop the motors
{
    leftMotor.pwm = 0;
    rightMotor.pwm = 0;
}

void rightSquare()
{
    for (int i = 0; i < 4; i++)
    {
        moveForward();
        if (i < 3)
        {
            turnRight();
        }
        else
        {
            motorStop();
        }
    }
}

void leftSquare()
{
    for (int i = 0; i < 4; i++)
    {
        moveForward();
        if (i < 3)
        {
            turnLeft();
        }
        else
        {
            motorStop();
        }
    }
}

// Ticker encoderTicker;

// void printEncoders()
// {
//     lcd.locate(0, 10);
//     lcd.printf("Encoder 1: %d", encoder1.getPulses());
//     lcd.locate(0, 20);
//     lcd.printf("Encoder 2: %d", encoder2.getPulses());
// }

int main()
{
    EnableMDB = 0; // Disable the motor driver board
    FirePressed.rise(&enableMotorDriverBoard);
    encoder1.reset();
    encoder2.reset();
    // Set motors to unipoar mode
    Bipolar1 = 0; // Set the bipolar 1 to 0
    Bipolar2 = 0; // Set the bipolar 2 to 0

    //encoderTicker.attach(&printEncoders, 0.1);

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 0;  // Set the direction of the left motor to 0
    rightMotor.dir = 1; // Set the direction of the right motor to 1
    while (1)
    {
        lcd.locate(0, 0);
        lcd.printf("Press button for Square");
        while (EnableMDB == 0)
        {
            wait(0.1);
        }
        lcd.cls();
        rightSquare();
        turnAround();
        leftSquare();
        motorStop();
        lcd.locate(0, 0);
        lcd.printf("squared");
        

        EnableMDB = 0;
    }
}