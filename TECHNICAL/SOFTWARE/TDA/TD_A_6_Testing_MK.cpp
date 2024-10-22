#include "mbed.h"   //Imports mbed libraries
#include "C12832.h" //Imports the library for the LCD screen
#include "math.h"
#include "QEI.h"

C12832 lcd(D11, D13, D12, D7, D10); // Initializes the LCD screen

InterruptIn downpressed(A3); // Initializes the interrupt for the down button
InterruptIn uppressed(A2);   // Initializes the interrupt for the up button
InterruptIn FirePressed(D4); // Initializes the interrupt for the fire button

// class LCD
// {
// public:
//     void print(int x, int y, char *text)
//     {
//         lcd.locate(x, y);
//         lcd.printf(text);
//     }
// };

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
    potentiometer pot;
    unipolarmotor(PinName pwmPin, PinName dirPin, potentiometer potNo) : pwm(pwmPin), dir(dirPin), pot(potNo)
    {
        pwm.period(0.00001);
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
    void fullDutyCycle
    {
        pwm = 1;
    }
    void halfDutyCycle
    {
        pwm = 1;
    }
};


// potentiometer leftPot(A0);
// potentiometer rightPot(A1);

// Define pins

unipolarmotor rightMotor(D9, D2, rightPot);  // D9 is the pwm pin, D2 is the direction pin
unipolarmotor leftMotor(D5, PA_13, leftPot); // D5 is the pwm pin, D7 is the direction pin

DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

// Set the number of pulses of each encoder to perfrom certain actions
int meter = 1164;   // Number of pulses of each encoder to move forward by 1 meter
int _90degree = 374;   // Number of pulses of the moving tyre to turn 90 degrees
int _180degree = 374;  // Number of pulses of each encoder to turn 180 degrees



int Pulse1 = 0;
int Pulse2 = 0;
int Last_Pulse1 = 0;
int Last_Pulse2 = 0;
int RPM1 = 0;
int RPM2 = 0;

Encoder encoder1(PC_8, PC_6, 1024);
Encoder encoder2(PA_10, PB_3, 1024);

//Function Declarations

void ToggleMDB() // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

// void uppressedint()
// {
//     rightMotor.changeDirection();
// }

// void downpressedint()
// {
//     leftMotor.changeDirection();
// }

// void NUM1()
// {
//     Pulse1 = encoder1.returnPulses() - Last_Pulse1;
//     Last_Pulse1 = encoder1.returnPulses();
// }

// void NUM2()
// {
//     Pulse2 = encoder2.returnPulses() - Last_Pulse2;
//     Last_Pulse2 = encoder2.returnPulses();
// }

void readEncoders() // Function to read the encoders
{
    Pulse1 = encoder1.getPulses();
    Pulse2 = encoder2.getPulses();
}

void moveForward()  // Function to move forward by 1 meter
{
    encoder1.doreset();
    encoder2.doreset();
    leftMotor.pwm = 1;  // Set the PWM to 1
    rightMotor.pwm = 1; // Set the PWM to 1
    while (Pulse1 >= 1164 && Pulse2 >= 1164) // 1164 is the number of pulses to move forward
    {
        readEncoders();
    }
}
void turnRight() // Function to turn right
{
    encoder1.doreset();
    encoder2.doreset();
    leftMotor.pwm = 0.5;  // Set the PWM to 0.5
    rightMotor.pwm = 0;   // Set the PWM to 1
    while (Pulse1 >= X) // X is the number of pulses to turn right
    {
        readEncoders();
    }
}

void turnLeft() // Function to turn left
{
    encoder1.doreset();
    encoder2.doreset();
    leftMotor.pwm = 0;   // Set the PWM to 0
    rightMotor.pwm = 0.5;  // Set the PWM to 0.5
    while (Pulse2 >= X) // X is the number of pulses to turn left
    {
        readEncoders();
    }
}

void turnAround()  // Function to turn around
{
    encoder1.doreset();
    encoder2.doreset();
    leftMotor.pwm = 0.5;  // Set the PWM to 0.5
    rightMotor.pwm = 0.5;   // Set the PWM to 0.5
    while (Pulse1 >= X && Pulse2 >= X) // X is the number of pulses to turn around
    {
        readEncoders();
    }
}
void motorStop()    // Function to stop the motors
{
    leftMotor.pwm = 0;
    rightMotor.pwm = 0;
}

int main()
{
    EnableMDB = 0; // Enable the motor driver board
    uppressed.rise(&uppressedint);
    downpressed.rise(&downpressedint);
    FirePressed.rise(&ToggleMDB
);
    encoder1.doreset();
    encoder2.doreset();
    // Set motors to unipoar mode
    Bipolar1 = 0; // Set the bipolar 1 to 0
    Bipolar2 = 0; // Set the bipolar 2 to 0

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 0;  // Set the direction of the left motor to 0
    rightMotor.dir = 1; // Set the direction of the right motor to 1


    // while (1)
    // {
    //     RPM1 = Pulse1 * 60 / 1024;
    //     RPM2 = Pulse2 * 60 / 1024;
    //     screen();
    //     wait(0.2);
    //     lcd.locate(0, 0);
    //     rightMotor.update();
    //     leftMotor.update();
    //     wait(0.1);
    // }
    moveForward();  // Move forward
    turnRight();    // Turn right
    moveForward();  // Move forward
    turnRight();    // Turn right
    moveForward();
    turnRight();
    moveForward();
    turnAround();
    moveForward();
    turnLeft();
    moveForward();
    turnLeft();
    moveForward();
    turnLeft();
    moveForward();
    motorStop();
    lcd.squre(0, 0, 10, 10, 1);
    while (1)
    {
        wait(0.1);
    }
}