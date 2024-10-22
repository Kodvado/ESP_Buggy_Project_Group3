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

/*potentiometer leftPot(A0);
potentiometer rightPot(A1);*/
unipolarmotor rightMotor(D9, D2, rightPot);  // D9 is the pwm pin, D2 is the direction pin
unipolarmotor leftMotor(D5, PA_13, leftPot); // D5 is the pwm pin, D7 is the direction pin

DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

void enableMotorDriverBoard() // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

void uppressedint()
{
    rightMotor.changeDirection();
}

void downpressedint()
{
    leftMotor.changeDirection();
}

int Pulse1 = 0;
int Pulse2 = 0;
int Last_Pulse1 = 0;
int Last_Pulse2 = 0;
int RPM1 = 0;
int RPM2 = 0;

// Set the number of pulses of each encoder to perfrom certain actions
int meter = 1164;   // Number of pulses of each encoder to move forward by 1 meter
int _90degree = 374;   // Number of pulses of the moving tyre to turn 90 degrees
int _180degree = 374;  // Number of pulses of each encoder to turn 180 degrees

/*Encoder encoder1(PC_8, PC_6, 1024);
Encoder encoder2(PA_10, PB_3, 1024);*/
QEI encoder1(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);
QEI encoder2(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);


void readEncoders() // Function to read the encoders
{
    Pulse1 = encoder1.getPulses();
    Pulse2 = encoder2.getPulses();
}

void moveForward()  // Function to move forward
{
    encoder1.doreset();
    encoder2.doreset();
    leftMotor.pwm = 1;  // Set the PWM to 1
    rightMotor.pwm = 1; // Set the PWM to 1
    while (Pulse1 >= meter && Pulse2 >= meter) // 1164 is the number of pulses to move forward
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
    while (Pulse1 >= _90degree) // 374 is the number of pulses to turn right
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
    while (Pulse2 >= _90degree) // 374 is the number of pulses to turn left
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
    while (Pulse1 >= _90degree && Pulse2 >= _90degree) // 374 is the number of pulses to turn around
    {
        readEncoders();
    }
}
void motorStop()    // Function to stop the motors
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

int main()
{
    EnableMDB = 0; // Enable the motor driver board
    uppressed.rise(&uppressedint);
    downpressed.rise(&downpressedint);
    FirePressed.rise(&enableMotorDriverBoard);
    encoder1.doreset();
    encoder2.doreset();
    // Set motors to unipoar mode
    Bipolar1 = 0; // Set the bipolar 1 to 0
    Bipolar2 = 0; // Set the bipolar 2 to 0

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 0;  // Set the direction of the left motor to 0
    rightMotor.dir = 1; // Set the direction of the right motor to 1
    //Control Variables
    float vs1, vs2, vs3, vs4, vs5, vs6; //voltages of the line sensors 1 through 6
    float error=0;
    float last_error=0;
    float integral=0;
    float derivative=0;
    float kp =0;
    float ki =0;
    float kd =0;
    float dt = 0.1;
    float output;
    float leftMultiplier;
    float rightMultiplier;



    lcd.locate(0, 0);
    lcd.printf("Press button for Square");
    while (EnableMDB == 1)
    {
        wait(0.1);
    }
    lcd.cls();
    while (1)
    {
        error = 3*vs1+2*vs2+vs3-vs4-2*vs5-3*vs6;
        integral = integral+(error*dt);
        derivative = (error-last_error)/dt;
        output = kp*error + ki*integral+ kd*derivative;
        last_error=error;
        /*the speed of the left motor is dependent on what it would be without this algorithm, but also the output of the PID function. 
        If output were not scaled down by a factor of 4, the range of PWM values would be approx -1 to 3, which would make the steering unstable*/
        leftMultiplier=1+(output/4);
        rightMultiplier=1-(output/4);
        leftMotor.pwm=leftMultiplier*0.7;    
        rightMotor.pwm=rightMultiplier*0.7;
        wait(0.1);
    }  
    lcd.squre(0, 0, 10, 10, 1);
    
} 