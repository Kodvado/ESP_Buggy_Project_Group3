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
        red = !r;
        green = !g;
        blue = !b;
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

class unipolarmotor // Class for the unipolar motor
{
public:
    PwmOut pwm;     // Pulse width modulation output
    DigitalOut dir; // Direction of the motor
    potentiometer pot;
    unipolarmotor(PinName pwmPin, PinName dirPin, potentiometer potNo) : pwm(pwmPin), dir(dirPin), pot(potNo)
    {
        pwm.period(0.00001);    // Set the period of the PWM to 0.00001
        dir = 0;    // Set the initial direction to 0
        pwm = 0; // Set the initial PWM to 0
    }
    void update()
    {
        pwm = pot.read();   // Set the PWM to the value of the potentiometer
    }
    void changeDirection()
    {
        dir = !dir; // Change the direction of the motor
    }
};

potentiometer leftPot(A0);
potentiometer rightPot(A1);
unipolarmotor rightMotor(D9, D2, leftPot);  //D9 is the pwm pin, D2 is the direction pin
unipolarmotor leftMotor(D5, D7, rightPot);  //D5 is the pwm pin, D7 is the direction pin
DigitalOut EnableMDB(D8);
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

char c;
char w;

Serial hm10(PA_11, PA_12); // TX, RX
Serial pc(USBTX, USBRX); // TX, RX 

void enableMotorDriverBoard()   // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
}   // Function to enable the motor driver board

void uppressedint()     // Function to change the direction of the right motor
{
    rightMotor.changeDirection();
}

void downpressedint()   // Function to change the direction of the left motor
{   
    leftMotor.changeDirection();
}

void serial_config();

int main() // Main function
{
    // Set the baud rate of the serial communication
    pc.baud(9600);
    hm10.baud(9600);

    //Set the initial value of the EnableMDB to 0 (off)
    EnableMDB = 0; 
    //Setup interrupts
    uppressed.rise(&uppressedint);
    downpressed.rise(&downpressedint);
    FirePressed.rise(&enableMotorDriverBoard);

    //Set motors to unipoar mode
    Bipolar1 = 0;    // Set the bipolar 1 to 0
    Bipolar2 = 0;    // Set the bipolar 2 to 0  

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 0;  // Set the direction of the left motor to 0
    rightMotor.dir = 1; // Set the direction of the right motor to 1

    // Wait for the HM10 to be ready
    while(!hm10.readable()){ // wait for the HM10 to be ready

    // Declare LED
    RGBLed TLED(D5, D9, D8);
    TLED.color(0, 0, 0);
    while (1)
    {
        // Read the data from the HM10
        if (hm10.readable()) {
            c = hm10.getc();
            pc.putc(c);
            
            switch (c)
            {
            case 'F':
                leftMotor.pwm = 0.5;
                rightMotor.pwm = 0.5;
                break;
            case 'L':
                leftMotor.pwm = 0.3;
                rightMotor.pwm = 0.7;
                break;
            case 'R': 
                leftMotor.pwm = 0.7;
                rightMotor.pwm = 0.3;
                break;
            case 'B':
                leftMotor.dir = !leftMotor.dir;
                rightMotor.dir = !rightMotor.dir;
                leftMotor.pwm = 0.5;
                rightMotor.pwm = 0.5;
                break;
            case 'T':
                TLED.color(0, 0, 1);
                wait(0.5);
                TLED.color(0, 0, 1);
                break;
            default:
                break;
            }

            serial_config();
        }
        }
        lcd.locate(0, 0);
        lcd.printf("RM: %f    dir: %d", rightPot.read(), rightMotor.dir.read());
        lcd.locate(0, 10);
        lcd.printf("LM: %f    dir: %d", leftPot.read(), leftMotor.dir.read());
        wait(0.1);
    }
}

void serial_config()
{
    if(pc.readable())
    {
        w = pc.getc();
        hm10.putc(w);
    }
}