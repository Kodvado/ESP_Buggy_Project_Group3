
#include "C12832.h" //Imports the library for the LCD screen
#include "QEI.h"
#include "math.h"
#include "mbed.h" //Imports mbed libraries

C12832 lcd(D11, D13, D12, D7, D10); // Initializes the LCD screen
InterruptIn FirePressed(D4); // Initializes the interrupt for the fire button

class RGBLed {
public:
  PwmOut red;
  PwmOut green;
  PwmOut blue;
  RGBLed(PinName redPin, PinName greenPin, PinName bluePin)
      : red(redPin), green(greenPin), blue(bluePin) {
    red.period(0.01);
    green.period(0.01);
    blue.period(0.01);
  }
  void color(float r, float g, float b) {
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
  unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin) {
    pwm.period(0.00001);
    dir = 0;
  }
  void changeDirection() { dir = !dir; }
};

unipolarmotor leftMotor(D9, D2); // D9 is the pwm pin, D2 is the direction pin
unipolarmotor rightMotor(D5,
                         PA_13); // D5 is the pwm pin, D7 is the direction pin
// unipolarmotor rightMotor(D9, D2);  // D9 is the pwm pin, D2 is the direction
// pin unipolarmotor leftMotor(D5, PA_13); // D5 is the pwm pin, D7 is the
// direction pin
DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

void enableMotorDriverBoard() // Function to enable the motor driver board
{
  EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

int Pulse1 = 0;
int Pulse2 = 0;
int Last_Pulse1 = 0;
int Last_Pulse2 = 0;
float RPM1 = 0;
float RPM2 = 0;
float PPS1 = 0;
float PPS2 = 0;
float targetPPS = 3500;
float targetRPM = 10;

// Set the number of pulses of each encoder to perfrom certain actions
int meter = 1164; // Number of pulses of each encoder to move forward by 1 meter
int _90degree = 374;  // Number of pulses of the moving tyre to turn 90 degrees
int _180degree = 374; // Number of pulses of each encoder to turn 180 degrees

/*
QEI encoder1(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);
QEI encoder2(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);*/
QEI encoder1(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);
QEI encoder2(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);

Ticker encoderTicker;
void NUM1() {
  Pulse1 = encoder1.getPulses() - Last_Pulse1;
  Last_Pulse1 = encoder1.getPulses();
}

void NUM2() {
  Pulse2 = encoder2.getPulses() - Last_Pulse2;
  Last_Pulse2 = encoder2.getPulses();
}
void updateNUMS() {
  NUM1();
  NUM2();
}
void readEncoders() // Function to read the encoders
{
  Pulse1 = encoder1.getPulses();
  Pulse2 = encoder2.getPulses();
}

void motorStop() // Function to stop the motors
{
  leftMotor.pwm = 0;
  rightMotor.pwm = 0;
}

int main() {
  EnableMDB = 0; // Enable the motor driver board
  FirePressed.rise(&enableMotorDriverBoard);
  encoder1.reset();
  encoder2.reset();
  encoderTicker.attach(&updateNUMS, 0.01);
  // Set motors to unipoar mode
  Bipolar1 = 0; // Set the bipolar 1 to 0
  Bipolar2 = 0; // Set the bipolar 2 to 0

  // Set the initial direction of the motors to move forward
  leftMotor.dir = 1;  // Set the direction of the left motor to 0
  rightMotor.dir = 0; // Set the direction of the right motor to 1

  while (EnableMDB == 0) {
    wait(0.1);
  }
  motorStop();

  while (1) {
    // RPM1=(Pulse1*600)/1024;
    PPS1 = Pulse1 * 100;
    PPS2 = Pulse2 * 100;
    /*
    if(RPM1<targetRPM-5)    //speed control
    {
        if(leftMotor.pwm<1){
            leftMotor.pwm=leftMotor.pwm+0.1;   //if the buggy speed is to low,
    increase the duty cycle
        }

    }
    else if(RPM1>targetRPM+5)   //if the buggy speed is too high, decrease the
    duty cycle
    {
        if(leftMotor.pwm>0)
        {
            leftMotor.pwm=leftMotor.pwm-0.1;
        }

    }
    else{
        leftMotor.pwm=leftMotor.pwm;
    }*/
    if (PPS1 < targetPPS - 500) // speed control
    {
      if (leftMotor.pwm < 1) {
        leftMotor.pwm =
            leftMotor.pwm +
            0.1; // if the buggy speed is to low, increase the duty cycle
      }

    } else if (PPS1 > targetPPS + 500) // if the buggy speed is too high,
                                       // decrease the duty cycle
    {
      if (leftMotor.pwm > 0) {
        leftMotor.pwm = leftMotor.pwm - 0.1;
      }

    } else {
      leftMotor.pwm = leftMotor.pwm;
    }
    if (PPS2 < targetPPS - 500) // speed control
    {
      if (rightMotor.pwm < 1) {
        rightMotor.pwm = rightMotor.pwm + 0.1; // if the buggy speed is to low, increase the duty cycle
      }

    } else if (PPS2 > targetPPS + 500) // if the buggy speed is too high, decrease the duty cycle
    {
      if (rightMotor.pwm > 0) {
        rightMotor.pwm = rightMotor.pwm - 0.1;
      }

    } else {
      rightMotor.pwm = rightMotor.pwm;
    }
    wait(0.2);
  }
}