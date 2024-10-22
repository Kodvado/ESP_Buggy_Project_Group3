#include "C12832.h" //Imports the library for the LCD screen
#include "QEI.h"
#include "math.h"
#include "mbed.h" //Imports mbed libraries
// #include "PID.h"

C12832 lcd(D11, D13, D12, D7, D10); // Initializes the LCD screen
InterruptIn FirePressed(D4);        // Initializes the interrupt for the fire button

class unipolarmotor // Class for the unipolar motor
{
public:
    PwmOut pwm;     // Pulse width modulation output
    DigitalOut dir; // Direction of the motor
    // potentiometer pot;
    unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin)
    {
        pwm.period(0.00004);
        dir = 0;
    }
    void changeDirection() { dir = !dir; }
};

unipolarmotor leftMotor(D9, D2);     // D9 is the pwm pin, D2 is the direction pin
unipolarmotor rightMotor(D5, PA_13); // D5 is the pwm pin, D7 is the direction pin

DigitalOut EnableMDB(D8); // Enable motor driver board
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

// PID variables
float integral_left = 0, previous_error_left = 0;
float integral_right = 0, previous_error_right = 0;

// PID controller function
float PID_controller(float error, float &integral, float &previous_error, float dt, float Kp, float Ki, float Kd)
{
    integral += error * dt;
    float derivative = (error - previous_error)/dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return output;
}

void enableMotorDriverBoard() // Function to enable the motor driver board
{
    EnableMDB = !EnableMDB;
} // Function to enable the motor driver board

int Pulse1 = 0;
int Pulse2 = 0;
int Last_Pulse1 = 0;
int Last_Pulse2 = 0;
float PPS1 = 0;
float PPS2 = 0;
float L_targetPPS = 3500;
float R_targetPPS = 3500;

// Set the number of pulses of each encoder to perfrom certain actions
int meter = 1164;     // Number of pulses of each encoder to move forward by 1 meter
int _90degree = 374;  // Number of pulses of the moving tyre to turn 90 degrees
int _180degree = 374; // Number of pulses of each encoder to turn 180 degrees

QEI encoder1(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);
QEI encoder2(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);

Ticker encoderTicker;

void updateEncoders()
{
    float temp1 = encoder1.getPulses();
    float temp2 = encoder2.getPulses();
    Pulse1 = temp1 - Last_Pulse1;
    Last_Pulse1 = temp1;
    Pulse2 = temp2 - Last_Pulse2;
    Last_Pulse2 = temp2;
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

// Main function
int main()
{
    EnableMDB = 0; // Enable the motor driver board
    FirePressed.rise(&enableMotorDriverBoard);
    encoder1.reset();
    encoder2.reset();
    encoderTicker.attach(&updateEncoders, 0.01);
    // Set motors to unipoar mode
    Bipolar1 = 0; // Set the bipolar 1 to 0
    Bipolar2 = 0; // Set the bipolar 2 to 0

    // Set the initial direction of the motors to move forward
    leftMotor.dir = 1;  // Set the direction of the left motor to 0
    rightMotor.dir = 0; // Set the direction of the right motor to 1

    // Set PID constants and variables
    float RM_Kp = 0.1;   // Proportional gain of right motor
    float RM_Ki = 0.1;   // Integral gain of right motor
    float RM_Kd = 0.1;   // Derivative gain of right motor
    float LM_Kp = 0.1;   // Proportional gain of left motor
    float LM_Ki = 0.1;   // Integral gain of left motor
    float LM_Kd = 0.1;   // Derivative gain of left motor
    float pwm_max = 1.0; // Maximum PWM duty cycle (adjust as needed)
    float dt = 0.01;     // Time between control loops (adjust as needed)

    while (EnableMDB == 0)
    {
        wait(0.1);
    }
    motorStop();

    while (1)
    {
        PPS1 = Pulse1 * 100; // converting pulses in 10 ms to pulses per second
        PPS2 = Pulse2 * 100; // converting pulses in 10 ms to pulses per second

        float error_left = L_targetPPS - PPS1;
        float error_right = R_targetPPS - PPS2;

        // Get PID outputs
        float PID_output_left = PID_controller(error_left, integral_left, previous_error_left, dt, LM_Kp, LM_Ki, LM_Kd);
        float PID_output_right = PID_controller(error_right, integral_right, previous_error_right, dt, RM_Kp, RM_Ki, RM_Kd);

        // Adjust motor PWM duty cycles
        leftMotor.pwm = PID_output_left;
        rightMotor.pwm = PID_output_right;

        // Wait for a while
        wait(dt);
    }
}
