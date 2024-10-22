#include "C12832.h"
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10);
InterruptIn FirePressed(D4);

class unipolarmotor
{
public:
    PwmOut pwm;
    DigitalOut dir;

    unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin)
    {
        pwm.period(0.00004);
        dir = 0;
    }

    void changeDirection()
    {
        dir = !dir;
    }
};

unipolarmotor leftMotor(D9, D2);
unipolarmotor rightMotor(D5, PA_13);

DigitalOut EnableMDB(D8);
DigitalOut Bipolar1(PB_13);
DigitalOut Bipolar2(PB_14);

int LeftPulse = 0;
int RightPulse = 0;
int Left_lastPulse = 0;
int Right_lastPulse = 0;
float L_targetPPS = 3500;
float R_targetPPS = 3500;
float L_defaultPWM = 0.70;
float R_defaultPWM = 0.70; // make the PWMS the same speed on ground

float vs1, vs2, vs3, vs4, vs5, vs6; // voltages of the line sensors 1 through 6
float lineSensorError = 0.0;

QEI leftEncoder(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);
QEI rightEncoder(PC_2, PC_3, NC, 256, QEI::X4_ENCODING);

Ticker encoderTicker;

void updateEncoders()
{
    float temp1 = leftEncoder.getPulses();
    float temp2 = rightEncoder.getPulses();
    LeftPulse = temp1 - Left_lastPulse;
    Left_lastPulse = temp1;
    RightPulse = temp2 - Right_lastPulse;
    Right_lastPulse = temp2;
    if (LeftPulse > 50000 || RightPulse > 50000)
    {
        leftEncoder.reset();
        rightEncoder.reset();
        Left_lastPulse = 0;
        Right_lastPulse = 0;
    }
}

AnalogIn lineSensor1(A0);
AnalogIn lineSensor2(A1);
AnalogIn lineSensor3(A2);
AnalogIn lineSensor4(A3);
AnalogIn lineSensor5(A4);
AnalogIn lineSensor6(A5); // Temp Code (James pls add correct code here)

void updateSensors() // Temp Code (James pls add correct code here)
{
    vs1 = lineSensor1.read();
    vs2 = lineSensor2.read();
    vs3 = lineSensor3.read();
    vs4 = lineSensor4.read();
    vs5 = lineSensor5.read();
    vs6 = lineSensor6.read();
}

void enableMotorDriverBoard()
{
    EnableMDB = !EnableMDB;
}

void motorStop()
{
    leftMotor.pwm = 0;
    rightMotor.pwm = 0;
}

float integral_left = 0, previous_error_left = 0;
float integral_right = 0, previous_error_right = 0;

float PID_controller(float error, float &integral, float &previous_error, float dt, float Kp, float Ki, float Kd)
{
    integral += error * dt;
    float derivative = (error - previous_error)/dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    if (output > 1)
    {
        output = 1;
    }
    else if (output < 0)
    {
        output = 0;
    }
    return output;
}

int main()
{
    EnableMDB = 0;
    FirePressed.rise(&enableMotorDriverBoard);
    leftEncoder.reset();
    rightEncoder.reset();

    Bipolar1 = 0;
    Bipolar2 = 0;

    leftMotor.dir = 1;
    rightMotor.dir = 0;

    float RM_Kp = 0.1;   // Proportional gain of right motor
    float RM_Ki = 0.1;   // Integral gain of right motor
    float RM_Kd = 0.1;   // Derivative gain of right motor
    float LM_Kp = 0.1;   // Proportional gain of left motor
    float LM_Ki = 0.1;   // Integral gain of left motor
    float LM_Kd = 0.1;   // Derivative gain of left motor

    float dt = 0.01;
    float speed_threshold = 2000; // Adjust this threshold as needed

    float currentLeftPPS, currentRightPPS, LeftM_error, RightM_error;

    while (EnableMDB == 0)
    {
        wait(0.1);
    }
    motorStop();

    while (1)
    {
        updateSensors();

        lineSensorError = 3 * vs1 + 2 * vs2 + vs3 - vs4 - 2 * vs5 - 3 * vs6;

        if (lineSensorError > 0.5)
        {
            L_targetPPS = 3500 - 500;
            R_targetPPS = 3500 + 500;
        }
        else if (lineSensorError < -0.5)
        {
            L_targetPPS = 3500 + 500;
            R_targetPPS = 3500 - 500;
        }
        else
        {
            L_targetPPS = 3500;
            R_targetPPS = 3500;
        }

        updateEncoders();

        // Calculate current speeds in PPS (pulses per second)
        currentLeftPPS = LeftPulse * 100;
        currentRightPPS = RightPulse * 100;

        LeftM_error = L_targetPPS - currentLeftPPS;
        RightM_error = R_targetPPS - currentRightPPS;

        // Determine if motors need to be turned on or off based on the error from target speed
        leftMotor.pwm = PID_controller(LeftM_error, integral_left, previous_error_left, dt, LM_Kp, LM_Ki, LM_Kd);
        rightMotor.pwm = PID_controller(RightM_error, integral_right, previous_error_right, dt, RM_Kp, RM_Ki, RM_Kd);

        wait(dt); // Wait for a while
    }
}
