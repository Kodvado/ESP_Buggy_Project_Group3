/**
 * @file main.cpp
 * @brief Autonomous Line Follower Program
 */

#include "C12832.h"
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10);  ///< LCD display object
InterruptIn FirePressed(D4);        ///< Interrupt for motor driver board enable

/**
 * @class unipolarmotor
 * @brief Represents a unipolar motor with PWM control
 */
class unipolarmotor
{
public:
    PwmOut pwm;         ///< PWM output for motor speed control
    DigitalOut dir;     ///< Direction control for motor rotation

    /**
     * @brief Constructor to initialize the motor with PWM and direction pins
     * @param pwmPin Pin for PWM control
     * @param dirPin Pin for direction control
     */
    unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin)
    {
        pwm.period(0.00004);  // Set PWM period
        dir = 0;              // Initialize direction as default
    }

    /**
     * @brief Change the direction of the motor
     */
    void changeDirection()
    {
        dir = !dir;
    }
};

unipolarmotor leftMotor(D9, D2);    ///< Left motor object
unipolarmotor rightMotor(D5, PA_13);///< Right motor object

DigitalOut EnableMDB(D8);           ///< Motor driver board enable
DigitalOut Bipolar1(PB_13);         ///< Bipolar control 1
DigitalOut Bipolar2(PB_14);         ///< Bipolar control 2

int LeftPulse = 0;                  ///< Left encoder pulse count
int RightPulse = 0;                 ///< Right encoder pulse count
int Left_lastPulse = 0;             ///< Last recorded left encoder pulse
int Right_lastPulse = 0;            ///< Last recorded right encoder pulse
float L_defaultPPS = 3500;           ///< Target PPS for left motor
float L_targetPPS;
float R_defaultPPS = 3500;           ///< Target PPS for right motor
float R_targetPPS;
float L_defaultPWM = 0.70;          ///< Default PWM for left motor
float R_defaultPWM = 0.70;          ///< Default PWM for right motor
float vs1, vs2, vs3, vs4, vs5, vs6;  ///< Voltage readings from line sensors
float lineSensorError = 0.0;        ///< Error calculated from line sensor values
float positiveTurnLimit = 0.5;
float negativeTurnLimit = -0.5;

QEI leftEncoder(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);   ///< Left motor encoder
QEI rightEncoder(PC_2, PC_3, NC, 256, QEI::X4_ENCODING); ///< Right motor encoder

Ticker encoderTicker;               ///< Ticker for encoder update

/**
 * @brief Update encoder pulse counts and calculate speed
 */
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

AnalogIn lineSensor1(A0);   ///< Analog input for line sensor 1
AnalogIn lineSensor2(A1);   ///< Analog input for line sensor 2
AnalogIn lineSensor3(A2);   ///< Analog input for line sensor 3
AnalogIn lineSensor4(A3);   ///< Analog input for line sensor 4
AnalogIn lineSensor5(A4);   ///< Analog input for line sensor 5
AnalogIn lineSensor6(A5);   ///< Analog input for line sensor 6

/**
 * @brief Update line sensor readings
 */
void updateSensors()
{
    vs1 = lineSensor1.read();
    vs2 = lineSensor2.read();
    vs3 = lineSensor3.read();
    vs4 = lineSensor4.read();
    vs5 = lineSensor5.read();
    vs6 = lineSensor6.read();
}

/**
 * @brief Enable the motor driver board
 */
void enableMotorDriverBoard()
{
    EnableMDB = !EnableMDB;
}

/**
 * @brief Stop both motors
 */
void motorStop()
{
    leftMotor.pwm = 0;
    rightMotor.pwm = 0;
}

float integral_left = 0;          ///< Integral term for left motor PID controller
float previous_error_left = 0;    ///< Previous error for left motor PID controller

float integral_right = 0;         ///< Integral term for right motor PID controller
float previous_error_right = 0;   ///< Previous error for right motor PID controller


/**
 * @brief PID controller to compute motor control output
 * @param error Error value to control
 * @param integral Integral component of PID
 * @param previous_error Previous error for derivative component
 * @param dt Time interval
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @return PID output
 */
float PID_controller(float error, float &integral, float &previous_error, float dt, float Kp, float Ki, float Kd)
{
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
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

/**
 * @brief Main function
 * @return Program exit status
 */
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
    float PPSChange = 500;

    float currentLeftPPS, currentRightPPS, LeftM_error, RightM_error;

    // Wait for motor driver board activation
    while (EnableMDB == 0)
    {
        wait(0.1);
    }
    motorStop();  // Stop motors initially

    // Main control loop
    while (1)
    {
        updateSensors();

        lineSensorError = 3 * vs1 + 2 * vs2 + vs3 - vs4 - 2 * vs5 - 3 * vs6;

        if (lineSensorError > positiveTurnLimit)
        {
            L_targetPPS = L_defaultPPS - PPSChange;
            R_targetPPS = R_defaultPPS + PPSChange;
        }
        else if (lineSensorError < negativeTurnLimit)
        {
            L_targetPPS = L_defaultPPS + PPSChange;
            R_targetPPS = R_defaultPPS - PPSChange;
        }
        else
        {
            L_targetPPS = L_defaultPPS;
            R_targetPPS = R_defaultPPS;
        }

        updateEncoders();

        // Calculate current speeds in PPS (pulses per second)
        currentLeftPPS = LeftPulse * 100;
        currentRightPPS = RightPulse * 100;

        LeftM_error = L_targetPPS - currentLeftPPS;
        RightM_error = R_targetPPS - currentRightPPS;

        // Control motor speeds using PID controller
        leftMotor.pwm = PID_controller(LeftM_error, integral_left, previous_error_left, dt, LM_Kp, LM_Ki, LM_Kd);
        rightMotor.pwm = PID_controller(RightM_error, integral_right, previous_error_right, dt, RM_Kp, RM_Ki, RM_Kd);

        wait(dt); // Wait for a while
    }

    return 0;
}
