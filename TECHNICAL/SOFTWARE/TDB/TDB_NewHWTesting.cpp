/**
 * @file main.cpp
 * @brief Autonomous Line Follower Program
 */

#include "C12832.h"
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10); ///< LCD display object
InterruptIn FirePressed(D4);        ///< Interrupt for motor driver board enable

/**
 * @class unipolarmotor
 * @brief Represents a unipolar motor with PWM control
 */
class unipolarmotor
{
public:
    PwmOut pwm;     ///< PWM output for motor speed control
    DigitalOut dir; ///< Direction control for motor rotation
    float pwmValue; ///< PWM value for motor speed control

    /**
     * @brief Constructor to initialize the motor with PWM and direction pins
     * @param pwmPin Pin for PWM control
     * @param dirPin Pin for direction control
     */
    unipolarmotor(PinName pwmPin, PinName dirPin) : pwm(pwmPin), dir(dirPin)
    {
        pwm.period(0.00004); // Set PWM period
        dir = 0;             // Initialize direction as default
    }

    /**
     * @brief Change the direction of the motor
     */
    void changeDirection()
    {
        dir = !dir;
    }

};

unipolarmotor leftMotor(D9, D2);     ///< Left motor object
unipolarmotor rightMotor(D5, PA_13); ///< Right motor object

DigitalOut EnableMDB(D8);   ///< Motor driver board enable
DigitalOut Bipolar1(PB_13); ///< Bipolar control 1
DigitalOut Bipolar2(PB_14); ///< Bipolar control 2

volatile float LeftPulse = 0;         ///< Left encoder pulse count
volatile float RightPulse = 0;        ///< Right encoder pulse count
volatile float Left_lastPulse = 0;    ///< Last recorded left encoder pulse
volatile float Right_lastPulse = 0;   ///< Last recorded right encoder pulse
volatile float L_defaultPPS = 1500; ///< Target PPS for left motor
volatile float L_targetPPS = 2000;
volatile float R_defaultPPS = 1500; ///< Target PPS for right motor
volatile float R_targetPPS = 2000;
volatile float L_PWM = 0.0;                  ///< Default PWM for left motor
volatile float R_PWM = 0.0;                  ///< Default PWM for right motor
volatile float L_PWM_Final = 0.0f;
volatile float R_PWM_Final = 0.0f;
// volatile float positiveTurnLimit = 2;
// volatile float negativeTurnLimit = -2;

volatile float currentLeftPPS, currentRightPPS, LeftM_error, RightM_error;

char c;
char w;

volatile float RM_Kp = 0.0000095;    // Proportional gain of right motor
volatile float RM_Ki = 0; // Integral gain of right motor
volatile float RM_Kd = 0;           // Derivative gain of right motor
volatile float LM_Kp = 0.000012;    // Proportional gain of left motor
volatile float LM_Ki = 0; // Integral gain of left motor
volatile float LM_Kd = 0;      // Derivative gain of left motor

volatile float dt = 0.01;

volatile float integral_left = 0;       ///< Integral term for left motor PID controller
volatile float previous_error_left = 0; ///< Previous error for left motor PID controller

volatile float integral_right = 0;       ///< Integral term for right motor PID controller
volatile float previous_error_right = 0; ///< Previous error for right motor PID controller

int _180degree = 1350; // Number of pulses of each encoder to turn 180 degrees

QEI leftEncoder(PC_8, PC_6, NC, 256, QEI::X4_ENCODING);  ///< Left motor encoder
QEI rightEncoder(A5, A4, NC, 256, QEI::X4_ENCODING); ///< Right motor encoder

Ticker MCTicker; ///< Ticker for motor control

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
    if (temp1 > 500000 || temp2 > 500000)
    {
        leftEncoder.reset();
        rightEncoder.reset();
        Left_lastPulse = 0;
        Right_lastPulse = 0;
    }
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
    leftMotor.pwm = 1;
    rightMotor.pwm = 1;
}


void readEncoders() // Function to read the encoders
{
    LeftPulse = leftEncoder.getPulses();
    RightPulse = rightEncoder.getPulses();
}


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
float PID_controller(float error, float integral, float previous_error, float dt, float Kp, float Ki, float Kd)
{
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    return output;
}


void calculateSpeed()
{
    updateEncoders();
    currentLeftPPS = LeftPulse / dt;
    currentRightPPS = RightPulse / dt;
}

void motorControl()
{
    calculateSpeed();
    LeftM_error = L_targetPPS - currentLeftPPS;
    RightM_error = R_targetPPS - currentRightPPS;

    L_PWM += PID_controller(LeftM_error, integral_left, previous_error_left, dt, LM_Kp, LM_Ki, LM_Kd);
    R_PWM += PID_controller(RightM_error, integral_right, previous_error_right, dt, RM_Kp, RM_Ki, RM_Kd);

    if (L_PWM > 1)
    {
        L_PWM = 1;
    }
    else if (L_PWM < 0)
    {
        L_PWM = 0;
    }
    if (R_PWM > 1)
    {
        R_PWM = 1;
    }
    else if (R_PWM < 0)
    {
        R_PWM = 0;
    }

    L_PWM_Final = 1.0f - L_PWM;
    R_PWM_Final = 1.0f - R_PWM;

    leftMotor.pwm = L_PWM_Final;
    rightMotor.pwm = R_PWM_Final;

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
    rightMotor.dir = 1;

    leftMotor.pwm = 1;
    rightMotor.pwm = 1;



    //sennyArray.toggle3and3Read();

    // Wait for motor driver board activation
    while (EnableMDB == 0)
    {
        wait(0.1);
    }


    motorStop(); // Stop motors initially
    MCTicker.attach(&motorControl, dt); // Attach motor control function to ticker

    // Main control loop
    while (1)
    {


        lcd.locate(0, 0);
        lcd.printf("R PWM: %f", currentLeftPPS);
        lcd.locate(0, 10);
        lcd.printf("L PWM: %f", L_PWM);
        lcd.locate(0, 20);
        lcd.printf("R PPS: %g", LeftPulse);
    }

    return 0;
}
