/**
 * @file main.cpp
 * @brief Autonomous Line Follower Program
 */

#include "C12832.h"
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10); ///< LCD display object
InterruptIn FirePressed(D4);        ///< Interrupt for motor driver board enable

void PID_reset();

DigitalOut EnableMDB(D8);   ///< Motor driver board enable
DigitalOut Bipolar1(PB_13); ///< Bipolar control 1
DigitalOut Bipolar2(PB_14); ///< Bipolar control 2

volatile float LeftPulse = 0;       ///< Left encoder pulse count
volatile float RightPulse = 0;      ///< Right encoder pulse count
volatile float Left_lastPulse = 0;  ///< Last recorded left encoder pulse
volatile float Right_lastPulse = 0; ///< Last recorded right encoder pulse
volatile float L_defaultPPS = 4500; ///< Target PPS for left motor
volatile float L_targetPPS = 4000;
volatile float R_defaultPPS = 4500; ///< Target PPS for right motor
volatile float R_targetPPS = 4000;
volatile float L_PWM = 0.0; ///< Default PWM for left motor
volatile float R_PWM = 0.0; ///< Default PWM for right motor
volatile float L_PWM_Final = 0.4f;
volatile float R_PWM_Final = 0.4f;

volatile float vs1, vs2, vs3, vs4, vs5, vs6; ///< Voltage readings from line sensors
volatile float lineSensorError = 0.0;        ///< Error calculated from line sensor values
// volatile float positiveTurnLimit = 2;
// volatile float negativeTurnLimit = -2;

volatile float currentLeftPPS, currentRightPPS, LeftM_error, RightM_error;

char c;
char w;

volatile float RM_Kp = 0.0002; // Proportional gain of right motor
volatile float RM_Ki = 0;         // Integral gain of right motor
volatile float RM_Kd = 0.00000000;         // Derivative gain of right motor
volatile float LM_Kp = 0.0002;  // Proportional gain of left motor
volatile float LM_Ki = 0;         // Integral gain of left motor
volatile float LM_Kd = 0.00000000;         // Derivative gain of left motor

volatile float dt = 0.001;

volatile float integral_left = 0;       ///< Integral term for left motor PID controller
volatile float previous_error_left = 0; ///< Previous error for left motor PID controller

volatile float integral_right = 0;       ///< Integral term for right motor PID controller
volatile float previous_error_right = 0; ///< Previous error for right motor PID controller

volatile float S_Kp = 735; // Proportional gain of steering
volatile float S_Ki = 0;  // Integral gain of steering
volatile float S_Kd = 0.005;  // Derivative gain of steering

volatile float integral_steering = 0;       ///< Integral term for steering PID controller
volatile float previous_error_steering = 0; ///< Previous error for steering PID controller

volatile float S_PID_output;

Serial hm10(PA_11, PA_12); // TX, RX
Serial pc(USBTX, USBRX);   // TX, RX

int _180degree = 1350; // Number of pulses of each encoder to turn 180 degrees

QEI leftEncoder(PC_8, PC_6, NC, 256, QEI::X4_ENCODING); ///< Left motor encoder
QEI rightEncoder(A3, A2, NC, 256, QEI::X4_ENCODING);    ///< Right motor encoder


Ticker SCTicker; ///< Ticker for steering control

Ticker MCTicker; ///< Ticker for motor control

void steeringControl();

void motorControl();

/**
 * @brief Class representing a sensor array.
 */

void motorStop();

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

class SensorArray
{
protected:
    int stopval;
    bool readOff;
    bool syncstate;
    bool motorcontrolstate;
    bool steeringcontrolstate; 
    bool readStopState;                                                               ///< Flag indicating whether reading is turned off.
    int read3and3State;                                                         ///< State for error value reading.
    int singleReadState;                                                           ///< State for single reading.
    int readAllState;                                                              ///< State for reading all sensors.
    double sensor1Val, sensor2Val, sensor3Val, sensor4Val, sensor5Val, sensor6Val; ///< Current sensor values.
    double sample1Val, sample2Val, sample3Val, sample4Val, sample5Val, sample6Val; ///< Sample sensor values.
    Ticker getValue;                                                               ///< Timer for triggering readings.
    AnalogIn sensor1In, sensor2In, sensor3In, sensor4In, sensor5In, sensor6In;     ///< Analog input pins for sensors.
    DigitalOut LED1Out, LED2Out, LED3Out, LED4Out, LED5Out, LED6Out;  ///< Digital output pins for LEDs.
    double s1MaxVal, s2MaxVal, s3MaxVal, s4MaxVal, s5MaxVal, s6MaxVal;
    double s1MinVal, s2MinVal, s3MinVal, s4MinVal, s5MinVal, s6MinVal;

public:
    /**
     * @brief Constructor for SensorArray class.
     * @param s1-s6 Pin names for sensors.
     * @param l1-l6 Pin names for LEDs.
     */
    SensorArray(PinName s1, PinName s2, PinName s3, PinName s4, PinName s5, PinName s6,
                PinName l1, PinName l2, PinName l3, PinName l4, PinName l5, PinName l6)
        : sensor1In(s1), sensor2In(s2), sensor3In(s3), sensor4In(s4), sensor5In(s5), sensor6In(s6),
          LED1Out(l1), LED2Out(l2), LED3Out(l3), LED4Out(l4), LED5Out(l5), LED6Out(l6)
    {
        // Initialize variables and turn off LEDs
        stopval = 0;
        readOff = true;
        syncstate = false;
        readStopState = false;
        read3and3State = 1;
        singleReadState = 1;
        readAllState = 1;
        sLEDOff(1);
        sLEDOff(2);
        sLEDOff(3);
        sLEDOff(4);
        sLEDOff(5);
        sLEDOff(6);
        sensor1Val = 0;
        sensor2Val = 0;
        sensor3Val = 0;
        sensor4Val = 0;
        sensor5Val = 0;
        sensor6Val = 0;
        sample1Val = 0;
        sample2Val = 0;
        sample3Val = 0;
        sample4Val = 0;
        sample5Val = 0;
        sample6Val = 0;
        s1MaxVal = 1.1943;
        s2MaxVal = 1.0154;
        s3MaxVal = 1.1064;
        s4MaxVal = 1.0242;
        s5MaxVal = 1.0065;
        s6MaxVal = 0.7494;
        motorcontrolstate = false;
        s1MinVal = 0.117;
        s2MinVal = 0.106;
        s3MinVal = 0.121;
        s4MinVal = 0.097;
        s5MinVal = 0.090;
        s6MinVal = 0.058;
    };

    void motorControlOff(){
        motorcontrolstate = false;
    }   

    double getSample1Val()
    {
        return sample1Val;
    }

    double getSample2Val()
    {
        return sample2Val;
    }

    double getSample3Val()
    {
        return sample3Val;
    }

    double getSample4Val()
    {
        return sample4Val;
    }

    double getSample5Val()
    {
        return sample5Val;
    }

    double getSample6Val()
    {
        return sample6Val;
    }
    void steeringControlOff(){
        steeringcontrolstate = false;
    }
    void steeringControlOn(){
        steeringcontrolstate = true;
    }
    /**
     * @brief Get voltage reading from a sensor.
     * @param i Sensor index.
     * @return Voltage value.
     */
    double getSensorVolts(int i)
    {
        switch (i)
        {
        case 1:
            return sensor1In.read() * 3.3;
        case 2:
            return sensor2In.read() * 3.3;
        case 3:
            return sensor3In.read() * 3.3;
        case 4:
            return sensor4In.read() * 3.3;
        case 5:
            return sensor5In.read() * 3.3;
        case 6:
            return sensor6In.read() * 3.3;
        }
    }
    /**
     * @brief Turn on sensor LEDs.
     * @param i LED index.
     */
    void sLEDOn(int i)
    {
        switch (i)
        {
        case 1:
            LED1Out = 1;
            break;
        case 2:
            LED2Out = 1;
            break;
        case 3:
            LED3Out = 1;
            break;
        case 4:
            LED4Out = 1;
            break;
        case 5:
            LED5Out = 1;
            break;
        case 6:
            LED6Out = 1;
            break;
        }
    }

    void readsStopOff(){
        readStopState = false;
    }
    void readsStopOn(){
        readStopState = true;
    }

    /**
     * @brief Turn off sensor LEDs.
     * @param i LED index.
     */
    void sLEDOff(int i)
    {
        switch (i)
        {
        case 1:
            LED1Out = 0;
            break;
        case 2:
            LED2Out = 0;
            break;
        case 3:
            LED3Out = 0;
            break;
        case 4:
            LED4Out = 0;
            break;
        case 5:
            LED5Out = 0;
            break;
        case 6:
            LED6Out = 0;
            break;
        }
    }

    /**
     * @brief Toggle reading of sensors 3 at a time.
     */
    void toggle3and3Read(void)
    {
        if (readOff)
        {
            getValue.attach(callback(this, &SensorArray::read3and3), (dt/2));
            readOff = false;
        }
        else
        {
            getValue.detach();
            readOff = true;
        }
    }

    /**
     * @brief Toggle reading of sensors one at a time.
     */
    void toggleReadSingle(void)
    {
        if (readOff)
        {
            getValue.attach_us(callback(this, &SensorArray::singleRead), 500);
            readOff = false;
        }
        else
        {
            getValue.detach();
            readOff = true;
        }
    }

    void toggleReadAll(void)
    {
        if (readOff)
        {
            getValue.attach_us(callback(this, &SensorArray::readAll), 2000);
            readOff = false;
        }
        else
        {
            getValue.detach();
            readOff = true;
        }
    }

    void readAll(void)
    {
        switch (readAllState)
        {
        case 1:
            sLEDOn(1);
            sLEDOn(2);
            sLEDOn(3);
            sLEDOn(4);
            sLEDOn(5);
            sLEDOn(6);
            readAllState = 2;
            break;
        case 2:
            sample1Val = getSensorVolts(1);
            sample2Val = getSensorVolts(2);
            sample3Val = getSensorVolts(3);
            sample4Val = getSensorVolts(4);
            sample5Val = getSensorVolts(5);
            sample6Val = getSensorVolts(6);
            break;

        default:
            break;
        }
    }

    void toggleMotorControl(void)
    {
        motorStop();
        motorcontrolstate = !motorcontrolstate;
    }


    /**
     * @brief Reads the line displacement 3 sensors at once, called by the getValue Ticker.
     */
    void read3and3(void)
    {
        switch (read3and3State)
        {
        case 1:

            sLEDOn(1);
            sLEDOn(3);
            sLEDOn(5);
            read3and3State = 2;
            break;
        case 2:
            sensor1Val = getSensorVolts(1);
            sensor3Val = getSensorVolts(3);
            sensor5Val = getSensorVolts(5);
            sLEDOff(1);
            sLEDOff(3);
            sLEDOff(5);
            sLEDOn(2);
            sLEDOn(4);
            sLEDOn(6);
            if (syncstate){
                if (sensor1Val > s1MaxVal){s1MaxVal = sensor1Val;}
                if (sensor3Val > s3MaxVal){s3MaxVal = sensor3Val;}
                if (sensor5Val > s5MaxVal){s5MaxVal = sensor5Val;}
                if (sensor1Val < s1MinVal){s1MinVal = sensor1Val;}
                if (sensor3Val < s3MinVal){s3MinVal = sensor3Val;}
                if (sensor5Val < s5MinVal){s5MinVal = sensor5Val;}
            }
            read3and3State = 3;
            break;
        case 3:
            sensor2Val = getSensorVolts(2);
            sensor4Val = getSensorVolts(4);
            sensor6Val = getSensorVolts(6);
            sLEDOff(2);
            sLEDOff(4);
            sLEDOff(6);
            sLEDOn(1);
            sLEDOn(3);
            sLEDOn(5);
            if (syncstate){
                if (sensor2Val > s2MaxVal){s2MaxVal = sensor2Val;}
                if (sensor4Val > s4MaxVal){s4MaxVal = sensor4Val;}
                if (sensor6Val > s6MaxVal){s6MaxVal = sensor6Val;}
                if (sensor2Val < s2MinVal){s2MinVal = sensor2Val;}
                if (sensor4Val < s4MinVal){s4MinVal = sensor4Val;}
                if (sensor6Val < s6MinVal){s6MinVal = sensor6Val;}
            }
            read3and3State = 2;
            sample1Val = sensor1Val / s1MaxVal;
            sample2Val = sensor2Val / s2MaxVal;
            sample3Val = sensor3Val / s3MaxVal;
            sample4Val = sensor4Val / s4MaxVal;
            sample5Val = sensor5Val / s5MaxVal;
            sample6Val = sensor6Val / s6MaxVal;
            if ((readStopState) && (sample1Val < (s1MinVal + 0.2 )) && (sample2Val < (s2MinVal + 0.2 )) && (sample3Val < (s3MinVal + 0.2 )) && (sample4Val < (s4MinVal + 0.2 )) && (sample5Val < (s5MinVal + 0.2 )) && (sample6Val < (s6MinVal + 0.2 )))
            {
                stopval += 1;
                if (stopval > 49){
                    motorStop();
                    motorcontrolstate = false;
                }
            } else {
                stopval = 0;
            }
            if (steeringcontrolstate){
            steeringControl();
            }
            if (motorcontrolstate){
                motorControl();
            }
            break;
        };
    }

    /**
     * @brief Returns the value of displacement from the line
     * @return Error value.
     */
    double getErrorValue(void)
    {
        return ((5 * sample1Val) + (2.5 * sample2Val) + sample3Val) - (sample4Val + (2.5 * sample5Val)+(5 * sample6Val));
    }

    /**
     * @brief Reads the line displacement 1 sensor at a time, called by the getValue Ticker.
     */
    void singleRead(void)
    {
        switch (singleReadState)
        {
        case 1:
            sLEDOn(1);
            singleReadState = 2;
            break;
        case 2:
            sensor1Val = getSensorVolts(1);
            sLEDOff(1);
            sLEDOn(2);
            singleReadState = 3;
            break;
        case 3:
            sensor2Val = getSensorVolts(2);
            sLEDOff(2);
            sLEDOn(3);
            singleReadState = 4;
            break;
        case 4:
            sensor3Val = getSensorVolts(3);
            sLEDOff(3);
            sLEDOn(4);
            singleReadState = 5;
            break;
        case 5:
            sensor4Val = getSensorVolts(4);
            sLEDOff(4);
            sLEDOn(5);
            singleReadState = 6;
            break;
        case 6:
            sensor5Val = getSensorVolts(5);
            sLEDOff(5);
            sLEDOn(6);
            singleReadState = 7;
            break;
        case 7:
            sensor6Val = getSensorVolts(6);
            sLEDOff(6);
            sLEDOn(1);
            singleReadState = 2;
            sample1Val = sensor1Val;
            sample2Val = sensor2Val;
            sample3Val = sensor3Val;
            sample4Val = sensor4Val;
            sample5Val = sensor5Val;
            sample6Val = sensor6Val;
            break;
        }
        return;
    };

    void sensorSync(){
        if (syncstate){syncstate = false;} else {syncstate = true;}
        return;
    };

    double getMaxVal(int i){
        switch (i)
        {
        case 1:
            return s1MaxVal;
            break;
        case 2:
            return s2MaxVal;
            break;
        case 3:
            return s3MaxVal;
            break;
        case 4:
            return s4MaxVal;
            break;
        case 5:
            return s5MaxVal;
            break;
        case 6:
            return s6MaxVal;
            break;
        };
    }
    float getMinVal(int i){
        switch (i)
        {
        case 1:
            return s1MinVal;
            break;
        case 2:
            return s2MinVal;
            break;
        case 3:
            return s3MinVal;
            break;
        case 4:
            return s4MinVal;
            break;
        case 5:
            return s5MinVal;
            break;
        case 6:
            return s6MinVal;
            break;
        };
    }

    void turnaround()   
        {
            motorStop();
            readsStopOff();
            steeringControlOff();
            PID_reset();
            leftMotor.dir = !leftMotor.dir;
            L_targetPPS = 1500;
            R_targetPPS = 1500;
            while (sample6Val < 0.4)
            {
                wait_us(1000);
            }                                       // Set the PWM to 0.5
            while (sample3Val < 0.6)
            {
                wait_us(1000);
            }
            leftMotor.dir = !leftMotor.dir;
            motorStop();
            PID_reset();
            steeringControlOn();
            readsStopOn();
        }

};


SensorArray sennyArray(PC_0, PC_2, PC_3, PC_5, PC_4, PB_1, PC_11, PB_7, PA_15, PA_14, PC_12, PC_10);




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
    LeftPulse = abs(LeftPulse);
    RightPulse = abs(RightPulse);
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


void motorStart()
{
    leftMotor.pwm = 0.7;
    rightMotor.pwm = 0.7;
    L_PWM = 0.7;
    R_PWM = 0.7;
}

void readEncoders() // Function to read the encoders
{
    LeftPulse = leftEncoder.getPulses();
    RightPulse = rightEncoder.getPulses();
}

// void turnAround() // Function to turn around
// {
//     sennyArray.readsStopOff();
//     leftEncoder.reset();
//     rightEncoder.reset();

//     readEncoders();
//     leftMotor.dir = !leftMotor.dir;                             // Change the direction of the right motor
//     leftMotor.pwm = 0.5;                                        // Set the PWM to 0.5
//     rightMotor.pwm = 0.5;                                       // Set the PWM to 0.5
//     while (LeftPulse <= _180degree && RightPulse <= _180degree) // 374 is the number of pulses to turn around
//     {
//         readEncoders();
//     }

//     leftMotor.dir = !leftMotor.dir; // Change the direction of the left motor

//     leftEncoder.reset();
//     rightEncoder.reset();
//     LeftPulse = 0;
//     RightPulse = 0;
//     Left_lastPulse = 0;
//     Right_lastPulse = 0;
//     sennyArray.readsStopOn();
// }

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

void PID_reset()
{
    integral_left = 0;
    previous_error_left = 0;
    integral_right = 0;
    previous_error_right = 0;
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

void steeringControl()
{
    S_PID_output = PID_controller(sennyArray.getErrorValue(), integral_steering, previous_error_steering, dt, S_Kp, S_Ki, S_Kd);

    // if (S_PID_output > 0)
    // {
    //     L_targetPPS = L_defaultPPS + S_PID_output;
    //     R_targetPPS = R_defaultPPS - S_PID_output;
    // }
    // else if (S_PID_output < 0)
    // {
    //     L_targetPPS = L_defaultPPS - 0.5 * S_PID_output;
    //     R_targetPPS = R_defaultPPS + 0.5 * S_PID_output;
    // }
    // else
    // {
    //     L_targetPPS = L_defaultPPS;
    //     R_targetPPS = R_defaultPPS;
    // }
    // I dont think this is necessary
    L_targetPPS = L_defaultPPS - S_PID_output;
    R_targetPPS = R_defaultPPS + S_PID_output;
}
void serial_config()
{
    if (pc.readable())
    {
        w = pc.getc();
        hm10.putc(w);
    }
}

/**
 * @brief Main function
 * @return Program exit status
 */
int main()
{
    EnableMDB = 0;
    leftEncoder.reset();
    rightEncoder.reset();
    //FirePressed.rise(&enableMotorDriverBoard);
    Bipolar1 = 0;
    Bipolar2 = 0;

    leftMotor.dir = 0;
    rightMotor.dir = 0;

    leftMotor.pwm = 1;
    rightMotor.pwm = 1;

    sennyArray.toggle3and3Read();

    // Wait for motor driver board activation

    while (!hm10.readable())
    {
        wait(0.1);
    }

    motorStop(); // Stop motors initially

    // Main control loop
    while (1)
    {
        if (hm10.readable())
        {
            c = hm10.getc();
            pc.putc(c);

            switch (c)
            {
            case 'T':
                sennyArray.turnaround();
                break;

            case 'S':
                EnableMDB = 1;
                motorStop();
                sennyArray.toggleMotorControl();
                sennyArray.readsStopOn();
                sennyArray.steeringControlOn();
                //MCTicker.attach(&motorControl, dt);
                // SCTicker.attach(&steeringControl, 0.05);

                break;

            case 'H':
                EnableMDB = 0;
                motorStop();
                sennyArray.motorControlOff();
                break;

            case 'I':
                S_Kp += 5;
                hm10.printf("S_Kp: %f\n", S_Kp);
                /*RM_Kp += 0.000001;
                LM_Kp += 0.000001;
                hm10.printf("RM_Kp: %f\n", RM_Kp);
                hm10.printf("LM_Kp: %f\n", LM_Kp);*/
                break;

            case 'D':
                S_Kp -= 5;
                hm10.printf("S_Kp: %f\n", S_Kp);
                /*RM_Kp -= 0.000001;
                LM_Kp -= 0.000001;
                hm10.printf("RM_Kp: %f\n", RM_Kp);
                hm10.printf("LM_Kp: %f\n", LM_Kp);*/
                break;
            
            case 'X':
                hm10.printf("S1 : %f\n", sennyArray.getSample1Val());
                hm10.printf("S2 : %f\n", sennyArray.getSample2Val());
                hm10.printf("S3 : %f\n", sennyArray.getSample3Val());
                hm10.printf("S4 : %f\n", sennyArray.getSample4Val());
                hm10.printf("S5 : %f\n", sennyArray.getSample5Val());
                hm10.printf("S6 : %f\n", sennyArray.getSample6Val());
                break;
            case 'Z':
                sennyArray.sensorSync();
                break;
            case 'Y':
                hm10.printf("S1 Max : %f\n", sennyArray.getMaxVal(1));
                hm10.printf("S2 Max : %f\n", sennyArray.getMaxVal(2));
                hm10.printf("S3 Max : %f\n", sennyArray.getMaxVal(3));
                hm10.printf("S4 Max : %f\n", sennyArray.getMaxVal(4));
                hm10.printf("S5 Max : %f\n", sennyArray.getMaxVal(5));
                hm10.printf("S6 Max : %f\n", sennyArray.getMaxVal(6));
                hm10.printf("S1 Min : %f\n", sennyArray.getMinVal(1));
                hm10.printf("S2 Min : %f\n", sennyArray.getMinVal(2));
                hm10.printf("S3 Min : %f\n", sennyArray.getMinVal(3));
                hm10.printf("S4 Min : %f\n", sennyArray.getMinVal(4));
                hm10.printf("S5 Min : %f\n", sennyArray.getMinVal(5));
                hm10.printf("S6 Min : %f\n", sennyArray.getMinVal(6));
                break;
            case 'A':
                S_Kd += 0.0001;
                hm10.printf("S_Kd: %f\n", S_Kd);
                break;
            case 'B':
                S_Kd -= 0.0001;
                hm10.printf("S_Kd: %f\n", S_Kd);
                break;
            case 'E':
                R_targetPPS += 100;
                L_targetPPS += 100;
                hm10.printf("L_targetPPS: %f\nR_targetPPS: %f\n", L_targetPPS, R_targetPPS);
                break;
            case 'F':
                R_targetPPS -= 100;
                L_targetPPS -= 100;
                hm10.printf("L_targetPPS: %f\nR_targetPPS: %f\n", L_targetPPS, R_targetPPS);
                break;
            case 'M':
                S_Ki += 0.0001;
                hm10.printf("S_Ki: %f\n", S_Ki);
                break;
            case 'N':
                S_Ki -= 0.0001;
                hm10.printf("S_Ki: %f\n", S_Ki);
                break;
            default:
                break;
            }

            serial_config();
        }

        // lcd.locate(0, 0);
        // lcd.printf("S1: %f", sennyArray.getSample1Val());
        // lcd.locate(0, 10);
        // lcd.printf("S2: %f", sennyArray.getSample2Val());
        // lcd.locate(0, 20);
        // lcd.printf("S3: %f", sennyArray.getSample3Val());
        // lcd.locate(64, 0);
        // lcd.printf("S4: %f", sennyArray.getSample4Val());
        // lcd.locate(64, 10);
        // lcd.printf("S5: %f", sennyArray.getSample5Val());
        // lcd.locate(64, 20);
        // lcd.printf("S6: %f", sennyArray.getSample6Val());


    }
    return 0;
}
