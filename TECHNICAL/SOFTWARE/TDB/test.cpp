#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
// #include "mbed2/299/TARGET_NUCLEO_F401RE/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F401RE/PeripheralNames.h"
// #include "mbed2/299/TARGET_NUCLEO_F401RE/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F401RE/PinNames.h"

#define M_PI 3.14159265358979323846f // Definition of π

AnalogIn sensorFarLeft(PB_1);
AnalogIn sensorLeft(PC_4);
AnalogIn sensorCenter(PC_2);
AnalogIn sensorRight(PC_3);
AnalogIn sensorFarRight(PC_5);

DigitalOut sensorDAFarLeft(PA_15);
DigitalOut sensorDALeft(PA_13);
DigitalOut sensorDACenter(PC_1);
DigitalOut sensorDARight(PA_14);
DigitalOut sensorDAFarRight(PB_12);

DigitalOut bipolar1(PC_8);    // JP1a Pin 1
DigitalOut direction1(PB_15); // JP1a Pin 2
DigitalOut pwm1(PB_14);       // JP1a Pin 3
DigitalOut bipolar2(PB_7);    // JP1a Pin 4
DigitalOut direction2(PB_2);  // JP1a Pin 5
DigitalOut pwm2(PB_13);       // JP1a Pin 6
DigitalOut enable(PC_0);      // JP1a Pin 7

QEI encoderRight(PC_6, PA_13, NC, 512); // Right encoder
QEI encoderLeft(PC_11, PD_2, NC, 512);

// bang bang parameters
float Kp = 0.5f;
float threshold = 0.5f;
float defaultSpeed = 0.5f;

// Motor Speeds
float straightSpeed = 0.6;
float turnSpeed = 0.3;
float leftWheelTarget;
float rightWheelTarget;
float leftMotorSpeed, rightMotorSpeed;

// BLE variables
volatile char s;
char w;

Timer dt; // Timer
Ticker ticker1;

class Sensor // Begin potentiometer class definition
{
private:
    AnalogIn inputSignal;
    int Count;
    float VDD, currentSampleNorm, currentSampleVolts;

public:
    Sensor(PinName pin, float v) : inputSignal(pin), VDD(v), Count(0) {} // Constructor with two arguments
    float amplitudeVolts(void)
    {
        return (inputSignal.read() * VDD);
    }
    float amplitudeNorm(void)
    {
        return inputSignal.read();
    }
    void sample(void)
    {
        currentSampleNorm = inputSignal.read();
        currentSampleVolts = currentSampleNorm * VDD;
        // Readings[Count] = currentSampleVolts;
    }
    float getCurrentSampleVolts(void)
    {
        return currentSampleVolts;
    }
    float getCurrentSampleNorm(void)
    {
        return currentSampleNorm;
    }
};
class SamplingSensor : public Sensor
{
private:
    float samplingFrequency, samplingPeriod;
    Ticker sampler;

public:
    SamplingSensor(PinName pin, float v, float fs) : Sensor(pin, v), samplingFrequency(fs) // Constructor matches the base class constructor
    {
        samplingPeriod = 1.0f / samplingFrequency;
        sampler.attach(callback(this, &SamplingSensor::sample), samplingPeriod); // Corrected the callback to SamplingSensor
    }
};

SamplingSensor S1(PB_1, 2.0, 200); // Initialize S1 with count 0
SamplingSensor S2(PC_4, 2.0, 200); // Initialize S2 with count 1
SamplingSensor S3(PC_3, 2.0, 200); // Initialize S3 with count 2
SamplingSensor S4(PC_5, 2.0, 200); // Initialize S4 with count 3

float getLeftEncoderSpeed(void)
{
    int leftPulses;
    float leftTickRate;
    static float previousLeftPulses = 0.0f;
    leftPulses = encoderLeft.getPulses();
    leftTickRate = (leftPulses - previousLeftPulses) / dt;
    return leftTickRate / 10.0f;
}

float getRightEncoderSpeed(void)
{
    int rightPulses;
    float rightTickRate;
    static float previousRightPulses = 0.0f;
    rightPulses = encoderRight.getPulses();
    rightTickRate = (rightPulses - previousRightPulses) / dt;
    return rightTickRate / 10.0f;
}

void speedControl()
{
    if (getLeftEncoderSpeed() < leftWheelTarget)
    {
        leftMotorSpeed = leftMotorSpeed + 0.01f;
    }
    else
    {
        leftMotorSpeed = leftMotorSpeed - 0.01f;
    }
    if (getRightEncoderSpeed() < rightWheelTarget)
    {
        rightMotorSpeed = rightMotorSpeed + 0.01f;
    }
    else
    {
        rightMotorSpeed = rightMotorSpeed - 0.01f;
    }

    pwm1.write(leftMotorSpeed);
    pwm2.write(rightMotorSpeed);
}

void BangBangControl(float sensorval1, float sensorval2, float sensorval3, float sensorval4)
{

    if (sensorval1 >= 1.0 && sensorval1 <= 3.0)
    {
        leftWheelTarget = 40.0f;
        rightWheelTarget = 30.0f;
    }
    else if (sensorval2 >= 1.0 && sensorval2 <= 3.0)
    {
        leftWheelTarget = 30.0f;
        rightWheelTarget = 20.0f;
    }
    else if (sensorval3 >= 0.4 && sensorval3 <= 1.0)
    {
        leftWheelTarget = 20.0f;
        rightWheelTarget = 30.0f;
    }
    else if (sensorval4 >= 0.4 && sensorval4 <= 1.0)
    {
        leftWheelTarget = 30.0f;
        rightWheelTarget = 40.0f;
    }
}

void readSensorVal()
{
    float w = S1.getCurrentSampleVolts();
    float x = S2.getCurrentSampleVolts();
    float y = S3.getCurrentSampleVolts();
    float z = S4.getCurrentSampleVolts();
    // float error = (sensorval1 + sensorval2) - (sensorval3 + sensorval4);
    BangBangControl(w, x, y, z);
}


int main(void)
{
    sensorDAFarLeft = 1;
    sensorDALeft = 1;
    sensorDARight = 1;
    sensorDAFarRight = 1;

    enable = 1;
    bipolar1 = 0;
    bipolar2 = 0;
    direction1 = 1;
    direction2 = 1;
    dt.start();

    ticker1.attach(&readSensorVal, 0.001);

    while (1)
    {
        speedControl();
        wait(0.1);
    }
}