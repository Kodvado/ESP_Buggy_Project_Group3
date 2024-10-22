#include "mbed.h"

InterruptIn FirePressed(D4);
DigitalOut EnableMDB(D8);   ///< Motor driver board enable
DigitalOut Bipolar1(PB_13); ///< Bipolar control 1
DigitalOut Bipolar2(PB_14); ///< Bipolar control 2

class RGB_LED
{
private:
    DigitalOut red;
    DigitalOut green;
    DigitalOut blue;

public:
    RGB_LED(PinName R, PinName G, PinName B) : red(R), green(G), blue(B) {}
    bool status;
    void color(bool r, bool g, bool b)
    {
        red = !r;
        green = !g;
        blue = !b;
        status = 1;
    }

    void greenOff() { green = 1; }

    void greenOn() { green = 0; }

    void greentoggle() { green = !green; }
    void bluetoggle() { blue = !blue; }
    void blueOn() { blue = 0; }
    void blueOff() { blue = 1; }
    void off()
    {
        red = 1;
        green = 1;
        blue = 1;
        status = 0;
    }
};

RGB_LED L(D5, D9, D8);

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

void enableMotorDriverBoard()
{
    EnableMDB = !EnableMDB;
}

unipolarmotor leftMotor(D9, D2);     ///< Left motor object
unipolarmotor rightMotor(D5, PA_13); ///< Right motor object

class MUSIC
{
protected:
    DigitalOut outputSignal;

public:
    MUSIC(PinName pin) : outputSignal(pin){};
    void playB(float l)
    {
        int cntr = l * 987.5;
        while (cntr != 0)
        {
            outputSignal = 1;
            wait(0.000506193);
            outputSignal = 0;
            wait(0.000506193);
            cntr -= 1;
        }
    }
    void playE(float l)
    {
        int cntr = l * 1318;
        while (cntr != 0)
        {
            outputSignal = 1;
            wait(0.000379215);
            outputSignal = 0;
            wait(0.000379215);
            cntr -= 1;
        }
    }
    void playD(float l)
    {
        int cntr = l * 1174;
        while (cntr != 0)
        {
            outputSignal = 1;
            wait(0.000425655);
            outputSignal = 0;
            wait(0.000425655);
            cntr -= 1;
        }
    }
    void playA(float l)
    {
        int cntr = l * 880;
        while (cntr != 0)
        {
            outputSignal = 1;
            wait(0.000568181);
            outputSignal = 0;
            wait(0.000568181);
            cntr -= 1;
        }
    }
};

int main(void)
{
    EnableMDB = 0;
    FirePressed.rise(&enableMotorDriverBoard);
    Bipolar1 = 0;
    Bipolar2 = 0;

    rightMotor.pwm = 1;
    leftMotor.pwm = 1;

    while (EnableMDB == 0)
    {
        wait(0.1);
    }
    rightMotor.dir = 1;
    leftMotor.dir = 0;
    rightMotor.pwm = 0;
    leftMotor.pwm = 0;

    while (1)
    {
        MUSIC darude(D6);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.2);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.2);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.05);
        wait(0.05);
        darude.playE(0.2);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.05);
        wait(0.05);
        darude.playD(0.2);
        wait(0.05);
        darude.playA(0.15);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.2);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.15);
        wait(0.05);
        darude.playE(0.175);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.2);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.05);
        wait(0.05);
        darude.playB(0.15);
        wait(0.05);
        darude.playD(0.175);
        wait(0.05);
    }
}