#include "mbed.h"
#include "C12832.h"

class SensorArray
{
protected:
    bool readOff;
    bool readValuesReady;
    int readErrorValueState;
    double sensor1Val, sensor2Val, sensor3Val, sensor4Val, sensor5Val, sensor6Val;
    Ticker getValue;
    AnalogIn sensor1In, sensor2In, sensor3In, sensor4In, sensor5In, sensor6In;
    DigitalOut LED1Out, LED2Out, LED3Out, LED4Out, LED5Out, LED6Out;
public:
    SensorArray(PinName s1, PinName s2, PinName s3, PinName s4, PinName s5, PinName s6, PinName l1,
     PinName l2, PinName l3, PinName l4, PinName l5, PinName l6) : sensor1In(s1), sensor2In(s2), sensor3In(s3), 
     sensor4In(s4), sensor5In(s5), sensor6In(s6), LED1Out(l1), LED2Out(l2), LED3Out(l3), LED4Out(l4), LED5Out(l5), LED6Out(l6) {
        readOff = true;
        readErrorValueState = 1;
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
        readValuesReady = false;
    };
    double getSensorVolts(int i){
        switch (i) {
        case 1:
            return sensor1In.read()*3.3;
            break;
        case 2:
            return sensor2In.read()*3.3;
            break;
        case 3:
            return sensor3In.read()*3.3;
            break;
        case 4:
            return sensor4In.read()*3.3;
            break;
        case 5:
            return sensor5In.read()*3.3;
            break;
        case 6:
            return sensor6In.read()*3.3;
            break;
        }
    }

    void sLEDOn(int i){
        switch (i) {
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
    void sLEDOff(int i){
        switch (i) {
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
    void toggleRead(void){
        if (readOff){
            getValue.attach_us(callback(this, &SensorArray::readErrorValue), 150000);
            readOff = false;
        } else {
            getValue.detach();
            readOff = true;
        }
    }
    void readErrorValue(void){
        switch (readErrorValueState){
        case 1:
            sLEDOn(1);
            sLEDOn(3);
            sLEDOn(5);
            readErrorValueState = 2;
            break;
        case 2:
            sensor1Val = getSensorVolts(1);
            sensor3Val = getSensorVolts(3);
            sensor5Val = getSensorVolts(5);
            sLEDOff(1);
            sLEDOff(3);
            sLEDOff(5);
            readErrorValueState = 3;
            break;
        case 3:
            sLEDOn(2);
            sLEDOn(4);
            sLEDOn(6);
            readErrorValueState = 4;
            break;
        case 4:
            sensor2Val = getSensorVolts(2);
            sensor4Val = getSensorVolts(4);
            sensor6Val = getSensorVolts(6);
            sLEDOff(2);
            sLEDOff(4);
            sLEDOff(6);
            readErrorValueState = 1;
            readValuesReady = true;
            break;
        }
    }
    double getErrorValue(void){
        if(readValuesReady){
            return ((3*sensor1Val)+(2*sensor2Val)+sensor3Val)-(sensor4Val+(2*sensor5Val)+(3*sensor6Val));
        }
        return 333.33;
    }
};

C12832 lcd(D11, D13, D12, D7, D10);

int main(void){
    SensorArray sennyArray(A0, A1, A2, A3, A4, A5, D10, D7, D2, D3, D4, D5);
    lcd.cls();
    sennyArray.toggleRead();
    while(1){
        lcd.locate(0, 10);
        lcd.printf("%.5f", sennyArray.getErrorValue());
    };
};
