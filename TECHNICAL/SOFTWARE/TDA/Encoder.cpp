#include "mbed.h"
#include "C12832.h"
#include "QEI.h"



class Encoder {
public:
    QEI encoder;
    int pulses;
    int lastPulses;
    Encoder(PinName channelA, PinName channelB, int pulsesPerRev) : encoder(channelA, channelB, NC, pulsesPerRev) {
        pulses = 0;
        lastPulses = 0;
    }
    void doreset() {
        encoder.reset();
    }
    int returnPulses() {
        pulses = encoder.getPulses();
        return pulses;
    }
};


int Pulse1 = 0;
int Pulse2 = 0;
int Last_Pulse1 = 0;
int Last_Pulse2 = 0;
int RPM1 = 0;
int RPM2 = 0;

C12832 lcd(D11, D13, D12, D7, D10);

Encoder encoder1(PC_8, PC_6, 1024);
Encoder encoder2(PA_10, PB_3, 1024);

void NUM1() {
    Pulse1 = encoder1.returnPulses() - Last_Pulse1;
    Last_Pulse1 = encoder1.returnPulses();
}

void NUM2() {
    Pulse2 = encoder2.returnPulses() - Last_Pulse2;
    Last_Pulse2 = encoder2.returnPulses();
}

void readEncoders() {
    NUM1();
    NUM2();
}

void screen() {
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Pulses: %d %d\n", encoder1.returnPulses(), encoder2.returnPulses());
    lcd.locate(0, 10);
    lcd.printf("RPM: %d %d\n", RPM1, RPM2);
}

int main() {
    encoder1.doreset();
    encoder2.doreset();
    
    Ticker ticker;
    ticker.attach(&readEncoders, 0.1); // Call readEncoders every 0.1 seconds
    
    while (1) {
        RPM1 = Pulse1 * 60 / 1024;
        RPM2 = Pulse2 * 60 / 1024;
        screen();
        wait(0.2);
    }
}