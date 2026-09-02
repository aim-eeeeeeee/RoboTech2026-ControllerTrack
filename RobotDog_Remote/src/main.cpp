#include <Arduino.h>
#include "RobotDog.h"

RobotDog dog;

void setup() {
    for(int i = 2; i <= 7; i++) {
        pinMode(i, OUTPUT);
    }
    digitalWrite(5, LOW);
    digitalWrite(7, LOW);
    Serial.begin(115200);
    delay(2000);
    dog.begin();
    dog.setControl(true, 0, 0, 0, 0, 0);
    delay(2000);
}

void loop() {
    /*
    static unsigned long time = 0;
    while(time < 10000) {
        dog.setControl(false, 1, 0, 0, 0, 0);
        dog.run();
        delay(10);
        time += 10;
    }
    //dog.setControl(false, 1, 0, 0, 0, 0);
    dog.LiftLeftFront();
    //dog.run();
    delay(1000);
    */
    dog.Roll();
    while(1);
}