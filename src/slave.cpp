#include <Arduino.h>
#include <Wire.h>

int sensorValues[] = {0, 0, 0, 0, 0, 0, 0, 0};
int sensorPorts[] = {0, 1, 2, 3, 4, 5, 6, 7};

void setup(){
    Wire.begin(1);
    Wire.onRequest();
    for(auto i: sensorPorts){
        pinMode(i, INPUT);
    }
}



void loop(){
    while (1){
        for(int i = 0; i < sizeof(sensorPorts)/sizeof(sensorPorts[0]); i++){
            sensorValues[i] = analogRead(sensorPorts[i]);
        }
    }
    
}

void
