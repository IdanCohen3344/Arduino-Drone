#include <Arduino.h>
#include "CommsManager.h"

CommsManager comms("Drone_R4", "12345678", 4210);

void setup() {
    Serial.begin(115200);
    comms.begin(); 
}

void loop() {
    comms.pollData();  
}