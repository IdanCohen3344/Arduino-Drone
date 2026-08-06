#pragma once

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

class CommsManager {
private:
    const char* ssid;
    const char* pass;
    unsigned int localPort;
    char packetBuffer[255];
    WiFiUDP Udp;
    unsigned long lastRecvTime;

    int throttle;
    int pitch;
    int roll;
    int yaw;

public:
    CommsManager(const char* ssid, const char* pass, unsigned int port);
    
    void begin();
    void pollData();

    int getThrottle() { 
        return throttle; 
    }
    int getPitch() {
         return pitch;
         }
    int getRoll() {
         return roll; 
        }
    int getYaw() { 
        return yaw; 
    }
};