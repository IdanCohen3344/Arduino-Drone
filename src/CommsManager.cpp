#include "CommsManager.h"

CommsManager::CommsManager(const char* ssid, const char* pass, unsigned int port) {
    this->ssid = ssid;
    this->pass = pass;
    this->localPort = port;
    
    throttle = 1000;
    pitch = 1500;
    roll = 1500;
    yaw = 1500;
    lastRecvTime = 0;
}

void CommsManager::begin() {
    delay(2000);
    Serial.print("Creating Access Point...");
    
    if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
        Serial.println("Creating access point failed");
        while (true); 
    }
    
    Serial.println(" AP Created!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); 
    
    Udp.begin(localPort);
    Serial.println("Listening for UDP packets from the App...");
}

void CommsManager::pollData() {
    int packetSize = Udp.parsePacket();
    
    if (packetSize) {
        int len = Udp.read(packetBuffer, 255);
        if (len > 0) {
            packetBuffer[len] = 0; 
        }
        
        if (sscanf(packetBuffer, "T:%d,P:%d,R:%d,Y:%d", &throttle, &pitch, &roll, &yaw) == 4) {
            lastRecvTime = millis(); 
            
            Serial.print("T: "); Serial.print(throttle);
            Serial.print(" | P: "); Serial.print(pitch);
            Serial.print(" | R: "); Serial.print(roll);
            Serial.print(" | Y: "); Serial.println(yaw);
        }
    }

    // Failsafe
    if (millis() - lastRecvTime > 500) {
        throttle = 1000; 
        pitch = 1500;    
        roll = 1500;
        yaw = 1500;
    }
}