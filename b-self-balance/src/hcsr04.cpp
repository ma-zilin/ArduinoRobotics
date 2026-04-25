#include "hcsr04.hpp"

HCSR04::HCSR04(uint8_t trigPin, uint8_t echoPin)
    : trig_(trigPin), echo_(echoPin), maxDistCm_(400.0f) {
    pinMode(trig_, OUTPUT);
    digitalWrite(trig_, LOW);
    pinMode(echo_, INPUT);
}

void HCSR04::setMaxDistance(float cm) {
    maxDistCm_ = cm;
}

float HCSR04::getDistance() {
    digitalWrite(trig_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_, LOW);
    
    long duration = pulseIn(echo_, HIGH, 30000);
    
    if (duration == 0) return 999.0f;
    
    float dist = duration / 58.0f;
    if (dist > maxDistCm_) return 999.0f;
    
    return dist;
}