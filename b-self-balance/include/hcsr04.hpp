#ifndef HCSR04_HPP
#define HCSR04_HPP

#include <Arduino.h>

class HCSR04 {
public:
    HCSR04(uint8_t trigPin, uint8_t echoPin);
    
    float getDistance();      // 返回 cm，失败返回 999.0
    void setMaxDistance(float cm);

private:
    uint8_t trig_, echo_;
    float maxDistCm_;
};

#endif