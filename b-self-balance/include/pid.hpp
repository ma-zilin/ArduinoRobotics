#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>

class PID {
public:
    PID(float kp, float ki, float kd);
    
    float compute(float setpoint, float input);
    void reset();
    
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);

private:
    float kp_, ki_, kd_;
    float integral_;
    float prevError_;
    unsigned long lastTimeUs_;
    static constexpr float INTEGRAL_LIMIT = 50.0f;
};

#endif