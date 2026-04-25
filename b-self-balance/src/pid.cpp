#include "pid.hpp"

PID::PID(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0f), prevError_(0.0f), lastTimeUs_(0) {}

float PID::compute(float setpoint, float input) {
    unsigned long now = micros();
    float dt = (now - lastTimeUs_) / 1000000.0f;
    
    // 首次保护
    if (lastTimeUs_ == 0 || dt <= 0 || dt > 0.5f) {
        lastTimeUs_ = now;
        prevError_ = setpoint - input;
        return kp_ * prevError_;
    }
    
    float error = setpoint - input;
    
    integral_ += error * dt;
    integral_ = constrain(integral_, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float derivative = (error - prevError_) / dt;
    
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    prevError_ = error;
    lastTimeUs_ = now;
    
    return output;
}

void PID::reset() {
    integral_ = 0.0f;
    prevError_ = 0.0f;
    lastTimeUs_ = 0;
}

void PID::setKp(float kp) { kp_ = kp; }
void PID::setKi(float ki) { ki_ = ki; }
void PID::setKd(float kd) { kd_ = kd; }