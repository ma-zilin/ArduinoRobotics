#include <Arduino.h>
#include <Servo.h>
#include "mpu6500.hpp"
#include "pid.hpp"
#include "hcsr04.hpp"

MPU6500 mpu;
PID pid(2.0f, 0.0f, 0.0f);
HCSR04 us(11, 12);
Servo servo;

const float TARGET_LEVEL = 90.0f;
const float TARGET_WARN  = 120.0f;
const unsigned long LOOP_US = 20000;

void setup() {
    Serial.begin(115200);
    
    Serial.println("Init...");
    
    mpu.begin();
    Serial.print("WHO_AM_I: 0x");
    Serial.println(mpu.getWhoAmI(), HEX);
    
    servo.attach(9);
    servo.write(TARGET_LEVEL);
    
    Serial.println("Init done. Starting...");
    delay(1000);
}

void loop() {
    static unsigned long lastMicros = 0;
    if (micros() - lastMicros < LOOP_US) return;
    lastMicros += LOOP_US;
    
    mpu.update();
    float pitch = mpu.getPitch();
    
    static uint8_t usCounter = 0;
    static float distance = 999.0f;
    if (++usCounter >= 5) {
        usCounter = 0;
        distance = us.getDistance();
    }
    
    float target = (distance < 20.0f && distance > 0) ? TARGET_WARN : TARGET_LEVEL;
    float output = pid.compute(target, pitch);
    output = constrain(output, 0.0f, 180.0f);
    
    servo.write((int)output);
    
    static uint8_t printCounter = 0;
    if (++printCounter >= 10) {
        printCounter = 0;
        
        char bufPitch[8], bufServo[8], bufDist[8];
        
        dtostrf(pitch,  6, 1, bufPitch);
        dtostrf(output, 6, 1, bufServo);
        dtostrf(distance, 6, 1, bufDist);
        
        Serial.print("Pitch:");
        Serial.print(bufPitch);
        Serial.print(" | Servo:");
        Serial.print(bufServo);
        Serial.print(" | Dist:");
        Serial.print(bufDist);
        Serial.print(" | Mode: ");
        Serial.println(distance < 20.0f && distance > 0 ? "WARN" : "RC  ");
    }
}