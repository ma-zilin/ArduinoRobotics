#ifndef MPU6500_HPP
#define MPU6500_HPP

#include <Arduino.h>
#include <Wire.h>

class MPU6500 {
public:
    MPU6500(uint8_t address = 0x68);
    
    bool begin();
    void update();
    float getPitch() const;
    float gyroOffsetX_, gyroOffsetY_, gyroOffsetZ_;
    bool calibrated_;
    
    int16_t getAccelX() const;
    int16_t getAccelY() const;
    int16_t getAccelZ() const;
    int16_t getGyroX() const;
    int16_t getGyroY() const;
    int16_t getGyroZ() const;
    uint8_t getWhoAmI() const;

private:
    uint8_t addr_;
    float pitch_;
    unsigned long lastUpdateUs_;
    
    int16_t accX_, accY_, accZ_;
    int16_t gyroX_, gyroY_, gyroZ_;
    uint8_t whoAmI_;
    
    void writeReg(uint8_t reg, uint8_t data);
    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
};

#endif