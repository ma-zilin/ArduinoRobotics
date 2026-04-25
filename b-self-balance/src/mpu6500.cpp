#include "mpu6500.hpp"
#include <math.h>

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_GYRO_XOUT_H  = 0x43;
static const uint8_t VAL_ACCEL_2G  = 0x00;
static const uint8_t VAL_GYRO_250D = 0x00;

MPU6500::MPU6500(uint8_t address) 
    : addr_(address), pitch_(0.0f), lastUpdateUs_(0),
      accX_(0), accY_(0), accZ_(0),
      gyroX_(0), gyroY_(0), gyroZ_(0),
      whoAmI_(0) {}

bool MPU6500::begin() {
    Wire.begin();
    
    whoAmI_ = readReg(REG_WHO_AM_I);
    
    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(10);
    
    writeReg(REG_SMPLRT_DIV, 0x00);
    writeReg(REG_CONFIG, 0x03);
    writeReg(REG_GYRO_CONFIG, VAL_GYRO_250D);
    writeReg(REG_ACCEL_CONFIG, VAL_ACCEL_2G);
    delay(10);
    
    gyroOffsetX_ = 0;
    gyroOffsetY_ = 0;
    gyroOffsetZ_ = 0;
    
    delay(50);
    
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 200;
    for (int i = 0; i < samples; i++) {
        uint8_t buf[6];
        readRegs(REG_GYRO_XOUT_H, buf, 6);
        sumX += (int16_t)((buf[0] << 8) | buf[1]);
        sumY += (int16_t)((buf[2] << 8) | buf[3]);
        sumZ += (int16_t)((buf[4] << 8) | buf[5]);
        delay(2);
    }
    gyroOffsetX_ = sumX / (float)samples;
    gyroOffsetY_ = sumY / (float)samples;
    gyroOffsetZ_ = sumZ / (float)samples;
    
    Serial.print("Gyro Offset: ");
    Serial.print(gyroOffsetX_);
    Serial.print(", ");
    Serial.print(gyroOffsetY_);
    Serial.print(", ");
    Serial.println(gyroOffsetZ_);
    
    lastUpdateUs_ = micros();
    pitch_ = 0.0f;
    
    return true;
}

void MPU6500::update() {
    uint8_t buf[14];
    readRegs(REG_ACCEL_XOUT_H, buf, 14);
    
    accX_  = (int16_t)((buf[0] << 8) | buf[1]);
    accY_  = (int16_t)((buf[2] << 8) | buf[3]);
    accZ_  = (int16_t)((buf[4] << 8) | buf[5]);
    gyroX_ = (int16_t)((buf[8] << 8)  | buf[9]);
    gyroY_ = (int16_t)((buf[10] << 8) | buf[11]);
    gyroZ_ = (int16_t)((buf[12] << 8) | buf[13]);
    
    unsigned long now = micros();
    float dt = (now - lastUpdateUs_) / 1000000.0f;
    lastUpdateUs_ = now;
    
    float ax = accX_ / 16384.0f;
    float ay = accY_ / 16384.0f;
    float az = accZ_ / 16384.0f;
    
    float gx = (gyroX_ - gyroOffsetX_) / 131.0f;
    float gy = (gyroY_ - gyroOffsetY_) / 131.0f;  // 备用
    float gz = (gyroZ_ - gyroOffsetZ_) / 131.0f;  // 备用
    
    float accPitch = atan2(ay, az) * 180.0f / M_PI;
    pitch_ = 0.98f * (pitch_ + gx * dt) + 0.02f * accPitch;
}

float MPU6500::getPitch() const {
    return pitch_;
}

int16_t MPU6500::getAccelX() const { return accX_; }
int16_t MPU6500::getAccelY() const { return accY_; }
int16_t MPU6500::getAccelZ() const { return accZ_; }
int16_t MPU6500::getGyroX() const { return gyroX_; }
int16_t MPU6500::getGyroY() const { return gyroY_; }
int16_t MPU6500::getGyroZ() const { return gyroZ_; }
uint8_t MPU6500::getWhoAmI() const { return whoAmI_; }

void MPU6500::writeReg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t MPU6500::readReg(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr_, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

void MPU6500::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr_, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}