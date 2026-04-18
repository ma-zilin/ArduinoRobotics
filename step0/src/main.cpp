#include <Wire.h>
#include <Arduino.h>
#include <avr/wdt.h>

#define MPU6500_ADDR 0x68

#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

#define I2C_TIMEOUT 100

bool WriteReg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(reg);
    Wire.write(data);
    
    uint8_t result = Wire.endTransmission();

    if (result != 0) {
        Serial.print("I2C Write Error: ");
        Serial.println(result);
        return false;
    }
    return true;
}

uint8_t ReadReg(uint8_t reg) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(reg);
    uint8_t result = Wire.endTransmission(0);

    if (result != 0) {
        Serial.print("I2C Write Error: ");
        Serial.println(result);
        return 0xFF;
    }

    uint8_t byteReceived = Wire.requestFrom(MPU6500_ADDR, 1);

    if (byteReceived != 1) {
        Serial.println("I2C Read Error: No data received");
        return 0xFF;
    }

    return Wire.read();
}

bool ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(reg);
    uint8_t result = Wire.endTransmission(0);

    if (result != 0) {
        Serial.print("I2C Write Error: ");
        Serial.println(result);
        return false;
    }

    uint8_t byteReceived = Wire.requestFrom(MPU6500_ADDR, length);

    if (byteReceived != length) {
        Serial.print("I2C Read Error, expecting: ");
        Serial.print(length);
        Serial.print(", Actually Received: ");
        Serial.print(byteReceived);
        return 0;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }

    return 1;
}

bool MPU_Init() {
    uint8_t whoami = ReadReg(REG_WHO_AM_I);
    if (whoami == 0xFF) {
        Serial.println("Error: Can't Receive WHOAMI");
        return 0;
    }

    if (whoami != 0x70) {
        Serial.print("Error: WHO_AM_I Value is Wrong: ");
        Serial.println(whoami, HEX);
        return 0;
    }
    Serial.println("Connect Succeed.");

    if (!WriteReg(REG_PWR_MGMT_1, 0x00)) {
        Serial.println("Error: Can't Wake.");
        return 0;
    }
    delay(100);

    if (!WriteReg(REG_GYRO_CONFIG, 0x18)) {
        Serial.println("错误：配置陀螺仪失败");
        return 0;
    }
    
    if (!WriteReg(REG_ACCEL_CONFIG, 0x18)) {
        Serial.println("错误：配置加速度计失败");
        return 0;
    }
    
    Serial.println("MPU6500初始化完成");
    return 1;
}

struct Data {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;

    float ax_g, ay_g, az_g;
    float temp_c;
    float gx_dps, gy_dps, gz_dps;
};

bool ReadAllData(Data* data) {
    uint8_t buffer[14];
    
    if (!ReadRegs(REG_ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }
    
    data->ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->az = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return 1;
}

void ConvertToPhysical(Data* data) {
    const float ACCEL_SCALE = 2048.0f;
    data->ax_g = (float)data->ax / ACCEL_SCALE;
    data->ay_g = (float)data->ay / ACCEL_SCALE;
    data->az_g = (float)data->az / ACCEL_SCALE;
    
    data->temp_c = (float)data->temp / 333.87f + 21.0f;
    
    const float GYRO_SCALE = 16.4f;
    data->gx_dps = (float)data->gx / GYRO_SCALE;
    data->gy_dps = (float)data->gy / GYRO_SCALE;
    data->gz_dps = (float)data->gz / GYRO_SCALE;
}

// 互补滤波（一维姿态角）

class ComplementaryFilter {
private:
    float angle;         
    float alpha;          
    unsigned long last_time;

public:
    ComplementaryFilter(float alpha_val = 0.98f) {
        angle = 0.0f;
        alpha = alpha_val;
        last_time = 0;
    }
    
    void Reset(float initial_angle) {
        angle = initial_angle;
    }
    
    float Update(float accel_angle, float gyro_rate, float dt) {
        float gyro_angle = angle + gyro_rate * dt;
        
        angle = alpha * gyro_angle + (1.0f - alpha) * accel_angle;
        
        return angle;
    }
    
    float UpdateAuto(float accel_angle, float gyro_rate) {
        unsigned long now = micros();
        float dt = 0.0f;
        
        if (last_time != 0) {
            dt = (now - last_time) / 1000000.0f;            
            if (dt > 0.1f) dt = 0.01f;
        }
        last_time = now;
        
        return Update(accel_angle, gyro_rate, dt);
    }
};

float CalculatePitchFromAccel(float ax_g, float ay_g, float az_g) {
    float denominator = sqrt(ay_g * ay_g + az_g * az_g);
    if (denominator < 0.0001f) denominator = 0.0001f;
    
    return atan2f(ax_g, denominator) * 180.0f / M_PI;
}

float CalculateRollFromAccel(float ax_g, float ay_g, float az_g) {
    return atan2f(ay_g, az_g) * 180.0f / M_PI;
}

bool Write_Timeout(uint8_t addr, uint8_t reg, uint8_t data, uint8_t timeout_ms) {
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(data);
        uint8_t result = Wire.endTransmission(1);

        if (result == 0) {
            return 1;
        }

        delay(1);
    }
    return false;
}

uint8_t Read_Timeout(uint8_t addr, uint8_t reg, uint32_t timeout_ms, bool* success) {
    uint32_t start = millis();
    
    while (millis() - start < timeout_ms) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        uint8_t result = Wire.endTransmission(false);
        
        if (result != 0) {
            delay(1);
            continue;
        }
        
        if (Wire.requestFrom(addr, 1) == 1) {
            if (success) *success = true;
            return Wire.read();
        }
        
        delay(1);
    }
    
    if (success) *success = false;
    return 0xFF;
}

bool ReadAllData_WithTimeout(Data* data, uint32_t timeout_ms) {
    uint32_t start = millis();
    uint8_t buffer[14];
    
    while (millis() - start < timeout_ms) {
        Wire.beginTransmission(MPU6500_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        uint8_t result = Wire.endTransmission(false);
        
        if (result == 0) {
            if (Wire.requestFrom(MPU6500_ADDR, 14) == 14) {
                for (int i = 0; i < 14; i++) {
                    buffer[i] = Wire.read();
                }
                
                data->ax = (int16_t)((buffer[0] << 8) | buffer[1]);
                data->ay = (int16_t)((buffer[2] << 8) | buffer[3]);
                data->az = (int16_t)((buffer[4] << 8) | buffer[5]);
                data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
                data->gx = (int16_t)((buffer[8] << 8) | buffer[9]);
                data->gy = (int16_t)((buffer[10] << 8) | buffer[11]);
                data->gz = (int16_t)((buffer[12] << 8) | buffer[13]);
                
                return true;
            }
        }
        
        delay(1);
    }
    
    Serial.println("MPU6500 Read Timeout.");
    return false;
}

void I2C_SoftwareReset() {
    Serial.println("执行I2C总线复位...");
    
    Wire.end();     
    delay(10);
    Wire.begin();
    Wire.setClock(400000);
    
    delay(100);
    Serial.println("I2C总线复位完成");
}

// ============================================
// 主程序
// ============================================

ComplementaryFilter filter(0.98f);
Data mpu_data;

void setup() {
    wdt_enable(WDTO_2S);

    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("MPU6500 驱动初始化...");
    
    delay(500);

    Wire.begin();
    Wire.setClock(100000);
    
    if (!MPU_Init()) {
        Serial.println("MPU6500初始化失败，停止运行");
        while (1) { delay(1000); }
    }
    
    if (ReadAllData(&mpu_data)) {
        ConvertToPhysical(&mpu_data);
        float init_pitch = CalculatePitchFromAccel(mpu_data.ax_g, mpu_data.ay_g, mpu_data.az_g);
        filter.Reset(init_pitch);
    }
    
    Serial.println("开始读取数据...");
}

void loop() {
    wdt_reset();

    if (ReadAllData_WithTimeout(&mpu_data, 100)) {
        ConvertToPhysical(&mpu_data);
    
        float pitch_acc = CalculatePitchFromAccel(mpu_data.ax_g, mpu_data.ay_g, mpu_data.az_g);
        float pitch = filter.UpdateAuto(pitch_acc, mpu_data.gy_dps);
    
        Serial.print("Pitch: ");
        Serial.print(pitch, 2);
        Serial.print("°  |  Acc Pitch: ");
        Serial.print(pitch_acc, 1);
        Serial.print("°  |  Raw GX: ");
        Serial.print(mpu_data.gx);
        Serial.print("   |  Raw GY: ");
        Serial.print(mpu_data.gy);
        Serial.print("   |  Raw GZ: ");
        Serial.println(mpu_data.gz);
    } else {
        Serial.println("Error, Skip this Loop.");

        static uint8_t fail_count = 0;
        fail_count++;

        if (fail_count >= 10) {
            Serial.println("Reset I2C...");
            
            while(1) {
                delay(1000);
            }
        }
    }
    delay(10);
}