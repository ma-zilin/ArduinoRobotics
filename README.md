# Arduino 姿态遥控与超声波警戒系统

基于 Arduino UNO 的嵌入式控制系统，使用 MPU6500 姿态传感器遥控 SG90 舵机，HC-SR04 超声波模块实现近距离警戒。

## 硬件

| 器件 | 接口 | 注意 |
|:---|:---|:---|
| Arduino UNO | — | 16MHz ATmega328P |
| MPU6500 | I2C (A4/A5) | AD0 接 GND，地址 0x68 |
| SG90 | PWM (D9) | - |
| HC-SR04 | D11/D12 | — |

## 软件

| 文件 | 作用 |
|:---|:---|
| `main.cpp` | 50Hz 主循环，模式切换 |
| `mpu6500.cpp` | I2C 寄存器读写，互补滤波 |
| `pid.cpp` | 位置式 PID，积分限幅 |
| `hcsr04.cpp` | 脉冲测距，超时保护 |

零第三方库，只用 Arduino 内置 `Wire.h`、`Servo.h`。

## 使用

**PlatformIO**
```bash
pio run -t upload
pio device monitor -b 115200
```

## 功能

| 模式 | 条件 | 行为 |
|:---|:---|:---|
| 姿态遥控 | 距离 ≥ 20cm | 手持 MPU6500，俯仰角控制舵机角度 |
| 超声波警戒 | 距离 < 20cm | 强制舵机抬头至 120° |

## 串口输出

```
Pitch:  45.0 | Servo:  90.0 | Dist: 999.0 | Mode: RC  
Pitch:   0.0 | Servo: 180.0 | Dist:  16.2 | Mode: WARN
```

- `Pitch`：MPU6500 俯仰角（°）
- `Servo`：舵机角度（°）
- `Dist`：超声波距离（cm，999 = 无效）
- `Mode`：`RC` / `WARN`

## 关键实现

**互补滤波**
```cpp
float accPitch = atan2(ay, az) * 180.0f / M_PI;
pitch = 0.98f * (pitch + gx * dt) + 0.02f * accPitch;
```

**零偏校准**
启动时静止采样 200 次陀螺仪数据求平均，后续读数扣除该偏移。

**积分抗饱和**
```cpp
integral += error * dt;
integral = constrain(integral, -50.0f, 50.0f);
```

## 文件结构

```
├── src/
│   ├── main.cpp
│   ├── mpu6500.cpp / .hpp
│   ├── pid.cpp / .hpp
│   └── hcsr04.cpp / .hpp
├── platformio.ini
└── SelfStabilizingGimbal.ino   # 合并版，Arduino IDE 可用
```

## License

MIT
