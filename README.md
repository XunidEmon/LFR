# line-follower-pid
Arduino-based line follower robot using PID control
# 🛠️ Line Follower Robot using Arduino (PID Controlled)

This project is a **PID-based Line Follower Robot** built with Arduino UNO, 5-channel IR sensor array, and L293D motor driver. Optimized for stability and smooth path tracking.

---

## 🚗 Hardware Components

- ✅ Arduino UNO
- ✅ L293D Motor Driver
- ✅ 2 × 12V DC Gear Motors (500 RPM)
- ✅ 5-Channel IR Sensor Array (connected to A1–A5)
- ✅ External Battery (12V for motors, 5V regulated for sensors/Arduino)
- ✅ Jumper wires, Breadboard or PCB
- ✅ Chassis, Wheels

---

## ⚙️ Motor Pin Configuration

| Component       | Pin |
|----------------|-----|
| Left Motor IN1 | 7   |
| Left Motor IN2 | 8   |
| Left Motor ENA | 3 (PWM) |
| Right Motor IN3| 12  |
| Right Motor IN4| 13  |
| Right Motor ENB| 5 (PWM) |

---

## 🔧 Sensor Pin Configuration

| IR Sensor | Arduino Analog Pin |
|----------|---------------------|
| Sensor 1 (Leftmost)  | A1 |
| Sensor 2             | A2 |
| Sensor 3 (Middle)    | A3 |
| Sensor 4             | A4 |
| Sensor 5 (Rightmost) | A5 |

---

## 🎛️ PID Controller Parameters

```cpp
float Kp = 10.0;
float Ki = 0.0;
float Kd = 15.0;
