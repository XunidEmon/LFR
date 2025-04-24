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
// মোটরের পিন
int in1 = 7;
int in2 = 8;
int Ena = 3;

int in3 = 12;
int in4 = 13;
int Enb = 5;

// সেন্সর পিন
int s1 = A1;
int s2 = A2;
int s3 = A3;
int s4 = A4;
int s5 = A5;

// কনফিগারেশন
int threshold = 550;

// PID variables
float Kp = 10.0;
float Ki = 0.0;
float Kd = 15.0;

float error = 0, previousError = 0, integral = 0;
int baseSpeed = 40;
int maxSpeed = 60;

void setup() {
  Serial.begin(9600);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(Ena, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(Enb, OUTPUT);

  pinMode(s1, INPUT); pinMode(s2, INPUT); pinMode(s3, INPUT);
  pinMode(s4, INPUT); pinMode(s5, INPUT);
}

void loop() {
  int IR1 = analogRead(s1);
  int IR2 = analogRead(s2);
  int IR3 = analogRead(s3);
  int IR4 = analogRead(s4);
  int IR5 = analogRead(s5);

  bool L1 = IR1 < threshold;
  bool L2 = IR2 < threshold;
  bool L3 = IR3 < threshold;
  bool L4 = IR4 < threshold;
  bool L5 = IR5 < threshold;

  // Position Calculation (weighted sum)
  int position = 0;
  int activeSensors = 0;

  if (L1) { position += -2; activeSensors++; }
  if (L2) { position += -1; activeSensors++; }
  if (L3) { position +=  0; activeSensors++; }
  if (L4) { position +=  1; activeSensors++; }
  if (L5) { position +=  2; activeSensors++; }

  if (activeSensors > 0)
    error = (float)position / activeSensors;
  else
    error = previousError;

  integral += error;
  float derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Speed limit clamp
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(Ena, leftSpeed);
  analogWrite(Enb, rightSpeed);

  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

  previousError = error;

  // Debugging info
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | Lspeed: "); Serial.print(leftSpeed);
  Serial.print(" Rspeed: "); Serial.println(rightSpeed);

  delay(30); // smoother behavior
}



///new code
// মোটরের পিন
int in1 = 7;
int in2 = 8;
int Ena = 3;

int in3 = 12;
int in4 = 13;
int Enb = 5;

// সেন্সর পিন
int s1 = A1;
int s2 = A2;
int s3 = A3;
int s4 = A4;
int s5 = A5;

// কনফিগারেশন
int threshold = 550;

// PID variables
float Kp = 12.0;
float Ki = 0.0;
float Kd = 20.0;

float error = 0, previousError = 0, integral = 0;
int baseSpeed = 40;
int maxSpeed = 50;

void setup() {
  Serial.begin(9600);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(Ena, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(Enb, OUTPUT);

  pinMode(s1, INPUT); pinMode(s2, INPUT); pinMode(s3, INPUT);
  pinMode(s4, INPUT); pinMode(s5, INPUT);
}

void loop() {
  int IR1 = analogRead(s1);
  int IR2 = analogRead(s2);
  int IR3 = analogRead(s3);
  int IR4 = analogRead(s4);
  int IR5 = analogRead(s5);

  bool L1 = IR1 < threshold;
  bool L2 = IR2 < threshold;
  bool L3 = IR3 < threshold;
  bool L4 = IR4 < threshold;
  bool L5 = IR5 < threshold;

  int position = 0;
  int activeSensors = 0;

  if (L1) { position += -2; activeSensors++; }
  if (L2) { position += -1; activeSensors++; }
  if (L3) { position +=  0; activeSensors++; }
  if (L4) { position +=  1; activeSensors++; }
  if (L5) { position +=  2; activeSensors++; }

  if (activeSensors > 0) {
    error = (float)position / activeSensors;
  } else {
    // No sensors detecting line, use last error
    error = previousError;
  }

  integral += error;
  float derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Adjust left motor speed to be slower
  int leftSpeed = baseSpeed + correction - 10;  // Decrease left motor speed
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(Ena, leftSpeed);
  analogWrite(Enb, rightSpeed);

  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

  previousError = error;

  Serial.print("Err: "); Serial.print(error);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);

  delay(25);
}
