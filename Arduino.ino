#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// PINOUT
#define IR_LEFT 12
#define IR_RIGHT 13
#define ECHO 2
#define TRIG 3

#define LEFT_IN1 4
#define LEFT_IN2 5
#define RIGHT_IN1 6
#define RIGHT_IN2 7

#define LED_YELLOW 8
#define LED_RED 9
#define LED_GREEN 11
#define BUZZER 10

#define SPEED_NORMAL 255
#define SPEED_SLOW 180
#define OBSTACLE_DIST 30

// THRESHOLDS (Adjusted for m/s^2)
#define TILT_THRESHOLD 5.5
#define VIBRATION_THRESHOLD 2.5

#define GYRO_VIBRATION_THRESHOLD 1.75

typedef enum { MODE_IDLE, MODE_A, MODE_B } RobotMode;
RobotMode currentMode = MODE_A;

struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  bool tiltDetected = false;
  bool vibrationDetected = false;
} sensor;

float prevX = 0, prevY = 0, prevZ = 0;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10); // Wait for Serial Monitor

  Serial.println(F("\n=== STARTING SETUP ==="));

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  // Motor pins
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  // Ultrasonic & IR
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Initialize Adafruit MPU6050
  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 not found! Check wiring (A4/A5)"));
    while (1) {
      digitalWrite(LED_RED, HIGH);
      delay(200);
      digitalWrite(LED_RED, LOW);
      delay(200);
    }
  }

  Serial.println(F("MPU6050 Found!"));

  // Set sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("=== SETUP COMPLETE ==="));
}

void loop() {
  static unsigned long lastDistCheck = 0;

  readSensors();
  checkThresholds();

  if (sensor.tiltDetected) {
    Serial.println(F("TILT DETECTED → STOPPED"));
    stopMotors();
  } else if (currentMode == MODE_A) {
    runModeADebug();
  } else if (currentMode == MODE_B) {
    runModeB();
  } else {
    stopMotors();
  }

  // Non-blocking distance print
  if (millis() - lastDistCheck > 800) {
    long d = getDistance();
    Serial.print(F("Dist: "));
    Serial.print(d);
    Serial.print(F("cm | X:"));
    Serial.print(sensor.accelX);
    Serial.print(F(" Y:"));
    Serial.println(sensor.accelY);
    lastDistCheck = millis();
  }

  delay(50); // Small delay for stability
}

void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensor.accelX = a.acceleration.x;
  sensor.accelY = a.acceleration.y;
  sensor.accelZ = a.acceleration.z;

  sensor.gyroX = g.gyro.x;
  sensor.gyroY = g.gyro.y;
  sensor.gyroZ = g.gyro.z;
}

void checkThresholds() {
  // Tilt (unchanged)
  sensor.tiltDetected = (abs(sensor.accelX) > TILT_THRESHOLD) ||
                        (abs(sensor.accelY) > TILT_THRESHOLD);

  // Accel-based vibration (delta between readings)
  float dX = abs(sensor.accelX - prevX);
  float dY = abs(sensor.accelY - prevY);
  float dZ = abs(sensor.accelZ - prevZ);
  bool accelVibration = (dX > VIBRATION_THRESHOLD) ||
                        (dY > VIBRATION_THRESHOLD) ||
                        (dZ > VIBRATION_THRESHOLD);

  // Gyro-based vibration (instantaneous rotational rate)
  bool gyroVibration = (abs(sensor.gyroX) > GYRO_VIBRATION_THRESHOLD) ||
                       (abs(sensor.gyroY) > GYRO_VIBRATION_THRESHOLD) ||
                       (abs(sensor.gyroZ) > GYRO_VIBRATION_THRESHOLD);

  sensor.vibrationDetected = accelVibration || gyroVibration;

  prevX = sensor.accelX;
  prevY = sensor.accelY;
  prevZ = sensor.accelZ;
}

void runModeADebug() {
  long distance = getDistance();
  bool cliffL = digitalRead(IR_LEFT) == LOW;
  bool cliffR = digitalRead(IR_RIGHT) == LOW;

  if (cliffL || cliffR) {
    moveBackward(200);
    delay(300);
    turnRight(220);
    delay(450);
  } else if (distance < OBSTACLE_DIST && distance > 2) {
    stopMotors();
    delay(100);
    moveBackward(180);
    delay(300);
    turnLeft(220);
    delay(400);
  } else {
    moveForward(SPEED_NORMAL);
  }
}

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 25000); // 25ms timeout
  return duration * 0.0343 / 2;
}

void pulsedTurn(bool rightTurn, int speed, int onMs, int offMs, int pulses) {
  for (int i = 0; i < pulses; i++) {
    if (rightTurn)
      turnRight(speed);
    else
      turnLeft(speed);
    delay(onMs);
    stopMotors();
    delay(offMs);
  }
}

void runModeB() {
  long distance = getDistance();
  bool cliffL = digitalRead(IR_LEFT) == LOW;
  bool cliffR = digitalRead(IR_RIGHT) == LOW;

  if (cliffL || cliffR) {
    moveBackward(200);
    delay(300);
    pulsedTurn(true, 220, 80, 60, 6);
  } else if (distance < OBSTACLE_DIST && distance > 2) {
    stopMotors();
    delay(100);
    moveBackward(100);
    delay(300);
    pulsedTurn(false, 220, 80, 60, 6);
  } else {
    moveForward(SPEED_NORMAL);
  }
}

void moveForward(int speed) {
  analogWrite(LEFT_IN1, speed);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, speed);
  digitalWrite(RIGHT_IN2, LOW);
}

void moveBackward(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  analogWrite(LEFT_IN2, speed);
  digitalWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}

void turnLeft(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  analogWrite(LEFT_IN2, speed);
  analogWrite(RIGHT_IN1, speed);
  digitalWrite(RIGHT_IN2, LOW);
}

void turnRight(int speed) {
  analogWrite(LEFT_IN1, speed);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}
