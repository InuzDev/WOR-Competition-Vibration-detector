#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>

// PINOUT
#define IR_LEFT      12
#define IR_RIGHT     13
#define ECHO          2
#define TRIG          3

#define LEFT_IN1      4
#define LEFT_IN2      5
#define RIGHT_IN1     6
#define RIGHT_IN2     7

#define LED_YELLOW    8
#define LED_RED       9
#define BUZZER       10
#define LED_GREEN    11

#define SPEED_NORMAL   255
#define SPEED_SLOW     180
#define OBSTACLE_DIST  30

typedef enum { MODE_IDLE, MODE_A, MODE_B } RobotMode;
RobotMode currentMode = MODE_A;

struct SensorData {
  int16_t accelX, accelY, accelZ;
  bool tiltDetected = false;
  bool vibrationDetected = false;
} sensor;

int16_t prevX=0, prevY=0, prevZ=0;
MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); 
  Serial.println(F("\n=== STARTING SETUP ==="));
  
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  
  // Motor pins
  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();
  
  Serial.println(F("Motor pins configured"));

  // Ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.println(F("Ultrasonic pins configured"));

  // IR
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  Serial.println(F("IR pins configured"));

  // MPU6050
  Serial.println(F("Starting Wire & MPU6050..."));
  Wire.begin();
  mpu.initialize();
  Serial.println(F("mpu.initialize() done"));

  bool connected = mpu.testConnection();
  Serial.print(F("MPU6050 testConnection: "));
  Serial.println(connected ? "SUCCESS" : "FAILED");

  if (!connected) {
    Serial.println(F("ERROR: MPU6050 not found! Check wiring (A4/A5)"));
    while(true) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      delay(200);
    }
  }

  Serial.println(F("=== SETUP COMPLETE - ENTERING LOOP ==="));
  Serial.println(F("Robot should start moving now..."));
}

void loop() {
  static unsigned long last = 0;
  
  readSensors();
  checkThresholds();
  
  Serial.print(F("Loop | Tilt:")); Serial.print(sensor.tiltDetected);
  Serial.print(F(" Vib:")); Serial.print(sensor.vibrationDetected);
  Serial.print(F(" | Mode:")); Serial.println(currentMode);

  if (sensor.tiltDetected) {
    Serial.println(F("TILT DETECTED → STOPPED"));
    stopMotors();
  } 
  else if (currentMode == MODE_A) {
    runModeADebug();
  } else {
    stopMotors();
  }

  if (millis() - last > 800) {
    long d = getDistance();
    Serial.print(F("Distance: ")); Serial.print(d); Serial.println(F(" cm"));
    last = millis();
  }

  delay(100);
}

// =============================================================================
void runModeADebug() {
  long distance = getDistance();
  bool cliffL = digitalRead(IR_LEFT) == LOW;
  bool cliffR = digitalRead(IR_RIGHT) == LOW;

  if (cliffL || cliffR) {
    Serial.println(F("[CLIFF] Avoiding..."));
    moveBackward(200);
    delay(300);
    turnRight(220);
    delay(450);
  }
  else if (distance < OBSTACLE_DIST && distance > 5) {
    Serial.println(F("[OBSTACLE] Avoiding..."));
    stopMotors();
    delay(150);
    moveBackward(180);
    delay(300);
    turnLeft(220);
    delay(400);
  }
  else {
    Serial.println(F("[FORWARD] Moving..."));
    moveForward(SPEED_NORMAL);
  }
}

void readSensors() {
  mpu.getAcceleration(&sensor.accelX, &sensor.accelY, &sensor.accelZ);
}

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 20000);
  return duration * 0.0343 / 2;
}

void checkThresholds() {
  sensor.tiltDetected = (abs(sensor.accelX) > 9000) || (abs(sensor.accelY) > 9000);

  int16_t dX = abs(sensor.accelX - prevX);
  int16_t dY = abs(sensor.accelY - prevY);
  int16_t dZ = abs(sensor.accelZ - prevZ);

  sensor.vibrationDetected = (dX > 4000) || (dY > 4000) || (dZ > 4000);

  prevX = sensor.accelX;
  prevY = sensor.accelY;
  prevZ = sensor.accelZ;
}

void moveForward(int speed) {
  analogWrite(LEFT_IN1, speed);  digitalWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, speed); digitalWrite(RIGHT_IN2, LOW);
}

void moveBackward(int speed) {
  analogWrite(LEFT_IN1, LOW);   digitalWrite(LEFT_IN2, HIGH);
  analogWrite(RIGHT_IN1, LOW);  digitalWrite(RIGHT_IN2, HIGH);
}

void turnLeft(int speed) {
  analogWrite(LEFT_IN1, LOW);    digitalWrite(LEFT_IN2, HIGH);
  analogWrite(RIGHT_IN1, speed); digitalWrite(RIGHT_IN2, LOW);
}

void turnRight(int speed) {
  analogWrite(LEFT_IN1, speed);  digitalWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, LOW);   digitalWrite(RIGHT_IN2, HIGH);
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
}
