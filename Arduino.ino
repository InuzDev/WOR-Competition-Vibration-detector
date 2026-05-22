#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ============================================================
// PINOUT - Definición de pines del hardware
// ============================================================
#define ECHO 2
#define TRIG 3

#define LEFT_IN1 4
#define LEFT_IN2 5
#define RIGHT_IN1 6
#define RIGHT_IN2 7

#define LED_YELLOW 8
#define LED_RED 9
#define BUZZER 10
#define LED_GREEN 11

#define SPEED_NORMAL 255
#define SPEED_SLOW 180
#define OBSTACLE_DIST 20

// ============================================================
// UMBRALES DE DETECCIÓN
// Acelerómetro en m/s², giroscopio en rad/s
// ============================================================
#define TILT_MODERATE 3.5 // Inicio de inclinación moderada → LED amarillo
#define TILT_CRITICAL 5.5 // Inclinación crítica → LED rojo + buzzer + stop
#define VIBRATION_LOW 30.0 // Inicio de vibración moderada → MODE_B + LED amarillo
#define VIBRATION_HIGH 40.0 // Vibración crítica → alerta (LED rojo + buzzer)
#define GYRO_VIBRATION_LOW 30.0  // Inicio de vibración giroscópica moderada
#define GYRO_VIBRATION_HIGH 40.0 // Vibración giroscópica crítica

// ============================================================
// ENUMERACIÓN DE MODOS
// Define los modos de operación del robot
// ============================================================
typedef enum { MODE_IDLE, MODE_A, MODE_B } RobotMode;
RobotMode currentMode = MODE_A;

// ============================================================
// ESTRUCTURA DE DATOS DEL SENSOR
// Almacena lecturas del MPU6050 y flags de detección
// ============================================================
struct SensorData {
  float accelX, accelY, accelZ;   // Aceleración en m/s²
  float gyroX, gyroY, gyroZ;      // Velocidad angular en rad/s
  bool tiltDetected = false;      // Inclinación crítica detectada
  bool tiltModerate = false;      // Inclinación moderada
  bool vibrationDetected = false; // Vibración moderada detectada
  bool alertDetected = false;     // Vibración crítica detectada
} sensor;

float prevX = 0, prevY = 0, prevZ = 0;
Adafruit_MPU6050 mpu;

// ============================================================
// SETUP
// Inicializa pines, comunicación serial y el MPU6050
// ============================================================
void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  Serial.println(F("\n=== INICIANDO SETUP ==="));

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  // Pines de motores
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  // Sensor ultrasónico
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 no encontrado. Revisar cableado (A4/A5)"));
    while (1) {
      digitalWrite(LED_RED, HIGH);
      delay(200);
      digitalWrite(LED_RED, LOW);
      delay(200);
    }
  }

  Serial.println(F("MPU6050 encontrado!"));

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("=== SETUP COMPLETO ==="));
}

// ============================================================
// LOOP PRINCIPAL
// Lee sensores, evalúa umbrales, actualiza indicadores y
// ejecuta el modo de operación correspondiente
// ============================================================
void loop() {
  static unsigned long lastDistCheck = 0;
  static long cachedDistance = 999;

  readSensors();
  checkThresholds();
  updateIndicators();

  cachedDistance = getDistance();

  if (sensor.tiltDetected || sensor.alertDetected) {
    stopMotors();
  } else if (currentMode == MODE_A) {
    runModeA(cachedDistance);
  } else if (currentMode == MODE_B) {
    runModeB(cachedDistance);
  } else {
    stopMotors();
  }

  if (millis() - lastDistCheck > 800) {
    Serial.print(F("Dist: "));
    Serial.print(cachedDistance);
    Serial.print(F("cm | X: "));
    Serial.print(sensor.accelX);
    Serial.print(F(" Y: "));
    Serial.print(sensor.accelY);
    Serial.print(F(" | Modo: "));
    Serial.println(currentMode == MODE_A ? F("A") : F("B"));
    lastDistCheck = millis();
  }

  delay(50);
}

// ============================================================
// readSensors
// Lee los datos crudos del MPU6050 y los almacena en
// la estructura global `sensor`
// ============================================================
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

// ============================================================
// checkThresholds
// Evalúa los datos del sensor contra los umbrales definidos.
// Clasifica el estado en: normal, vibración moderada (MODE_B)
// o vibración crítica (alerta). También detecta inclinación.
// ============================================================
void checkThresholds() {
  float maxAccel = max(abs(sensor.accelX), abs(sensor.accelY));

  // Clasificar nivel de inclinación
  sensor.tiltDetected = maxAccel > TILT_CRITICAL;
  sensor.tiltModerate = !sensor.tiltDetected && (maxAccel > TILT_MODERATE);

  // Deltas del acelerómetro entre lecturas
  float dX = abs(sensor.accelX - prevX);
  float dY = abs(sensor.accelY - prevY);
  float dZ = abs(sensor.accelZ - prevZ);
  float maxAccelDelta = max(dX, max(dY, dZ));

  // Máximo valor instantáneo del giroscopio
  float maxGyro =
      max(abs(sensor.gyroX), max(abs(sensor.gyroY), abs(sensor.gyroZ)));

  // Clasificar nivel de vibración
  bool accelAlert = maxAccelDelta > VIBRATION_HIGH;
  bool accelVibration = maxAccelDelta > VIBRATION_LOW;
  bool gyroAlert = maxGyro > GYRO_VIBRATION_HIGH;
  bool gyroVibration = maxGyro > GYRO_VIBRATION_LOW;

  sensor.alertDetected = accelAlert || gyroAlert;
  sensor.vibrationDetected =
      !sensor.alertDetected && (accelVibration || gyroVibration);

  // Cambio de modo — tilt crítico y alerta tienen prioridad
  if (sensor.tiltDetected || sensor.alertDetected) {
    currentMode = MODE_IDLE;
  } else if (sensor.vibrationDetected || sensor.tiltModerate) {
    currentMode = MODE_B;
  } else {
    currentMode = MODE_A;
  }

  prevX = sensor.accelX;
  prevY = sensor.accelY;
  prevZ = sensor.accelZ;
}

// ============================================================
// updateIndicators
// Controla los LEDs y el buzzer según el estado actual:
//   - Alerta/inclinación → LED rojo + buzzer
//   - MODE_B (vibración moderada) → LED amarillo
//   - MODE_A (normal) → LED verde
// ============================================================
void updateIndicators() {
  if (sensor.tiltDetected || sensor.alertDetected) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    tone(BUZZER, 1000);
    if (sensor.tiltDetected)
      Serial.println(F("INCLINACIÓN CRÍTICA → DETENIDO"));
    if (sensor.alertDetected)
      Serial.println(F("VIBRACIÓN CRÍTICA → DETENIDO"));
  } else {
    digitalWrite(LED_RED, LOW);
    noTone(BUZZER);
    digitalWrite(LED_GREEN, currentMode == MODE_A ? HIGH : LOW);
    digitalWrite(LED_YELLOW, currentMode == MODE_B ? HIGH : LOW);
  }
}

// ============================================================
// getDistance
// Dispara el sensor ultrasónico HC-SR04 y retorna la
// distancia en centímetros. Retorna 999 si hay timeout
// (sin obstáculo detectado).
// ============================================================
long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 25000);
  if (duration == 0)
    return 999;
  return duration * 0.0343 / 2;
}

// ============================================================
// runModeA
// Modo de operación normal. Avanza hacia adelante y esquiva
// obstáculos con reversa + giro izquierda.
// ============================================================
void runModeA(long distance) {
  if (distance < OBSTACLE_DIST) {
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

// ============================================================
// runModeB
// Modo de vibración moderada. Igual que MODE_A pero usa
// giros pulsados para evitar el giro de 360° causado por
// las ruedas traseras de baja fricción.
// ============================================================
void runModeB(long distance) {
  if (distance < OBSTACLE_DIST) {
    stopMotors();
    delay(100);
    moveBackward(100);
    delay(300);
    pulsedTurn(false, 220, 80, 60, 6);
  } else {
    moveForward(SPEED_NORMAL);
  }
}

// ============================================================
// pulsedTurn
// Realiza un giro en pulsos cortos para evitar que el robot
// gire de más debido a las ruedas traseras de baja fricción.
// Parámetros:
//   rightTurn → true = derecha, false = izquierda
//   speed     → velocidad PWM (0-255)
//   onMs      → duración de cada pulso en ms
//   offMs     → pausa entre pulsos en ms
//   pulses    → cantidad de pulsos
// ============================================================
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

// ============================================================
// moveForward
// Mueve el robot hacia adelante a la velocidad indicada
// ============================================================
void moveForward(int speed) {
  analogWrite(LEFT_IN1, speed);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, speed);
  digitalWrite(RIGHT_IN2, LOW);
}

// ============================================================
// moveBackward
// Mueve el robot hacia atrás a la velocidad indicada
// ============================================================
void moveBackward(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  analogWrite(LEFT_IN2, speed);
  digitalWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}

// ============================================================
// turnLeft
// Gira el robot hacia la izquierda (rueda izquierda atrás,
// rueda derecha adelante)
// ============================================================
void turnLeft(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  analogWrite(LEFT_IN2, speed);
  analogWrite(RIGHT_IN1, speed);
  digitalWrite(RIGHT_IN2, LOW);
}

// ============================================================
// turnRight
// Gira el robot hacia la derecha (rueda izquierda adelante,
// rueda derecha atrás)
// ============================================================
void turnRight(int speed) {
  analogWrite(LEFT_IN1, speed);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}

// ============================================================
// stopMotors
// Detiene ambos motores cortando todas las señales PWM
// ============================================================
void stopMotors() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}
