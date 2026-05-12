#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// PINOUT
#define PIN_IR_LEFT       12
#define PIN_IR_RIGHT      13

#define PIN_ECHO          2
#define PIN_TRIG          3

#define PIN_LEFT_IN1      4
#define PIN_LEFT_IN2      5

#define PIN_RIGHT_IN1     6
#define PIN_RIGHT_IN2     7

#define PIN_LED_YELLOW    8    // Modo B — vibración baja
#define PIN_LED_RED       9    // Modo S — vibración alta / inclinación
#define PIN_BUZZER        10   // Modo S — alerta audible
#define PIN_LED_GREEN     11   // Modo A — sistema activo normal


// =============================================================================
// UMBRALES DE DETECCIÓN (en m/s²)
// Adafruit MPU6050 retorna valores en m/s² directamente.
// Robot horizontal en reposo: accelZ ≈ 9.8 m/s² (gravedad), accelX/Y ≈ 0
// TODO: Ajustar después de pruebas físicas monitoreando el Serial.
// =============================================================================

// Inclinación — activa Modo S directamente
// 5.5 m/s² en X o Y ≈ inclinación de ~34°
#define TILT_THRESHOLD        5.5

// Vibración baja — activa Modo B (advertencia, robot sigue moviéndose)
#define VIBRATION_LOW         2.5

// Vibración alta — activa Modo S (parada total)
#define VIBRATION_HIGH        5.0

// Distancia mínima al obstáculo frontal antes de evadir (cm)
#define OBSTACLE_DIST         30

// Tiempos de maniobra (ms)
#define BACKUP_DURATION       300
#define TURN_DURATION         450


// =============================================================================
// ESTADOS DEL ROBOT
// =============================================================================

typedef enum {
  MODE_A,
  MODE_B, 
  MODE_S    
} RobotMode;

RobotMode currentMode = MODE_A;


// =============================================================================
// DATOS DEL SENSOR MPU6050
// =============================================================================

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  bool tiltDetected;
  bool vibrationLow;
  bool vibrationHigh;
};

SensorData sensor = {0, 0, 0, false, false, false};

float prevX = 0, prevY = 0, prevZ = 0;

Adafruit_MPU6050 mpu;

void setup() {

  Serial.begin(9600);
  Serial.println(F("\n=== WRO Robot — Iniciando ==="));

  Wire.begin();

  if (!mpu.begin()) {
    Serial.println(F("[ERROR] MPU6050 no encontrado. Verificar SDA(A4)/SCL(A5)."));
    pinMode(PIN_LED_RED, OUTPUT);
    while (true) {
      digitalWrite(PIN_LED_RED, HIGH); delay(200);
      digitalWrite(PIN_LED_RED, LOW);  delay(200);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("[OK] MPU6050 conectado y configurado."));

  pinMode(PIN_IR_LEFT,  INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_LEFT_IN1,  OUTPUT);
  pinMode(PIN_LEFT_IN2,  OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);

  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);

  stopMotors();
  clearAlerts();

  delay(500);

  Serial.println(F("[OK] Sistema listo. Iniciando Modo A."));
}


// =============================================================================
// LOOP PRINCIPAL
// =============================================================================
void loop() {

  readSensors();

  checkThresholds();

  updateMode();

  switch (currentMode) {
    case MODE_A: runModeA(); break;
    case MODE_B: runModeB(); break;
    case MODE_S: runModeS(); break;
  }

  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 800) {
    long dist = getDistance();
    Serial.print(F("Modo: "));
    Serial.print(currentMode == MODE_A ? "A" : currentMode == MODE_B ? "B" : "S");
    Serial.print(F(" | Dist: ")); Serial.print(dist);
    Serial.print(F("cm | X: ")); Serial.print(sensor.accelX);
    Serial.print(F(" | Y: ")); Serial.print(sensor.accelY);
    Serial.print(F(" | Tilt: ")); Serial.print(sensor.tiltDetected ? "SI" : "NO");
    Serial.print(F(" | VibLow: ")); Serial.print(sensor.vibrationLow ? "SI" : "NO");
    Serial.print(F(" | VibHigh: ")); Serial.println(sensor.vibrationHigh ? "SI" : "NO");
    lastDebugPrint = millis();
  }

  delay(50);
}


// =============================================================================
// FUNCIÓN: readSensors
// Lee aceleración del MPU6050 usando la librería Adafruit.
// Valores en m/s² — robot horizontal: accelZ ≈ 9.8, accelX/Y ≈ 0
// =============================================================================
void readSensors() {
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  sensor.accelX = accelEvent.acceleration.x;
  sensor.accelY = accelEvent.acceleration.y;
  sensor.accelZ = accelEvent.acceleration.z;
}


// =============================================================================
// FUNCIÓN: checkThresholds
// Evalúa lecturas del MPU6050 contra los umbrales definidos.
// Actualiza: tiltDetected, vibrationLow, vibrationHigh.
// =============================================================================
void checkThresholds() {
  sensor.tiltDetected =
    (abs(sensor.accelX) > TILT_THRESHOLD) ||
    (abs(sensor.accelY) > TILT_THRESHOLD);
  float dX = abs(sensor.accelX - prevX);
  float dY = abs(sensor.accelY - prevY);
  float dZ = abs(sensor.accelZ - prevZ);

  float maxDelta = max(dX, max(dY, dZ));

  sensor.vibrationLow  = (maxDelta >= VIBRATION_LOW && maxDelta < VIBRATION_HIGH);

  sensor.vibrationHigh = (maxDelta >= VIBRATION_HIGH);

  prevX = sensor.accelX;
  prevY = sensor.accelY;
  prevZ = sensor.accelZ;
}


// =============================================================================
// FUNCIÓN: updateMode
// Transiciona entre modos según las lecturas del MPU6050.
// Prioridad: Modo S (crítico) > Modo B (advertencia) > Modo A (normal)
// =============================================================================
void updateMode() {

  if (sensor.tiltDetected || sensor.vibrationHigh) {
    currentMode = MODE_S;

  } else if (sensor.vibrationLow) {
    if (currentMode == MODE_A) {
      currentMode = MODE_B;
    }

  } else {
    currentMode = MODE_A;
  }
}


// =============================================================================
// FUNCIÓN: runModeA
// Modo A — Movimiento autónomo normal.
// Evita obstáculos (HC-SR04) y precipicios (IR).
// LED verde encendido.
// =============================================================================
void runModeA() {
  digitalWrite(PIN_LED_GREEN,  HIGH);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     LOW);

  runAutonomousMovement();
}


// =============================================================================
// FUNCIÓN: runModeB
// Modo B — Vibración baja detectada.
// Robot continúa moviéndose con la misma lógica del Modo A.
// LED amarillo encendido como advertencia visual.
// =============================================================================
void runModeB() {

  digitalWrite(PIN_LED_YELLOW, HIGH);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     LOW);

  runAutonomousMovement();
}


// =============================================================================
// FUNCIÓN: runModeS
// Modo S — Parada de emergencia total.
// Motores detenidos, LED rojo + buzzer activos.
// El robot permanece aquí hasta que updateMode() detecte lecturas normales.
// =============================================================================
void runModeS() {
  stopMotors();
  digitalWrite(PIN_LED_RED,    HIGH);
  digitalWrite(PIN_BUZZER,     HIGH);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
}


// =============================================================================
// FUNCIÓN: runAutonomousMovement
// Lógica compartida de movimiento autónomo para Modo A y Modo B.
// Prioridad: precipicio (IR) > obstáculo frontal (HC-SR04) > avanzar
// =============================================================================
void runAutonomousMovement() {
  bool cliffLeft  = (digitalRead(PIN_IR_LEFT)  == LOW);
  bool cliffRight = (digitalRead(PIN_IR_RIGHT) == LOW);

  if (cliffLeft && cliffRight) {
    moveBackward();
    delay(BACKUP_DURATION);
    turnRight();
    delay(TURN_DURATION);
    return;
  }

  if (cliffLeft) {
    moveBackward();
    delay(BACKUP_DURATION);
    turnRight();
    delay(TURN_DURATION);
    return;
  }

  if (cliffRight) {
    moveBackward();
    delay(BACKUP_DURATION);
    turnLeft();
    delay(TURN_DURATION);
    return;
  }

  long distanceCm = getDistance();

  if (distanceCm > 2 && distanceCm < OBSTACLE_DIST) {
    stopMotors();
    delay(100);
    moveBackward();
    delay(BACKUP_DURATION);
    turnLeft();
    delay(TURN_DURATION);
    return;
  }

  moveForward();
}


// =============================================================================
// FUNCIÓN: getDistance
// Dispara el HC-SR04 y retorna distancia en centímetros.
// Retorna 0 si la lectura está fuera de rango o es inválida.
// =============================================================================
long getDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 25000);
  if (duration == 0) return 0;
  return duration * 0.0343 / 2;
}


// =============================================================================
// HELPERS DE MOVIMIENTO
// L298N con jumpers EN activos: velocidad fija, control por IN1/IN2 únicamente.
// NO se usa analogWrite() — los pines IN solo aceptan HIGH o LOW.
//
// TODO: Si algún motor gira en dirección contraria a lo esperado,
//       intercambiar HIGH/LOW en los pines IN de ese motor.
// =============================================================================

void moveForward() {
  digitalWrite(PIN_LEFT_IN1,  HIGH);
  digitalWrite(PIN_LEFT_IN2,  LOW);
  digitalWrite(PIN_RIGHT_IN1, HIGH);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}

void moveBackward() {
  digitalWrite(PIN_LEFT_IN1,  LOW);
  digitalWrite(PIN_LEFT_IN2,  HIGH);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, HIGH);
}

void turnRight() {
  digitalWrite(PIN_LEFT_IN1,  HIGH);
  digitalWrite(PIN_LEFT_IN2,  LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, HIGH);
}

void turnLeft() {
  digitalWrite(PIN_LEFT_IN1,  LOW);
  digitalWrite(PIN_LEFT_IN2,  HIGH);
  digitalWrite(PIN_RIGHT_IN1, HIGH);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}

void stopMotors() {
  digitalWrite(PIN_LEFT_IN1,  LOW);
  digitalWrite(PIN_LEFT_IN2,  LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}


// =============================================================================
// HELPER: clearAlerts
// Apaga todos los LEDs y el buzzer. Estado visual neutro.
// =============================================================================
void clearAlerts() {
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_BUZZER,     LOW);
}
