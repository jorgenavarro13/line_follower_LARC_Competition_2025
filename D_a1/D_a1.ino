//PEPE Día 1 de competencia, versión funcional pero con trabas en las curvas.
// =====================================================
// SEGUIDOR DE LÍNEA CON 2 MOTORES Y 8 SENSORES (MUX)
// ESP32-C6 + DRV8833 + CD74HC151E + LM339
// Pepe – v6.2 (Boost 250 + Curvas estándar)
// =====================================================

#define MA1 18
#define MA2 19
#define MB1 20
#define MB2 21

#define MUX_S0 4
#define MUX_S1 5
#define MUX_S2 0
#define MUX_Y  1

#define LED_ON   6
#define LED_OFF  7
#define BUTTON_PIN 14

// === Parámetros de movimiento ===
float baseSpeed = 128;    // velocidad base reducida
float boostSpeed = 180;   // 🔸 velocidad boost reducida
float Kp = 85;
float Kd = 20;

const int pairCurveBoost = 20;
const int blackMajority = 5;
const int gapDelay = 70;

// === Control ===
bool started = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 300;

float lastError = 0;
float derivative = 0;

unsigned long allBlackStartTime = 0;
bool allBlackDetected = false;
bool allBlackConfirmed = false;

int ledSensorPins[8] = {8, 9, 10, 11, 12, 13, 22, 23};

// =====================================================
// PROTOTIPOS
// =====================================================
void setMotorSpeeds(int leftSpeed, int rightSpeed);
int readMuxSensor(int channel);
void readAllSensors(int sensors[8]);
void simpleLineFollower(int sensors[8]);

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("🚗 Seguidor de línea (v6.2 - Boost 250 + Curvas estándar)");

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_Y, INPUT);

  pinMode(LED_ON, OUTPUT);
  pinMode(LED_OFF, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);

  for (int i = 0; i < 8; i++) {
    pinMode(ledSensorPins[i], OUTPUT);
    digitalWrite(ledSensorPins[i], LOW);
  }

  setMotorSpeeds(0, 0);
  digitalWrite(LED_ON, LOW);
  digitalWrite(LED_OFF, HIGH);
  Serial.println("Listo. Presiona el botón para iniciar o detener.");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    started = !started;
    lastDebounceTime = millis();

    if (started) {
      Serial.println("✅ Seguidor ACTIVADO");
      digitalWrite(LED_ON, HIGH);
      digitalWrite(LED_OFF, LOW);
    } else {
      Serial.println("⛔ Seguidor DETENIDO");
      setMotorSpeeds(0, 0);
      digitalWrite(LED_ON, LOW);
      digitalWrite(LED_OFF, HIGH);
    }
  }

  lastButtonState = buttonState;
  if (!started) return;

  int sensors[8];
  readAllSensors(sensors);
  simpleLineFollower(sensors);
  delay(10);
}

// =====================================================
// FUNCIONES DE LECTURA Y MOTORES
// =====================================================
int readMuxSensor(int channel) {
  digitalWrite(MUX_S0, channel & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  delayMicroseconds(200);
  return digitalRead(MUX_Y);
}

void readAllSensors(int sensors[8]) {
  for (int i = 0; i < 8; i++) {
    sensors[i] = readMuxSensor(i);
    digitalWrite(ledSensorPins[i], sensors[i] == LOW ? HIGH : LOW);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) { analogWrite(MA1, leftSpeed); analogWrite(MA2, 0); }
  else { analogWrite(MA1, 0); analogWrite(MA2, -leftSpeed); }

  if (rightSpeed >= 0) { analogWrite(MB1, 0); analogWrite(MB2, rightSpeed); }
  else { analogWrite(MB1, -rightSpeed); analogWrite(MB2, 0); }
}

// =====================================================
// ALGORITMO DE SEGUIMIENTO DE LÍNEA
// =====================================================
void simpleLineFollower(int sensors[8]) {
  float weights[8] = {-4.5, -3.2, -1, 0, 0, 1, 3.2, 4.5};
  int activeCount = 0;
  float weightedSum = 0;
  int blackCount = 0;

  for (int i = 0; i < 8; i++) {
    if (sensors[i] == LOW) {
      activeCount++;
      blackCount++;
      weightedSum += weights[i];
    }
  }

  // === META (todos negros ≥ 0.2s) ===
  if (blackCount >= 8) {
    if (!allBlackDetected) {
      allBlackDetected = true;
      allBlackStartTime = millis();
      Serial.println("⬛ Todos los sensores en negro: conteo (0.2s)...");
    } else if (!allBlackConfirmed && millis() - allBlackStartTime >= 200) {
      allBlackConfirmed = true;
      Serial.println("🏁 META CONFIRMADA. DETENIENDO...");
      setMotorSpeeds(0, 0);

      for (int i = 0; i < 6; i++) {
        digitalWrite(LED_ON, HIGH);
        digitalWrite(LED_OFF, LOW);
        delay(100);
        digitalWrite(LED_ON, LOW);
        digitalWrite(LED_OFF, HIGH);
        delay(100);
      }
      while (true) setMotorSpeeds(0, 0);
    }
  } else {
    allBlackDetected = false;
    allBlackConfirmed = false;
  }

  if (activeCount == 0) return;

  // === PD ===
  float error = weightedSum / (activeCount * 4.5);
  derivative = error - lastError;
  lastError = error;
  float correction = (Kp * error) + (Kd * derivative);

  // === Velocidad base adaptativa ===
  float currentBaseSpeed = baseSpeed;

  // 🔹 Boost flexible: si uno o ambos sensores del centro están activos
  if ((sensors[3] == LOW) || (sensors[4] == LOW)) {
    currentBaseSpeed = boostSpeed;
  }

  // 🔸 Detección de curvas: baja la velocidad a la mitad
  bool leftPair  = (sensors[0] == LOW && sensors[1] == LOW) ||
                   (sensors[1] == LOW && sensors[2] == LOW);
  bool rightPair = (sensors[6] == LOW && sensors[7] == LOW) ||
                   (sensors[5] == LOW && sensors[6] == LOW);

  if (leftPair || rightPair) {
    currentBaseSpeed *= 0.7;
  }

  int leftSpeed  = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;

  // Curvas estándar (sin suavizado adicional)
  if (leftPair && !rightPair) {
    rightSpeed += pairCurveBoost;
  }
  if (rightPair && !leftPair) {
    leftSpeed += pairCurveBoost;
  }

  setMotorSpeeds(leftSpeed, rightSpeed);
}
