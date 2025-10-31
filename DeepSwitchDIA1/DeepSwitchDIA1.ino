/*
  =====================================================
  SEGUIDOR DE LÍNEA CON 3 MODOS (DIP SWITCH)
  ESP32-C6 + DRV8833 + CD74HC151E + LM339
  
  Modos seleccionables por Dip Switch:
  - GPIO 6 (SW3-1)
  - GPIO 7 (SW3-2)
  
  Botón de inicio/paro:
  - GPIO 14 (SW2)
  =====================================================
*/

// --- Pines de Motores ---
#define MA1 18
#define MA2 19
#define MB1 20
#define MB2 21

// --- Pines de Sensores (MUX) ---
#define MUX_S0 4
#define MUX_S1 5
#define MUX_S2 0
#define MUX_Y  1

// --- Pines de Control y LEDs ---
#define LED_ON     6 // Conflicto: Usado para Dip Switch
#define LED_OFF    7 // Conflicto: Usado para Dip Switch
#define BUTTON_PIN 14

// --- NUEVOS: Pines de Selección de Modo (SW3) ---
#define MODE_PIN_A 6 // GPIO 6
#define MODE_PIN_B 7 // GPIO 7

// === Parámetros Constantes ===
const int pairCurveBoost = 20;
const int blackMajority = 5;
const int gapDelay = 70;

// === Estructura para Parámetros de Modo ===
struct ModeParameters {
  const char* modeName;
  float baseSpeed;
  float boostSpeed;
  float Kp;
  float Kd;
  float weights[8];
  float curveSlowdown;
};

// --- Definición Modo 1: SEGURA (de Segura.ino) ---
ModeParameters params_segura = {
  "🛡️ MODO SEGURA",
  120, // baseSpeed
  130, // boostSpeed
  85,  // Kp
  12,  // Kd
  {-6.8, -6.0, -3.8, 1, 1, 3.8, 6.0 , 6.8}, // weights
  0.75 // curveSlowdown
};

// --- Definición Modo 2: MEDIA (de Media.ino) ---
ModeParameters params_media = {
  "🚗 MODO MEDIA",
  140, // baseSpeed
  165, // boostSpeed
  85,  // Kp
  12,  // Kd
  {-6.8, -6.0, -3.8, 1, 1, 3.8, 6.0 , 6.8}, // weights
  0.75 // curveSlowdown
};

// --- Definición Modo 3: RÁPIDA (¡PLACEHOLDER!) ---
// ¡¡AJUSTA ESTOS VALORES CON LOS DE TU ARCHIVO Rapida.ino!!
ModeParameters params_rapida = {
  "⚡ MODO RÁPIDA (Placeholder)",
  170, // baseSpeed
  180, // boostSpeed
  85,  // Kp
  12,  // Kd
  {-6.7, -6.5, -4.0, 1, 1, 4.0, 6.5 , 6.7}, // weights (usando los de Media)
  1.0  // curveSlowdown (sin frenado)
};

// Puntero a los parámetros del modo actual
ModeParameters* currentParams;

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

// OJO: Los pines 8, 9, 10, 11, 12, 13, 22, 23
// están definidos para los LEDs de los sensores.
int ledSensorPins[8] = {8, 9, 10, 11, 12, 13, 22, 23};


// =====================================================
// PROTOTIPOS
// =====================================================
void setMotorSpeeds(int leftSpeed, int rightSpeed);
int readMuxSensor(int channel);
void readAllSensors(int sensors[8]);
void simpleLineFollower(int sensors[8]);
void selectMode(); // Nueva función

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("==========================================");
  Serial.println("🚗 Seguidor de línea Multi-Modo");

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_Y, INPUT);

  // Configurar pines de control (Botón y Dip Switches)
  // Usamos INPUT y digitalWrite(HIGH) para activar pull-up (como en tu código)
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  
  pinMode(MODE_PIN_A, INPUT);
  digitalWrite(MODE_PIN_A, HIGH);
  pinMode(MODE_PIN_B, INPUT);
  digitalWrite(MODE_PIN_B, HIGH);

  // LEDs de sensores
  for (int i = 0; i < 8; i++) {
    pinMode(ledSensorPins[i], OUTPUT);
    digitalWrite(ledSensorPins[i], LOW);
  }

  // LEDs de estado (¡¡ATENCIÓN!! Tus archivos originales
  // usaban pines 6 y 7, que ahora son para los switches.
  // Si tienes LEDs de estado, debes moverlos a otros pines
  // y actualizar los #define. Por ahora, los comento.
  // pinMode(LED_ON, OUTPUT);
  // pinMode(LED_OFF, OUTPUT);
  // digitalWrite(LED_ON, LOW);
  // digitalWrite(LED_OFF, HIGH);

  setMotorSpeeds(0, 0);

  // Seleccionar el modo de manejo basado en los switches
  selectMode(); 
  
  Serial.println("Listo. Presiona el botón (GPIO 14) para iniciar.");
  Serial.println("==========================================");
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
      // digitalWrite(LED_ON, HIGH);
      // digitalWrite(LED_OFF, LOW);
    } else {
      Serial.println("⛔ Seguidor DETENIDO");
      setMotorSpeeds(0, 0);
      // digitalWrite(LED_ON, LOW);
      // digitalWrite(LED_OFF, HIGH);
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
// ¡NUEVO! FUNCIÓN DE SELECCIÓN DE MODO
// =====================================================
void selectMode() {
  // Leer el estado de los switches. LOW = ON (presionado)
  bool modeA_ON = (digitalRead(MODE_PIN_A) == LOW);
  bool modeB_ON = (digitalRead(MODE_PIN_B) == LOW);

  if (modeA_ON && !modeB_ON) {
    // Switch 1 ON, Switch 2 OFF
    currentParams = &params_media;
  } 
  else if (!modeA_ON && modeB_ON) {
    // Switch 1 OFF, Switch 2 ON
    currentParams = &params_rapida;
  } 
  else {
    // Ambos OFF o ambos ON (default)
    currentParams = &params_segura;
  }
  
  // Imprimir el modo seleccionado al Monitor Serial
  Serial.print("Modo seleccionado: ");
  Serial.println(currentParams->modeName);
}


// =====================================================
// FUNCIONES DE LECTURA Y MOTORES (Sin cambios)
// =====================================================
int readMuxSensor(int channel) {
  digitalWrite(MUX_S0, channel & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  delayMicroseconds(130);
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
// ALGORITMO DE SEGUIMIENTO (¡MODIFICADO!)
// =====================================================
void simpleLineFollower(int sensors[8]) {
  // Los parámetros (weights, Kp, Kd, etc.)
  // ahora se leen desde el puntero global 'currentParams'
  
  int activeCount = 0;
  float weightedSum = 0;
  int blackCount = 0;

  for (int i = 0; i < 8; i++) {
    if (sensors[i] == LOW) {
      activeCount++;
      blackCount++;
      weightedSum += currentParams->weights[i]; // <--- MODIFICADO
    }
  }

  // === META (todos negros ≥ 0.2s) ===
  if (blackCount >= 7) {
    if (!allBlackDetected) {
      allBlackDetected = true;
      allBlackStartTime = millis();
      Serial.println("⬛ Todos los sensores en negro: conteo...");
    } else if (!allBlackConfirmed && millis() - allBlackStartTime >= 120) { // Usando un valor medio
      allBlackConfirmed = true;
      Serial.println("🏁 META CONFIRMADA. DETENIENDO...");
      setMotorSpeeds(0, 0);

      for (int i = 0; i < 6; i++) {
        // ... parpadeo de LEDs si se reasignan los pines ...
        delay(100);
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
  
  // Usa Kp y Kd del modo actual
  float correction = (currentParams->Kp * error) + (currentParams->Kd * derivative); // <--- MODIFICADO

  // === Velocidad base adaptativa ===
  float currentBaseSpeed = currentParams->baseSpeed; // <--- MODIFICADO

  // 🔹 Boost flexible
  if ((sensors[3] == LOW) || (sensors[4] == LOW)) {
    currentBaseSpeed = currentParams->boostSpeed; // <--- MODIFICADO
  }

  // 🔸 Detección de curvas
  bool leftPair  = (sensors[0] == LOW && sensors[1] == LOW) ||
                   (sensors[1] == LOW && sensors[2] == LOW);
  bool rightPair = (sensors[6] == LOW && sensors[7] == LOW) ||
                   (sensors[5] == LOW && sensors[6] == LOW);

  if (leftPair || rightPair) {
    currentBaseSpeed *= currentParams->curveSlowdown; // <--- MODIFICADO
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
