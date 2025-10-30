/*
  Prueba de Llantas (Motores A y B)
  
  Toma como referencia los pines del código v6.2.
  - Usa el botón en GPIO 14 para cambiar de estado.
  - Controla los motores A y B (pines 18, 19, 20, 21).
*/

// --- Pines de Motores (de tu código) ---
#define MA1 18 // Motor A (Izquierdo)
#define MA2 19
#define MB1 20 // Motor B (Derecho)
#define MB2 21

// --- Pin del Botón (de tu código) ---
#define BUTTON_PIN 14

// --- Variables de control ---
int testState = 0; // Estado actual de la prueba
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200; // Tiempo de espera (debounce)


// =====================================================
// PROTOTIPOS DE FUNCIONES 
// (Le dicen al compilador que estas funciones existen)
// =====================================================
void runTestState(int state);
void allStop();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();


// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("🚗 Prueba de Llantas Iniciada");

  // Configurar todos los pines de motor como SALIDA
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);

  // Configurar el botón como ENTRADA (con pull-up interno)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Asegurarse de que los motores estén detenidos al inicio
  allStop(); // <- Ahora esta llamada funcionará
  Serial.println("Estado 0: DETENIDO. Presiona el botón para iniciar.");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // Leer el estado del botón (Lógica de tu código)
  bool buttonState = digitalRead(BUTTON_PIN);
  
  // Detectar cuando se presiona el botón (flanco de bajada)
  if (lastButtonState == HIGH && buttonState == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis(); // Reiniciar el temporizador de debounce
    
    // Avanzar al siguiente estado de prueba
    testState++;
    if (testState > 4) {
      testState = 0; // Volver al inicio
    }

    // Ejecutar la acción del estado actual
    runTestState(testState);
  }
  
  lastButtonState = buttonState;
} // <-- LLAVE DE CIERRE PARA loop()


// =====================================================
// FUNCIÓN DE CONTROL DE ESTADOS
// =====================================================
void runTestState(int state) {
  switch (state) {
    case 0:
      Serial.println("Estado 0: DETENIDO");
      allStop();
      break;
    case 1:
      Serial.println("Estado 1: ADELANTE");
      moveForward();
      break;
    case 2:
      Serial.println("Estado 2: ATRÁS");
      moveBackward();
      break;
    case 3:
      Serial.println("Estado 3: GIRAR DERECHA");
      turnRight();
      break;
    case 4:
      Serial.println("Estado 4: GIRAR IZQUIERDA");
      turnLeft();
      break;
  }
}

// =====================================================
// FUNCIONES BÁSICAS DE MOVIMIENTO
// (Aquí estaban las definiciones que faltaban)
// =====================================================

// Basado en la lógica de tu función setMotorSpeeds:
// Motor A (Izquierda): MA1=HIGH -> Adelante
// Motor B (Derecha):   MB2=HIGH -> Adelante

void allStop() {
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, LOW);
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, LOW);
}

void moveForward() {
  // Motor A Adelante
  digitalWrite(MA1, HIGH);
  digitalWrite(MA2, LOW);
  // Motor B Adelante
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, HIGH);
}

void moveBackward() {
  // Motor A Atrás
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, HIGH);
  // Motor B Atrás
  digitalWrite(MB1, HIGH); // <- ¡Error corregido! (Antes 'digitalVrite')
  digitalWrite(MB2, LOW);
}

void turnRight() {
  // Motor A Adelante
  digitalWrite(MA1, HIGH);
  digitalWrite(MA2, LOW);
  // Motor B Atrás
  digitalWrite(MB1, HIGH);
  digitalWrite(MB2, LOW);
}

void turnLeft() {
  // Motor A Atrás
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, HIGH);
  // Motor B Adelante
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, HIGH);
}
