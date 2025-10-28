# HOW TO START THE SIMULATOR
Follow these steps to set up and run the line follower simulation:
1. **Download and Install CoppeliaSim Edu Edition**  
   [CoppeliaSim Edu Edition](https://www.coppeliarobotics.com/)

2. **Install Python**  
   Make sure [Python](https://www.python.org/downloads/) is installed on your system.

3. **Download the Repository**  
   [tir-lineFollower GitHub Repository](https://github.com/IRS-tecMty/tir-lineFollower)

4. **Install Required Python Libraries**  
   Run the following command in your terminal:
   ```bash
   pip install coppeliasim_zmqremoteapi_client numpy pynput
   ```

5. **Open the Simulation File in CoppeliaSim**  
   Open:
   ```
   tir-lineFollower\CoppeliaSim2\linefollower.ttt
   ```

6. **Run the Python Script**  
   Execute:
   ```
   tir-lineFollower\CoppeliaSim2\line_follower.py
   ```
   The robot should now start moving in the CoppeliaSim window.




## Hello world to ESP32-C6 Dev Module


### Install the ESP32-C6 Dev Module Board in Arduino by **Expressif**


1.-Go to your IDE Arduino-> Tools -> Board -> Board Manager 

2.-Search **esp32** by **Exoressif Systems** 

3.-Install the latest version of it


### Install NeoPixel

1.- Go to your IDE Arduino-> Tools -> Manage Libraries
2.-Search **Adafruit NeoPixel** by Adafruit
3.-Install the latest version


Use the next script to test it


```
#include <Adafruit_NeoPixel.h>

#define LED_PIN       8    // pin del LED integrado
#define NUMPIXELS     1    // un solo LED

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  pixels.show(); // Apaga inicialmente
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // rojo
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // azul
  pixels.show();
  delay(2500);
  pixels.setPixelColor(0,pixels.Color(255,255,255));
  pixels.show();
  delay(500);
}

```


The script to try the motors

````
// ==========================================================
// PINES DE CONTROL DE MOTORES (DRV8833 en modo IN/IN)
// Mapeo basado en el esquemático (DRV8833)
// ==========================================================
// Motor Izquierdo (Motor A)
// Motor Izquierdo (Motor A)
#define MA1_PIN 2 // Pin MA1 (GPIO 18)
#define MA2_PIN 3 // Pin MA2 (GPIO 19)

// Motor Derecho (Motor B)
#define MB1_PIN 20 // Pin MB1 (GPIO 20)
#define MB2_PIN 21 // Pin MB2 (GPIO 21)

// ==========================================================
// CONFIGURACIÓN
// ==========================================================
// ¡ADVERTENCIA! analogWrite en ESP32 usa una resolución por defecto 
// diferente a Arduino (típicamente 12 bits, lo que significa 0-4095).
// Para compatibilidad y simplicidad, mantendremos el rango 0-255, 
// lo que funcionará, pero no utilizará la resolución completa del chip.
const int MAX_SPEED = 180; 

// ==========================================================
// FUNCIONES DE CONTROL
// ==========================================================

/**
 * @brief Configura la velocidad de un motor usando analogWrite.
 * @param pin_1 Pin GPIO para la dirección 1.
 * @param pin_2 Pin GPIO para la dirección 2.
 * @param speed Velocidad (0 a MAX_SPEED). Positivo = Adelante, Negativo = Atrás.
 * si quiero que vaya mas lento un pulso mas grande 
 */
void set_motor_speed(int pin_1, int pin_2, int speed) {
    // Asegurar que la velocidad esté dentro del rango [-MAX_SPEED, MAX_SPEED]
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed > 0) { // Movimiento Hacia Adelante
        // Pin 1: PWM (Velocidad)
        analogWrite(pin_1, speed);
        // Pin 2: LOW (Dirección opuesta apagada)
        digitalWrite(pin_2, LOW); 
    } else if (speed < 0) { // Movimiento Hacia Atrás
        // Pin 1: LOW (Dirección opuesta apagada)
        digitalWrite(pin_1, LOW); 
        // Pin 2: PWM (Velocidad en valor absoluto)
        analogWrite(pin_2, abs(speed));
    } else { // Detener (Frenado por corto circuito en DRV8833)
        // Ambos pines a LOW para detener la rotación
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
    }
}

void mover_atras(int speed) {
    // Ambos motores giran hacia adelante (velocidad positiva)
    set_motor_speed(MA1_PIN, MA2_PIN, speed);
    set_motor_speed(MB1_PIN, MB2_PIN, speed);
}

void mover_adelante(int speed) {
    // Ambos motores giran hacia atrás (velocidad negativa)
    set_motor_speed(MA1_PIN, MA2_PIN, -speed);
    set_motor_speed(MB1_PIN, MB2_PIN, -speed);
}

void girar_izquierda(int speed) {
    // Motor Izquierdo (A) detenido o lento, Motor Derecho (B) rápido
    set_motor_speed(MA1_PIN, MA2_PIN, speed * 0.2); // Gira más suave
    set_motor_speed(MB1_PIN, MB2_PIN, speed);
}

void girar_derecha(int speed) {
    // Motor Izquierdo (A) rápido, Motor Derecho (B) detenido o lento
    set_motor_speed(MA1_PIN, MA2_PIN, speed);
    set_motor_speed(MB1_PIN, MB2_PIN, speed * 0.2); // Gira más suave
}

void frenar() {
    set_motor_speed(MA1_PIN, MA2_PIN, 0);
    set_motor_speed(MB1_PIN, MB2_PIN, 0);
}

// ==========================================================
// CÓDIGO ARDUINO SETUP Y LOOP
// ==========================================================

void setup() {
    Serial.begin(115200);
    
    // Solo necesitamos configurar los pines como salidas
    pinMode(MA1_PIN, OUTPUT);
    pinMode(MA2_PIN, OUTPUT);
    pinMode(MB1_PIN, OUTPUT);
    pinMode(MB2_PIN, OUTPUT);

    Serial.println("Motores configurados con analogWrite. Secuencia de prueba.");
}

void loop() {
    int current_speed = 140; // Velocidad de prueba (entre 0 y 255)

    Serial.println("Adelante..");
    mover_adelante(current_speed);
    delay(1000);

    Serial.println("Derecha..");
    girar_derecha(current_speed);
    delay(500);

    Serial.println("Izquierda..");
    girar_izquierda(current_speed);
    delay(500);

    Serial.println("Pausa...");
    frenar();
    delay(2000);

    
    
    /*Serial.println("Giro Derecha...");
    girar_derecha(current_speed);
    delay(2000);*/
    
}
````
Script para recopilar los valores del sensor

```
#include <Arduino.h>

// === Your pins ===
constexpr int PIN_S0 = 4;  // LSB
constexpr int PIN_S1 = 5;
constexpr int PIN_S2 = 0;  // MSB
constexpr int PIN_Y  = 1;  // mux common output → digital read

// --- tweak if needed ---
constexpr uint32_t SETTLE_US = 5;   // allow mux to settle after switching
constexpr uint32_t ROW_DELAY_MS = 10; // delay between full-table prints

static inline void selectChannel(uint8_t ch) {
  // ch = 0..7 (bit0→S0, bit1→S1, bit2→S2)
  digitalWrite(PIN_S0, (ch >> 0) & 1);
  digitalWrite(PIN_S1, (ch >> 1) & 1);
  digitalWrite(PIN_S2, (ch >> 2) & 1);
}

void setup() {
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_Y,  INPUT);   // external pull-ups present

  // start at channel 0
  digitalWrite(PIN_S0, LOW);
  digitalWrite(PIN_S1, LOW);
  digitalWrite(PIN_S2, LOW);

  Serial.begin(115200);
  // optional: uncomment if you want a header
  // Serial.println(F("ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7"));
}

void loop() {
  int v[8];

  // scan channels 0..7
  for (uint8_t ch = 0; ch < 8; ++ch) {
    selectChannel(ch);
    delayMicroseconds(SETTLE_US);
    v[ch] = digitalRead(PIN_Y);  // read once, as requested
  }

  // print one row with the 8 readings (CSV)
  Serial.print(v[0]); Serial.print(',');
  Serial.print(v[1]); Serial.print(',');
  Serial.print(v[2]); Serial.print(',');
  Serial.print(v[3]); Serial.print(',');
  Serial.print(v[4]); Serial.print(',');
  Serial.print(v[5]); Serial.print(',');
  Serial.print(v[6]); Serial.print(',');
  Serial.println(v[7]);

  // short pause before next table
  delay(ROW_DELAY_MS);
}

```

