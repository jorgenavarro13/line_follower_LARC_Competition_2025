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
