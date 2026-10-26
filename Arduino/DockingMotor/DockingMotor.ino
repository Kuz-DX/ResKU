#include <Arduino.h>

// Motor driver: D9 = PWM/enable, D8 = direction. Do not connect a motor directly to D9.
const uint8_t PWM_PIN = 9;
const uint8_t DIR_PIN = 8;
const uint8_t MOTOR_PWM = 250;
const unsigned long RUN_MS = 3000;

bool docked = false;
bool running = false;
bool targetDocked = false;
unsigned long startedAt = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);
  Serial.begin(115200);
  Serial.println(F("STATE 0"));
}

void loop() {
  while (Serial.available()) {
    char command = (char)Serial.read();
    if (command == 'T') {
      if (running) {
        Serial.println(F("BUSY"));
      } else {
        targetDocked = !docked;
        digitalWrite(DIR_PIN, targetDocked ? HIGH : LOW);
        analogWrite(PWM_PIN, MOTOR_PWM);
        startedAt = millis();
        running = true;
        Serial.println(F("MOVING"));
      }
    } else if (command == 'S') {
      analogWrite(PWM_PIN, 0);
      running = false;
      Serial.println(F("STOPPED"));
    } else if (command == 'Q') {
      if (running) {
        Serial.println(F("MOVING"));
      } else {
        Serial.print(F("STATE "));
        Serial.println(docked ? '1' : '0');
      }
    }
  }

  if (running && (unsigned long)(millis() - startedAt) >= RUN_MS) {
    analogWrite(PWM_PIN, 0);
    running = false;
    docked = targetDocked;
    Serial.print(F("STATE "));
    Serial.println(docked ? '1' : '0');
  }
}
