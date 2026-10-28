#include <Arduino.h>
#include <Servo.h>

// MG996R signal wire only on D9. Power must come from a suitable external supply.
const uint8_t SERVO_PIN = 9;
const int UNDOCKED_ANGLE = 170;
const int DOCKED_ANGLE = 0;
const unsigned long SETTLE_MS = 3000;

Servo motor;
bool docked = false;
bool stateKnown = false;
bool moving = false;
bool targetDocked = false;
unsigned long startedAt = 0;

bool configured() {
  return UNDOCKED_ANGLE >= 0 && UNDOCKED_ANGLE <= 180 &&
         DOCKED_ANGLE >= 0 && DOCKED_ANGLE <= 180 &&
         UNDOCKED_ANGLE != DOCKED_ANGLE;
}

void setup() {
  Serial.begin(115200);
  if (!configured()) {
    Serial.println(F("ERROR CONFIGURE_ANGLES"));
    return;
  }
  // Do not attach or command the servo at boot. With no physical position
  // sensor, reset cannot establish whether the mechanism is docked.
  Serial.println(F("STATE UNKNOWN"));
}

void loop() {
  while (Serial.available()) {
    const char command = (char)Serial.read();
    if (command == 'D' || command == 'R') {
      if (!configured()) {
        Serial.println(F("ERROR CONFIGURE_ANGLES"));
      } else if (moving) {
        Serial.println(F("BUSY"));
      } else {
        targetDocked = command == 'D';
        if (stateKnown && docked == targetDocked) {
          Serial.print(F("STATE "));
          Serial.println(docked ? '1' : '0');
          continue;
        }
        const int targetAngle = targetDocked ? DOCKED_ANGLE : UNDOCKED_ANGLE;
        if (!motor.attached()) {
          // Servo.attach() can otherwise begin at the library's default angle
          // (normally 90 degrees). Cache the requested angle before enabling
          // PWM so a RELEASE command cannot briefly twitch toward DOCK.
          motor.write(targetAngle);
          motor.attach(SERVO_PIN);
        } else {
          motor.write(targetAngle);
        }
        startedAt = millis();
        moving = true;
        Serial.println(F("MOVING"));
      }
    } else if (command == 'S') {
      // A standard positional servo cannot be stopped reliably between positions.
      Serial.println(F("ERROR STOP_UNAVAILABLE"));
    } else if (command == 'Q') {
      if (!configured()) {
        Serial.println(F("ERROR CONFIGURE_ANGLES"));
      } else if (moving) {
        Serial.println(F("MOVING"));
      } else if (!stateKnown) {
        Serial.println(F("STATE UNKNOWN"));
      } else {
        Serial.print(F("STATE "));
        Serial.println(docked ? '1' : '0');
      }
    }
  }

  if (moving && (unsigned long)(millis() - startedAt) >= SETTLE_MS) {
    moving = false;
    docked = targetDocked;
    stateKnown = true;
    Serial.print(F("STATE "));
    Serial.println(docked ? '1' : '0');
  }
}
