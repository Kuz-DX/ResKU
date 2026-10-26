#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// Upload this sketch, then use Arduino IDE Serial Monitor at 115200 baud.
// MG996R signal: D9. Power: external 5-6 V supply with common ground.
const uint8_t SERVO_PIN = 9;
const int START_ANGLE = 90;
Servo motor;
int commandedAngle = START_ANGLE;
bool attached = false;
char input[16];
uint8_t inputLength = 0;

void report() {
  Serial.print(F("ANGLE "));
  Serial.print(commandedAngle);
  Serial.println(attached ? F(" (commanded)") : F(" (detached)"));
}

void setAngle(int angle) {
  if (angle < 0 || angle > 180) {
    Serial.println(F("ERROR: angle must be 0..180"));
    return;
  }
  commandedAngle = angle;
  if (!attached) {
    // The first movement command enables the servo. It does not move on boot.
    motor.write(commandedAngle);
    motor.attach(SERVO_PIN);
    attached = true;
  } else {
    motor.write(commandedAngle);
  }
  report();
}

void handleCommand(char *command) {
  if (strcmp(command, "+") == 0) {
    setAngle(constrain(commandedAngle + 1, 0, 180));
  } else if (strcmp(command, "-") == 0) {
    setAngle(constrain(commandedAngle - 1, 0, 180));
  } else if (strcmp(command, "]") == 0) {
    setAngle(constrain(commandedAngle + 5, 0, 180));
  } else if (strcmp(command, "[") == 0) {
    setAngle(constrain(commandedAngle - 5, 0, 180));
  } else if (strcmp(command, "?") == 0) {
    report();
  } else if (strcmp(command, "off") == 0) {
    motor.detach();
    attached = false;
    report();
  } else if (command[0] != '\0') {
    char *end;
    long angle = strtol(command, &end, 10);
    if (end != command && *end == '\0' && angle >= 0 && angle <= 180) {
      setAngle((int)angle);
    } else {
      Serial.println(F("ERROR: send 0..180, +, -, [, ], ?, or off"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("MG996R D9 calibration ready"));
  Serial.println(F("Send 0..180, +, -, [, ], ?, or off; press Send/Enter"));
  report();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      input[inputLength] = '\0';
      handleCommand(input);
      inputLength = 0;
    } else if (inputLength < sizeof(input) - 1) {
      input[inputLength++] = c;
    } else {
      inputLength = 0;
      Serial.println(F("ERROR: command too long"));
    }
  }
}
