const int IN1 = 9;
const int IN2 = 8;
const int ENA = 10;

const int IN3 = 3;
const int IN4 = 4;
const int ENB = 5;

const unsigned long MOTOR_RUN_TIME_MS = 3000;
const int MOTOR_POWER = 255;

char commandBuffer[8];
size_t commandLength = 0;
bool discardingCommand = false;
bool motorRunning = false;
unsigned long motorStartedAt = 0;

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  motorRunning = false;
}

void runMotors(bool reverse) {
  digitalWrite(IN1, reverse ? LOW : HIGH);
  digitalWrite(IN2, reverse ? HIGH : LOW);
  digitalWrite(IN3, reverse ? LOW : HIGH);
  digitalWrite(IN4, reverse ? HIGH : LOW);

  analogWrite(ENA, MOTOR_POWER);
  analogWrite(ENB, MOTOR_POWER);

  motorStartedAt = millis();
  motorRunning = true;
}

void applyCommand() {
  // ROS bridge sends one ASCII digit followed by a newline.
  if (commandLength != 1) {
    Serial.println(F("ERR expected 0, 1, or 2"));
    return;
  }

  switch (commandBuffer[0]) {
    case '0':
      stopMotors();
      Serial.println(F("STOP"));
      break;
    case '1':
      runMotors(false);
      Serial.println(F("FORWARD 3000ms"));
      break;
    case '2':
      runMotors(true);
      Serial.println(F("REVERSE 3000ms"));
      break;
    default:
      Serial.println(F("ERR expected 0, 1, or 2"));
      break;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (discardingCommand) {
        Serial.println(F("ERR command too long"));
      } else {
        applyCommand();
      }
      commandLength = 0;
      discardingCommand = false;
      continue;
    }

    if (discardingCommand) {
      continue;
    }
    if (commandLength < sizeof(commandBuffer)) {
      commandBuffer[commandLength++] = c;
    } else {
      // Discard an overlong/malformed line. It cannot start a motor.
      commandLength = 0;
      discardingCommand = true;
    }
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Always boot stopped. A valid serial command is required to move.
  stopMotors();

  Serial.begin(115200);
  Serial.println(F("Gripper_BOX ready (0=stop, 1=forward, 2=reverse)"));
}

void loop() {
  readSerialCommands();

  // Unsigned subtraction remains correct when millis() wraps around.
  if (motorRunning &&
      (unsigned long)(millis() - motorStartedAt) >= MOTOR_RUN_TIME_MS) {
    stopMotors();
    Serial.println(F("STOP timeout"));
  }
}
