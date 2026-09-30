const int IN1 = 9;
const int IN2 = 8;
const int ENA = 10;

const int IN3 = 3;
const int IN4 = 4;
const int ENB = 5;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 모터 A 정방향
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // 모터 B 정방향
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // 최대 출력
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void loop() {
}