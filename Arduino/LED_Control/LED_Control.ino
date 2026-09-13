/**
 * @file LED_Control.ino
 * @brief 시리얼 명령으로 LED 2개(빨강/초록)를 제어한다 — 봄 미션(피아식별)
 *        결과를 아군(초록)/적군(빨강)으로 표시.
 *
 * led_bridge_node(dolbotz 패키지, ROS2)가 "<cmd>\n" 형태로 보내는 시리얼
 * 문자열을 그대로 파싱한다.
 *
 * 시리얼 명령:
 *   "roka"  -> 초록 LED ON, 빨강 LED OFF   (아군)
 *   "enemy" -> 빨강 LED ON, 초록 LED OFF   (적군)
 *   "none" (또는 그 외 아무 문자열)         -> 둘 다 OFF (미확정)
 *
 * 파싱은 앞뒤 공백 제거 + 소문자 변환까지 해서, led_bridge_node가 보내는
 * 형식(이미 trim+lower된 문자열)이 아니어도 어느 정도 견고하게 동작한다.
 *
 * [2026-08-30] 핀 번호/극성은 실물 배선 확인본(팀원이 확인한 실측값)으로
 * 갱신함 — 공통 애노드 RGB LED라 HIGH = OFF, LOW = ON. 보드에 파랑(D10)도
 * 나와 있지만 이 미션은 아군(초록)/적군(빨강) 2색만 쓰므로 사용하지 않음.
 */

#include <Arduino.h>

// ===== 핀 정의 (실물 배선 확인됨, 공통 애노드: HIGH = OFF) =====
#define Red_Pin    11  // 빨강 LED(적군 표시) 핀
#define Green_Pin   9  // 초록 LED(아군 표시) 핀
// D10 = 파랑 핀이 보드에 있지만 이 미션은 2색(빨강/초록)만 쓰므로 미사용.

// ===== 시리얼 버퍼 =====
#define SERIAL_BUFFER_SIZE 64
static char    serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t bufferIndex = 0;

/**
 * @brief C 문자열 앞뒤 공백을 제자리에서 제거한다.
 */
static void trim_in_place(char* s) {
  // 뒤쪽 공백 제거
  int end = strlen(s) - 1;
  while (end >= 0 && (s[end] == ' ' || s[end] == '\t' || s[end] == '\r' || s[end] == '\n')) {
    s[end--] = '\0';
  }
  // 앞쪽 공백 제거(포인터 이동 없이 제자리에서)
  int start = 0;
  while (s[start] == ' ' || s[start] == '\t') start++;
  if (start > 0) {
    int i = 0;
    while (s[start]) s[i++] = s[start++];
    s[i] = '\0';
  }
}

/**
 * @brief C 문자열을 제자리에서 소문자로 변환한다.
 */
static void tolower_in_place(char* s) {
  for (; *s; ++s) {
    if (*s >= 'A' && *s <= 'Z') *s = *s - 'A' + 'a';
  }
}

/**
 * @brief 받은 명령을 LED에 적용한다.
 * @param cmd "roka" | "enemy" | 그 외("none" 취급)
 *
 * 공통 애노드 배선이라 LOW = ON, HIGH = OFF (일반적인 능동-HIGH LED와 반대).
 */
static void applyCommand(const char* cmd) {
  if (strcmp(cmd, "roka") == 0) {
    digitalWrite(Green_Pin, LOW);
    digitalWrite(Red_Pin, HIGH);
  } else if (strcmp(cmd, "enemy") == 0) {
    digitalWrite(Green_Pin, HIGH);
    digitalWrite(Red_Pin, LOW);
  } else { // "none" 또는 그 외 인식 못 하는 문자열 -- 둘 다 끔
    digitalWrite(Green_Pin, HIGH);
    digitalWrite(Red_Pin, HIGH);
  }
}

/**
 * @brief 시리얼로 들어오는 데이터를 한 줄(개행 기준)씩 파싱한다.
 */
static void parseSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // CRLF 처리: '\r'은 무시
    if (c == '\r') continue;

    if (c == '\n') {
      // 한 줄 완성
      serialBuffer[bufferIndex] = '\0';
      bufferIndex = 0;

      // 전처리: 공백 제거 + 소문자 변환
      trim_in_place(serialBuffer);
      tolower_in_place(serialBuffer);

      if (serialBuffer[0] != '\0') {
        applyCommand(serialBuffer);
      }
    } else {
      // 버퍼에 누적
      if (bufferIndex < (SERIAL_BUFFER_SIZE - 1)) {
        serialBuffer[bufferIndex++] = c;
      } else {
        // 버퍼 오버플로 방지: 이번 줄 버리고 처음부터 다시
        bufferIndex = 0;
      }
    }
  }
}

/**
 * @brief 시작 시 한 번 실행 — 핀 모드/시리얼 초기화.
 */
void setup() {
  pinMode(Red_Pin, OUTPUT);
  pinMode(Green_Pin, OUTPUT);
  // 공통 애노드: HIGH = OFF -- 시작 시 둘 다 꺼진 상태로.
  digitalWrite(Red_Pin, HIGH);
  digitalWrite(Green_Pin, HIGH);

  Serial.begin(115200);
  // while (!Serial) { ; } // 시리얼 연결 대기가 필요하면 주석 해제
  delay(100);
  Serial.println(F("SERIAL ready (send: roka | enemy | none)"));
}

/**
 * @brief 메인 루프 — 계속 시리얼 파서만 호출.
 */
void loop() {
  parseSerial();
}
