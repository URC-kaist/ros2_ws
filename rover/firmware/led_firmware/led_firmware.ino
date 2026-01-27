#include <Arduino.h>/private/var/folders/zw/53_149hx0wsbn1lp2xfb0yq00000gn/T/.arduinoIDE-unsaved20251020-3294-11xzl5q.jo94/Blink/Blink.ino

const int BUTTON_PIN = D1;    // 푸시 버튼

// 나중에 MOSFET 제어용 핀도 쓰고 싶으면 예:
// const int MOSFET_PIN = D2;

bool sleepMode = false;       // false = 깨어 있음, true = "sleep 모드" 상태

// 디바운스용 변수들
int lastReading = HIGH;
int stableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // ms

void applySleepState() {
  if (sleepMode) {
    // ----- Sleep 모드 ON -----
    digitalWrite(LED_BUILTIN, HIGH);   // 예: Sleep 모드일 때 LED ON
    // digitalWrite(MOSFET_PIN, LOW);  // 예: MOSFET 끄기 등
    Serial.println("Sleep mode: ON");
  } else {
    // ----- Sleep 모드 OFF -----
    digitalWrite(LED_BUILTIN, LOW);    // 예: Sleep 모드 아닐 때 LED OFF
    // digitalWrite(MOSFET_PIN, HIGH); // 예: MOSFET 켜기 등
    Serial.println("Sleep mode: OFF");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // 버튼: 풀업, GND로 눌리는 방식
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(MOSFET_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  // digitalWrite(MOSFET_PIN, HIGH); // 기본은 켜져있다고 가정

  applySleepState();
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  // 입력 값이 바뀌면 디바운스 타이머 리셋
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  // debounceDelay 동안 값이 안정적이면 "진짜"로 인정
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;

      // 버튼이 눌린 순간 (HIGH -> LOW 엣지에서만 토글)
      if (stableState == LOW) {
        sleepMode = !sleepMode;    // 토글
        applySleepState();
      }
    }
  }

  lastReading = reading;
}