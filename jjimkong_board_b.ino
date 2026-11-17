// 찜콩 보드 B - 잠금, 비밀번호

#include <Wire.h>
#include <Stepper.h>

const byte PIN_BTN[4] = {5, 4, 3, 2};   // 버튼 1~4
const byte PIN_BUZZ  = 10;
const byte PIN_LED_B = 11;
const byte PIN_LED_R = 12;

const byte MY_ADDR = 8;   // I2C 주소

const byte ST_IDLE = 0, ST_ARMED = 1, ST_WARN = 2, ST_ALARM = 3;

const byte PASSWORD[4] = {2, 4, 1, 3};
const int LOCK_STEPS = 512;              // 90도
const unsigned long PASS_TIMEOUT = 5000;

// 28BYJ-48은 IN1, IN3, IN2, IN4 순서로 넣어야 함
Stepper motor(2048, A0, A2, A1, A3);

volatile byte stateFromA = ST_IDLE;
volatile byte unlockFlag = 0;
byte state = ST_IDLE;
bool locked = false;
bool lastBtn[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long tBtn[4] = {0, 0, 0, 0};
byte passIndex = 0;
unsigned long tPass = 0, tFlash = 0;
bool flashRed = false;

void onReceive(int n) {
  while (Wire.available()) stateFromA = Wire.read();
  (void)n;
}
void onRequest() {
  Wire.write(unlockFlag);
  unlockFlag = 0;
}

void motorOff() {   // 발열 방지
  digitalWrite(A0, LOW); digitalWrite(A1, LOW);
  digitalWrite(A2, LOW); digitalWrite(A3, LOW);
}
void setLock(bool want) {
  if (want == locked) return;
  motor.step(want ? LOCK_STEPS : -LOCK_STEPS);
  motorOff();
  locked = want;
}

void passOk() {
  passIndex = 0;
  unlockFlag = 1;
  stateFromA = ST_IDLE;    // A 없어도 바로 해제
  tone(PIN_BUZZ, 1800, 120);
}
void pressed(byte num) {
  tPass = millis();
  if (num == PASSWORD[passIndex]) {
    passIndex++;
    tone(PIN_BUZZ, 1200, 40);
    if (passIndex == 4) passOk();
  } else {
    passIndex = (num == PASSWORD[0]) ? 1 : 0;
    tone(PIN_BUZZ, 300, 150);
  }
}
void readButtons() {
  for (byte i = 0; i < 4; i++) {
    bool now = digitalRead(PIN_BTN[i]);

    if (now != lastBtn[i] && millis() - tBtn[i] > 30) {   // 디바운스
      tBtn[i] = millis();
      lastBtn[i] = now;
      if (now == LOW) pressed(i + 1);
    }
  }
  if (passIndex > 0 && millis() - tPass > PASS_TIMEOUT) passIndex = 0;
}

void alarmEffects() {
  if (state == ST_ALARM) {
    if (millis() - tFlash >= 200) {
      tFlash = millis();
      flashRed = !flashRed;
      digitalWrite(PIN_LED_R, flashRed);
      digitalWrite(PIN_LED_B, !flashRed);
      tone(PIN_BUZZ, flashRed ? 1600 : 1000);
    }
  }
}

void setup() {
  for (byte i = 0; i < 4; i++) pinMode(PIN_BTN[i], INPUT_PULLUP);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  motor.setSpeed(10);   // 15 넘으면 탈조
  motorOff();

  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.begin(9600);
  tone(PIN_BUZZ, 1200, 100);
}

void loop() {
  readButtons();

  byte s = stateFromA;
  if (s != state) {
    if (state == ST_ALARM) {
      noTone(PIN_BUZZ);
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_B, LOW);
    }
    state = s;
    Serial.print(F("state=")); Serial.println(state);
  }

  setLock(state != ST_IDLE);
  alarmEffects();
}
