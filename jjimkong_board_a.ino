// 찜콩 보드 A - 감지
// 라이브러리: LiquidCrystal I2C

#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

const byte PIN_LED_G = 2;
const byte PIN_BT_RX = 3;    // HC-06 TXD
const byte PIN_LED_Y = 4;
const byte PIN_LED_R = 5;
const byte PIN_BUZZ  = 8;
const byte PIN_SERVO = 9;
const byte PIN_ECHO  = 10;
const byte PIN_BT_TX = 11;   // HC-06 RXD, 1k/2k 분압
const byte PIN_TRIG  = 12;
const byte PIN_POT   = A0;

const byte ADDR_BOARD_B = 8;
const byte ADDR_MPU     = 0x68;
const byte ADDR_LCD     = 0x27;   // 안 나오면 0x3F

// 대기, 경계, 경고, 경보
const byte ST_IDLE  = 0;
const byte ST_ARMED = 1;
const byte ST_WARN  = 2;
const byte ST_ALARM = 3;

unsigned long awayTimeout = 5000;         // 신호 끊김 판단(ms)
const unsigned long WARN_TO_ALARM = 3000;
const unsigned long WARN_CLEAR    = 2000;
const float ACC_LIMIT = 3000.0;           // 16384 = 1g
const int FLAG_DOWN = 10;
const int FLAG_UP   = 100;

SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);
LiquidCrystal_I2C lcd(ADDR_LCD, 16, 2);
Servo flag;

byte state = ST_IDLE;
bool everConnected = false;
bool manualArm = false;
bool mpuOk = false;
unsigned long lastBeat = 0;
unsigned long warnSince = 0;
unsigned long farSince = 0;
unsigned long tSensor = 0, tSound = 0, tLink = 0, tReport = 0;
int distCm = 999;
int warnDist = 40;
byte nearCount = 0;
byte moveCount = 0;
int warnTotal = 0, alarmTotal = 0;
int16_t baseX = 0, baseY = 0, baseZ = 0;
bool sirenHigh = false;
unsigned long tFlag = 0;
bool flagBusy = false;
char message[17] = "BE RIGHT BACK";
char btBuf[24];  byte btLen = 0;
char pcBuf[24];  byte pcLen = 0;

// 움직일 때만 신호를 주고 끊음 (블루투스 통신 중 서보 떨림 방지)
void moveFlag(int angle) {
  flag.attach(PIN_SERVO);
  flag.write(angle);
  tFlag = millis();
  flagBusy = true;
}

void tickFlag() {
  if (flagBusy && millis() - tFlag > 700) {
    flag.detach();
    flagBusy = false;
  }
}

int readDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long t = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (t == 0) return 999;
  return (int)(t * 0.0343 / 2.0);   // 왕복
}

bool mpuBegin() {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x6B);
  Wire.write(0);      // 절전 해제
  return Wire.endTransmission() == 0;
}

bool readAccel(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDR_MPU, (byte)6) != 6) return false;
  byte d[6];
  for (byte i = 0; i < 6; i++) d[i] = Wire.read();
  x = (int16_t)((d[0] << 8) | d[1]);
  y = (int16_t)((d[2] << 8) | d[3]);
  z = (int16_t)((d[4] << 8) | d[5]);
  return true;
}

void showLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (state == ST_IDLE) lcd.print("JJIMKONG");
  else                  lcd.print(message);
  lcd.setCursor(0, 1);
  if (state == ST_IDLE)       lcd.print("READY");
  else if (state == ST_ARMED) lcd.print("SEAT TAKEN");
  else if (state == ST_WARN)  lcd.print("STEP BACK!");
  else                        lcd.print("!!! ALARM !!!");
}

void showLeds() {
  digitalWrite(PIN_LED_G, state == ST_IDLE || state == ST_ARMED);
  digitalWrite(PIN_LED_Y, state == ST_WARN);
  digitalWrite(PIN_LED_R, state == ST_ALARM);
}

void setState(byte s) {
  if (s == state) return;
  state = s;
  noTone(PIN_BUZZ);
  if (s == ST_WARN)  { warnTotal++;  warnSince = millis(); farSince = 0; }
  if (s == ST_ALARM) { alarmTotal++; }
  showLeds();
  showLcd();
  Serial.print(F("state=")); Serial.println(state);
}

void arm() {
  warnTotal = 0; alarmTotal = 0; nearCount = 0; moveCount = 0;
  if (mpuOk) readAccel(baseX, baseY, baseZ);
  moveFlag(FLAG_UP);
  setState(ST_ARMED);
}

void disarm(bool keepLink) {
  bool wasActive = (state != ST_IDLE);
  moveFlag(FLAG_DOWN);
  setState(ST_IDLE);
  manualArm = false;
  if (!keepLink) everConnected = false;   // 다음 신호 전까지 자동 경계 안 함
  if (wasActive) {
    bt.print(F("LOG,")); bt.print(warnTotal); bt.print(','); bt.println(alarmTotal);
    Serial.print(F("LOG,")); Serial.print(warnTotal); Serial.print(','); Serial.println(alarmTotal);
  }
}

// H 신호, M 메시지, T 대기시간(초), A 경계, D 해제
void handleLine(char *line) {
  char c = line[0];
  if (c == 'H') {
    lastBeat = millis();
    everConnected = true;
    if (state != ST_IDLE && !manualArm) disarm(true);
  } else if (c == 'M') {
    strncpy(message, line + 1, 16);
    message[16] = '\0';
    if (state != ST_IDLE) showLcd();
  } else if (c == 'T') {
    int n = atoi(line + 1);
    if (n >= 1 && n <= 60) awayTimeout = (unsigned long)n * 1000UL;
  } else if (c == 'A') {
    manualArm = true;
    arm();
  } else if (c == 'D') {
    disarm(false);
  }
}

void readLines(Stream &port, char *buf, byte &len) {
  while (port.available()) {
    char ch = (char)port.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[len] = '\0';
      if (len > 0) handleLine(buf);
      len = 0;
    } else if (len < 23) {
      buf[len++] = ch;
    }
  }
}

void tickSensors() {
  if (millis() - tSensor < 100) return;
  tSensor = millis();

  warnDist = map(analogRead(PIN_POT), 0, 1023, 10, 100);

  if (state == ST_IDLE) {
    if (everConnected && millis() - lastBeat > awayTimeout) { manualArm = false; arm(); }
    return;
  }

  distCm = readDistance();
  if (distCm < warnDist) { if (nearCount < 10) nearCount++; } else nearCount = 0;

  if (mpuOk) {
    int16_t x, y, z;
    if (readAccel(x, y, z)) {
      float dx = (float)x - baseX, dy = (float)y - baseY, dz = (float)z - baseZ;
      float delta = sqrt(dx * dx + dy * dy + dz * dz);
      if (delta > ACC_LIMIT) { if (moveCount < 10) moveCount++; } else moveCount = 0;
    }
  }
  // 잡음 때문에 2번 연속일 때만
  if (moveCount >= 2 && state != ST_ALARM) { setState(ST_ALARM); return; }

  if (state == ST_ARMED) {
    if (nearCount >= 2) setState(ST_WARN);
  } else if (state == ST_WARN) {
    if (nearCount > 0) {
      farSince = 0;
      if (millis() - warnSince > WARN_TO_ALARM) setState(ST_ALARM);
    } else {
      if (farSince == 0) farSince = millis();
      if (millis() - farSince > WARN_CLEAR) setState(ST_ARMED);
    }
  }

}

void tickSound() {
  if (state == ST_WARN) {
    if (millis() - tSound >= 600) { tSound = millis(); tone(PIN_BUZZ, 1500, 80); }
  } else if (state == ST_ALARM) {
    if (millis() - tSound >= 250) {
      tSound = millis();
      sirenHigh = !sirenHigh;
      tone(PIN_BUZZ, sirenHigh ? 1400 : 900);
    }
  }
}

void tickLink() {
  if (millis() - tLink < 200) return;
  tLink = millis();
  // 해제 여부를 먼저 물어보고 상태 전송
  if (Wire.requestFrom(ADDR_BOARD_B, (byte)1) == 1) {
    if (Wire.read() == 1 && state != ST_IDLE) disarm(false);
  }
  Wire.beginTransmission(ADDR_BOARD_B);
  Wire.write(state);
  Wire.endTransmission();
}

void tickReport() {
  if (millis() - tReport < 1000) return;
  tReport = millis();
  bt.print(F("S,")); bt.print(state); bt.print(','); bt.println(distCm);
}

void setup() {
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  Serial.begin(9600);
  bt.begin(9600);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  lcd.init();
  lcd.backlight();
  mpuOk = mpuBegin();
  Serial.println(mpuOk ? F("MPU6050 OK") : F("MPU6050 not found"));

  moveFlag(FLAG_DOWN);

  showLeds();
  showLcd();
  tone(PIN_BUZZ, 1200, 100);
}

void loop() {
  readLines(bt, btBuf, btLen);
  readLines(Serial, pcBuf, pcLen);
  tickSensors();
  tickFlag();
  tickSound();
  tickLink();
  tickReport();
}
