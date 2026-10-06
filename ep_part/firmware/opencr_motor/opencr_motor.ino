// PROTOCOL v1
//  PC -> OpenCR : "V,좌raw,우raw\n"   속도 명령 (전진 +), 500ms 끊기면 정지
//                 한 글자(w a s d x + - z) = 디버그 키보드 조작
//  OpenCR -> PC : "F,ms,좌tick,우tick"  인코더 (50Hz)
//                 "U,ms,거리mm,estop"   후방 초음파 (약 16Hz, -1=범위 밖)
//                 "# ..."               디버그 문구 (파이에서 무시)
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL  Serial3
#define PC_SERIAL   Serial
const int DXL_DIR_PIN = 84;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUD = 1000000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------- 모터 설정 ----------
// ID1=왼앞, ID2=왼뒤, ID3=오뒤, ID4=오앞 (위에서 봤을 때 반시계 순서)
const uint8_t MOTOR_ID[4]   = {1, 2, 3, 4};
const int8_t  MOTOR_DIR[4]  = {+1, -1, -1, +1};   // 전진 시 tick이 + 가 되는 부호 (확정값)
// ★ 직접 검증한 최종값으로 교체할 것 (-1=왼쪽 그룹, +1=오른쪽 그룹)
const int8_t  MOTOR_SIDE[4] = {-1, -1, +1, +1};

int speedVal = 100;
const int SPEED_STEP = 20;
const int SPEED_MIN  = 20;
const int SPEED_MAX  = 250;     // XL430-W250 기본 제한(265) 이하

// ---------- 타임아웃 / 주기 ----------
const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long FB_INTERVAL_MS = 20;
unsigned long lastCmdMs = 0;    // 0 = 타임아웃 비활성 (키보드 모드)
unsigned long lastFbMs  = 0;

// ---------- 후방 초음파 + 긴급정지 ----------
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const unsigned long US_INTERVAL_MS = 60;
const unsigned long US_TIMEOUT_US  = 12000;   // 약 2m
unsigned long lastUsMs = 0;

const long ESTOP_MM       = 50;   // 5cm 이하 → 후진 차단
const long ESTOP_CLEAR_MM = 80;   // 8cm 이상 → 해제
const int  ESTOP_CONFIRM  = 2;    // 연속 N회 감지
bool estop = false;
int  estopCount = 0;

int32_t startPos[4] = {0, 0, 0, 0};
char lineBuf[48];
uint8_t lineLen = 0;

// ---------- 구동 ----------
void driveRaw(int left, int right) {
  left  = constrain(left,  -SPEED_MAX, SPEED_MAX);
  right = constrain(right, -SPEED_MAX, SPEED_MAX);

  // 후방 센서: 긴급정지 중에는 후진(음수) 성분 차단, 전진은 허용
  if (estop) {
    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
  }

  for (int i = 0; i < 4; i++) {
    int v = (MOTOR_SIDE[i] < 0) ? left : right;
    dxl.setGoalVelocity(MOTOR_ID[i], v * MOTOR_DIR[i]);
  }
}

void stopAll() { driveRaw(0, 0); }

// ---------- 인코더 ----------
void zeroEncoders() {
  for (int i = 0; i < 4; i++) {
    startPos[i] = (int32_t)dxl.getPresentPosition(MOTOR_ID[i]);
  }
}

void readSideTicks(int32_t &left, int32_t &right) {
  int32_t sum[2] = {0, 0};
  for (int i = 0; i < 4; i++) {
    int32_t raw = (int32_t)dxl.getPresentPosition(MOTOR_ID[i]);
    int32_t t = (raw - startPos[i]) * MOTOR_DIR[i];
    sum[MOTOR_SIDE[i] < 0 ? 0 : 1] += t;
  }
  left  = sum[0] / 2;
  right = sum[1] / 2;
}

void sendFeedback() {
  int32_t l, r;
  readSideTicks(l, r);
  PC_SERIAL.print("F,");
  PC_SERIAL.print(millis()); PC_SERIAL.print(",");
  PC_SERIAL.print(l);        PC_SERIAL.print(",");
  PC_SERIAL.println(r);
}

// ---------- 초음파 ----------
void sendUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT_US);
  long mm = (us == 0) ? -1 : (long)(us * 0.343f / 2.0f);

  bool tooClose = (mm > 0 && mm <= ESTOP_MM);
  if (tooClose) {
    if (++estopCount >= ESTOP_CONFIRM && !estop) {
      estop = true;
      stopAll();
      PC_SERIAL.println("# ESTOP ON");
    }
  } else {
    estopCount = 0;
    if (estop && (mm < 0 || mm >= ESTOP_CLEAR_MM)) {
      estop = false;
      PC_SERIAL.println("# ESTOP OFF");
    }
  }

  PC_SERIAL.print("U,");
  PC_SERIAL.print(millis()); PC_SERIAL.print(",");
  PC_SERIAL.print(mm);       PC_SERIAL.print(",");
  PC_SERIAL.println(estop ? 1 : 0);
}

// ---------- 명령 처리 ----------
void handleKey(char c) {
  lastCmdMs = 0;
  switch (c) {
    case 'w': case 'W': driveRaw(+speedVal, +speedVal); break;
    case 's': case 'S': driveRaw(-speedVal, -speedVal); break;
    case 'a': case 'A': driveRaw(-speedVal, +speedVal); break;   // 제자리 좌회전
    case 'd': case 'D': driveRaw(+speedVal, -speedVal); break;   // 제자리 우회전
    case 'x': case 'X': stopAll(); break;
    case '+': case '=': speedVal = min(speedVal + SPEED_STEP, SPEED_MAX); break;
    case '-': case '_': speedVal = max(speedVal - SPEED_STEP, SPEED_MIN); break;
    case 'z': case 'Z': zeroEncoders(); break;
    default: break;
  }
}

void handleLine(const char* s) {
  if (s[0] == 'V' && s[1] == ',') {
    int l, r;
    if (sscanf(s, "V,%d,%d", &l, &r) == 2) {
      driveRaw(l, r);
      lastCmdMs = millis();
      if (lastCmdMs == 0) lastCmdMs = 1;
    }
  } else if (s[0] != '\0' && s[1] == '\0') {
    handleKey(s[0]);
  }
}

void setup() {
  PC_SERIAL.begin(115200);
  unsigned long t0 = millis();
  while (!PC_SERIAL && millis() - t0 < 2000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++) {
    uint8_t id = MOTOR_ID[i];
    if (!dxl.ping(id)) {
      PC_SERIAL.print("# Motor ID "); PC_SERIAL.print(id); PC_SERIAL.println(" no response");
    }
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.torqueOn(id);
  }
  zeroEncoders();
  stopAll();
  PC_SERIAL.println("# ready");
}

void loop() {
  while (PC_SERIAL.available()) {
    char c = PC_SERIAL.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  if (lastCmdMs != 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopAll();
    lastCmdMs = 0;
  }

  if (millis() - lastFbMs >= FB_INTERVAL_MS) {
    lastFbMs = millis();
    sendFeedback();
  }

  if (millis() - lastUsMs >= US_INTERVAL_MS) {
    lastUsMs = millis();
    sendUltrasonic();
  }
}
