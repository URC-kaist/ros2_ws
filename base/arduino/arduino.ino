/*
  MD13S + Encoder + Home switch controller
  Binary serial protocol (SOF AA55, LEN, SEQ, CMD, PAYLOAD, CRC16 CCITT-FALSE)
  Supports:
    - HOMING_START (CMD 0x01)
    - MOVE_TO_RAD  (CMD 0x02, payload int32 Q16.16 rad)
  Responds:
    - ACK  (CMD 0x80, payload: ack_seq, ack_cmd)
    - DONE (CMD 0x81, payload: done_seq, done_cmd)
    - ERROR(CMD 0x82, payload: err_seq, err_code, detail_u16)

  CRC range: from LEN field through end of PAYLOAD (SOF excluded, CRC excluded).
*/

#include <Arduino.h>

// ---------------------- Pins ----------------------
constexpr uint8_t PIN_PWM  = 5;
constexpr uint8_t PIN_DIR  = 4;
constexpr uint8_t ENC_A    = 2;
constexpr uint8_t ENC_B    = 3;
constexpr uint8_t HOME_PIN = 7;               // switch input + INPUT_PULLUP

// The installed base-station home switch is wired active-high, so a triggered
// switch reads HIGH. Flip this to LOW for a switch-to-GND INPUT_PULLUP setup.
constexpr uint8_t HOME_ACTIVE_LEVEL = HIGH;

// ---------------------- Encoder ----------------------
constexpr long COUNTS_PER_REV = 3614;          // output shaft counts per revolution
volatile long g_encCount = 0;

// ---------------------- Control Loop ----------------------
constexpr uint16_t CTRL_HZ = 100;
constexpr uint16_t CTRL_PERIOD_MS = 1000 / CTRL_HZ;
constexpr float    CTRL_DT = 1.0f / CTRL_HZ;

// ---------------------- Motion conversion ----------------------
// Angle conversion factor preserved from the original code.
// rad -> revolutions = rad / (2*pi) => counts = rev * CPR * (GEAR_NUM / GEAR_DEN)
constexpr float GEAR_NUM = 80.0f;
constexpr float GEAR_DEN = 20.0f;
constexpr float TWO_PI_F = 6.2831853071795864769f;

// Clamp commanded angle range.
constexpr float MAX_ABS_TARGET_RAD = (3.14159265358979323846f / 2.0f);   // +-90 deg

// ---------------------- Position PID ----------------------
constexpr long DEAD_BAND = 8;
constexpr int  MIN_PWM   = 35;
constexpr int  MAX_PWM   = 120;

struct PID {
  float Kp{0.18f};
  float Ki{0.00f};
  float Kd{0.03f};
  float integ{0};
  float prevErr{0};
  float integLimit{2000.0f};

  void reset() { integ = 0; prevErr = 0; }

  float update(float err, float dt) {
    integ += err * dt;
    if (integ > integLimit) integ = integLimit;
    if (integ < -integLimit) integ = -integLimit;

    float deriv = (err - prevErr) / dt;
    prevErr = err;
    return Kp * err + Ki * integ + Kd * deriv;
  }
};

PID g_pid;

// ---------------------- PWM Ramp ----------------------
struct Ramp {
  int step{3};
  int value{0};
  void reset(int v = 0) { value = v; }
  int updateTo(int target) {
    if (target > value) value = min(value + step, target);
    else                value = max(value - step, target);
    return value;
  }
};

Ramp g_ramp;

// ---------------------- Homing params ----------------------
constexpr int  HOME_PWM_FAST      = 60;
constexpr int  HOME_PWM_SLOW      = 40;
constexpr int  HOME_DIR           = 1;   // +1 or -1 for homing direction
constexpr long BACKOFF_COUNTS     = 80;
constexpr long HOME_RELEASE_COUNTS= 120;
constexpr long HOME_OFFSET_COUNTS =
  (long)((0.15f / TWO_PI_F) * (float)COUNTS_PER_REV * (GEAR_NUM / GEAR_DEN)); // +0.15 rad
// Search range in counts: one output shaft revolution with gear ratio applied.
constexpr long HOME_SEARCH_MAX_COUNTS =
  (long)((float)COUNTS_PER_REV * (GEAR_NUM / GEAR_DEN));
constexpr uint16_t AUTO_RETURN_IDLE_MS = 1000;
constexpr float AUTO_RETURN_RAD = 0.0f;

// ---------------------- State Machine ----------------------
enum class State : uint8_t {
  IDLE,
  HOMING_SEARCH,
  HOMING_BACKOFF,
  HOMING_APPROACH,
  HOMING_RELEASE,
  READY,
  ERROR_LATCH
};

struct HomingCtx {
  long startCount = 0;
  long searchStartCount = 0;
  void reset() { startCount = 0; searchStartCount = 0; }
};

State g_state = State::IDLE;
HomingCtx g_homeCtx;

long g_targetCount = 0;
bool g_homed = false;

// current “active action” for DONE reporting
uint8_t g_activeReqSeq = 0xFF;
uint8_t g_activeReqCmd = 0x00;
bool    g_actionActive = false;
unsigned long g_lastCmdMs = 0;
bool    g_autoReturnActive = false;

// ---------------------- Protocol constants ----------------------
constexpr uint8_t SOF0 = 0xAA;
constexpr uint8_t SOF1 = 0x55;

constexpr uint8_t CMD_HOMING_START = 0x01;
constexpr uint8_t CMD_MOVE_TO_RAD  = 0x02;

constexpr uint8_t CMD_ACK   = 0x80;
constexpr uint8_t CMD_DONE  = 0x81;
constexpr uint8_t CMD_ERROR = 0x82;

// Error codes (as spec)
constexpr uint8_t ERR_CRC_MISMATCH            = 0x01;
constexpr uint8_t ERR_BAD_LEN                 = 0x02;
constexpr uint8_t ERR_UNKNOWN_CMD             = 0x03;
constexpr uint8_t ERR_HOME_TRIGGERED_DURING_MOVE = 0x10;
constexpr uint8_t ERR_NOT_HOMED               = 0x11;
constexpr uint8_t ERR_OUT_OF_RANGE            = 0x12;
constexpr uint8_t ERR_BUSY                    = 0x13;
constexpr uint8_t ERR_HOME_NOT_FOUND          = 0x14;

// Device TX sequence (optional, for outgoing frames)
uint8_t g_txSeq = 0;

// ---------------------- Low-level helpers ----------------------
static inline long getCount() {
  noInterrupts();
  long c = g_encCount;
  interrupts();
  return c;
}

static inline void setCount(long v) {
  noInterrupts();
  g_encCount = v;
  interrupts();
}

static inline bool homeTriggered() {
  return digitalRead(HOME_PIN) == HOME_ACTIVE_LEVEL;
}

static inline void stopMotor() {
  analogWrite(PIN_PWM, 0);
}

static inline void setMotorPWM(int pwm) {
  int dir = (pwm >= 0) ? LOW : HIGH; // 필요시 폴라리티만 여기서 바꾸세요
  int out = constrain(abs(pwm), 0, 255);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_PWM, out);
}

// Encoder ISR (x2 on ENC_A edges)
void isrEncA() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  if (a == b) g_encCount++;
  else        g_encCount--;
}

// ---------------------- CRC16 CCITT-FALSE ----------------------
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// ---------------------- Protocol TX ----------------------
void txFrame(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen) {
  // LEN counts SEQ + CMD + PAYLOAD
  const uint8_t lenField = (uint8_t)(2 + payloadLen);
  const uint8_t seqField = g_txSeq++;

  uint8_t hdr[5];
  hdr[0] = SOF0;
  hdr[1] = SOF1;
  hdr[2] = lenField;
  hdr[3] = seqField;
  hdr[4] = cmd;

  // CRC range: LEN..PAYLOAD (LEN field + LEN bytes)
  // We build a small temp buffer for CRC: [LEN, SEQ, CMD, PAYLOAD...]
  uint8_t crcBuf[1 + 2 + 255]; // safe upper bound
  crcBuf[0] = lenField;
  crcBuf[1] = seqField;
  crcBuf[2] = cmd;
  for (uint8_t i = 0; i < payloadLen; i++) crcBuf[3 + i] = payload[i];

  uint16_t crc = crc16_ccitt_false(crcBuf, (size_t)(1 + lenField));

  // Send
  Serial.write(hdr, sizeof(hdr));
  if (payloadLen) Serial.write(payload, payloadLen);
  uint8_t crcLE[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
  Serial.write(crcLE, 2);
}

void txACK(uint8_t ack_seq, uint8_t ack_cmd) {
  uint8_t p[2] = { ack_seq, ack_cmd };
  txFrame(CMD_ACK, p, 2);
}

void txDONE(uint8_t done_seq, uint8_t done_cmd) {
  uint8_t p[2] = { done_seq, done_cmd };
  txFrame(CMD_DONE, p, 2);
}

void txERROR(uint8_t err_seq, uint8_t err_code, uint16_t detail) {
  uint8_t p[4] = { err_seq, err_code, (uint8_t)(detail & 0xFF), (uint8_t)(detail >> 8) };
  txFrame(CMD_ERROR, p, 4);
}

// ---------------------- Motion primitives ----------------------
int positionControllerCounts(long target, long current) {
  long errCounts = target - current;
  if (labs(errCounts) <= DEAD_BAND) return 0;

  float u = g_pid.update((float)errCounts, CTRL_DT);
  int pwm = (int)constrain(u, -MAX_PWM, +MAX_PWM);

  if (pwm > 0) pwm = max(pwm, MIN_PWM);
  if (pwm < 0) pwm = min(pwm, -MIN_PWM);

  return pwm;
}

void beginAction(uint8_t reqSeq, uint8_t reqCmd) {
  g_activeReqSeq = reqSeq;
  g_activeReqCmd = reqCmd;
  g_actionActive = true;
}

void endActionDone() {
  if (g_actionActive) {
    txDONE(g_activeReqSeq, g_activeReqCmd);
  }
  g_actionActive = false;
  g_activeReqSeq = 0xFF;
  g_activeReqCmd = 0x00;
}

void latchError(uint8_t errSeq, uint8_t errCode, uint16_t detail) {
  stopMotor();
  g_ramp.reset(0);
  g_state = State::ERROR_LATCH;
  g_actionActive = false; // latch error cancels action
  txERROR(errSeq, errCode, detail);
}

// ---------------------- Action start APIs ----------------------
bool startHoming(uint8_t reqSeq, uint8_t reqCmd) {
  // Busy check
  if (g_state == State::HOMING_SEARCH || g_state == State::HOMING_BACKOFF ||
      g_state == State::HOMING_APPROACH || g_state == State::HOMING_RELEASE) {
    txERROR(reqSeq, ERR_BUSY, (uint16_t)g_state);
    return false;
  }
  if (g_state == State::ERROR_LATCH) {
    // In error latch, a reset policy would be needed; treat as busy for now.
    txERROR(reqSeq, ERR_BUSY, (uint16_t)g_state);
    return false;
  }

  stopMotor();
  g_ramp.reset(0);
  g_pid.reset();
  g_homeCtx.reset();
  g_homed = false;

  beginAction(reqSeq, reqCmd);
  txACK(reqSeq, reqCmd);

  if (homeTriggered()) {
    g_homeCtx.startCount = getCount();
    g_state = State::HOMING_BACKOFF;
  } else {
    g_homeCtx.searchStartCount = getCount();
    g_state = State::HOMING_SEARCH;
  }
  return true;
}

bool startMoveToRad(uint8_t reqSeq, uint8_t reqCmd, float thetaRad) {
  // Must be homed
  if (!g_homed) {
    txERROR(reqSeq, ERR_NOT_HOMED, 0);
    return false;
  }
  // Busy check
  if (g_state == State::HOMING_SEARCH || g_state == State::HOMING_BACKOFF ||
      g_state == State::HOMING_APPROACH || g_state == State::HOMING_RELEASE) {
    txERROR(reqSeq, ERR_BUSY, (uint16_t)g_state);
    return false;
  }
  if (g_state == State::ERROR_LATCH) {
    txERROR(reqSeq, ERR_BUSY, (uint16_t)g_state);
    return false;
  }

  if (fabs(thetaRad) > MAX_ABS_TARGET_RAD) {
    txERROR(reqSeq, ERR_OUT_OF_RANGE, 0);
    return false;
  }

  // Convert rad -> counts
  float revolutions = thetaRad / TWO_PI_F;
  float countsF = revolutions * (float)COUNTS_PER_REV * (GEAR_NUM / GEAR_DEN);
  long cnt = (long)lround(countsF);

  g_targetCount = cnt;
  g_ramp.reset(0);
  g_pid.reset();

  beginAction(reqSeq, reqCmd);
  txACK(reqSeq, reqCmd);

  g_state = State::READY;
  return true;
}

static inline void startMoveToRadInternal(float thetaRad) {
  float clamped = constrain(thetaRad, -MAX_ABS_TARGET_RAD, MAX_ABS_TARGET_RAD);
  float revolutions = clamped / TWO_PI_F;
  float countsF = revolutions * (float)COUNTS_PER_REV * (GEAR_NUM / GEAR_DEN);
  long cnt = (long)lround(countsF);

  g_targetCount = cnt;
  g_ramp.reset(0);
  g_pid.reset();
  g_actionActive = false; // internal move: no DONE/ACK
  g_state = State::READY;
}

// ---------------------- RX Parser ----------------------
// Frame: AA 55 | LEN | SEQ | CMD | PAYLOAD... | CRC_L CRC_H
// Total bytes = 2 + 1 + LEN + 2
struct RxParser {
  enum class PState : uint8_t { WAIT_AA, WAIT_55, WAIT_LEN, WAIT_BODY } st = PState::WAIT_AA;
  uint8_t len = 0;
  uint8_t buf[1 + 255 + 2]; // [LEN..PAYLOAD] (1+LEN bytes) + CRC16(2)
  uint16_t need = 0;        // how many bytes remaining to fill buf (after writing LEN)
  uint16_t idx = 0;

  void reset() {
    st = PState::WAIT_AA;
    len = 0;
    need = 0;
    idx = 0;
  }

  // returns true when a complete valid frame is ready in internal buffer
  bool feed(uint8_t b) {
    switch (st) {
      case PState::WAIT_AA:
        if (b == SOF0) st = PState::WAIT_55;
        break;

      case PState::WAIT_55:
        if (b == SOF1) st = PState::WAIT_LEN;
        else st = PState::WAIT_AA;
        break;

      case PState::WAIT_LEN:
        len = b;
        // sanity: LEN must be at least 2 (SEQ+CMD) and not too large
        if (len < 2 || len > 255) { reset(); break; }
        // store LEN as first byte in buf (CRC range starts here)
        buf[0] = len;
        idx = 1;
        // need to read remaining (LEN bytes of SEQ..PAYLOAD) + CRC(2)
        need = (uint16_t)len + 2;
        st = PState::WAIT_BODY;
        break;

      case PState::WAIT_BODY:
        if (need == 0) { reset(); break; }
        buf[idx++] = b;
        need--;
        if (need == 0) {
          // complete. Validate CRC
          // CRC input length = 1 + LEN bytes => bytes buf[0 .. (0+LEN)]
          const uint16_t crcCalc = crc16_ccitt_false(buf, (size_t)(1 + len));
          const uint8_t crcL = buf[1 + len];
          const uint8_t crcH = buf[1 + len + 1];
          const uint16_t crcRx = (uint16_t)crcL | ((uint16_t)crcH << 8);

          if (crcCalc != crcRx) {
            // best-effort: we can’t know SEQ safely if parsing failed midstream, but here we have it
            uint8_t seq = buf[1];
            txERROR(seq, ERR_CRC_MISMATCH, 0);
            reset();
            return false;
          }
          return true; // frame ready in buf
        }
        break;
    }
    return false;
  }

  // Extractors (valid only if feed() returned true)
  uint8_t seq() { return buf[1]; }
  uint8_t cmd() { return buf[2]; }
  const uint8_t* payload() { return &buf[3]; }
  uint8_t payloadLen() { return (uint8_t)(len - 2); }
};

RxParser g_rx;

// ---------------------- Command handling ----------------------
static inline int32_t readI32LE(const uint8_t* p) {
  return (int32_t)(
    ((uint32_t)p[0]) |
    ((uint32_t)p[1] << 8) |
    ((uint32_t)p[2] << 16) |
    ((uint32_t)p[3] << 24)
  );
}

void handleFrame(RxParser& fr) {
  const uint8_t seq = fr.seq();
  const uint8_t cmd = fr.cmd();
  const uint8_t plen = fr.payloadLen();
  const uint8_t* pl = fr.payload();
  g_lastCmdMs = millis();
  g_autoReturnActive = false;

  if (cmd == CMD_HOMING_START) {
    if (plen != 0) { txERROR(seq, ERR_BAD_LEN, plen); return; }
    startHoming(seq, cmd);
    return;
  }

  if (cmd == CMD_MOVE_TO_RAD) {
    if (plen != 4) { txERROR(seq, ERR_BAD_LEN, plen); return; }
    int32_t theta_q = readI32LE(pl);
    float thetaRad = (float)theta_q / 65536.0f;
    startMoveToRad(seq, cmd, thetaRad);
    return;
  }

  // Unknown
  txERROR(seq, ERR_UNKNOWN_CMD, cmd);
}

// ---------------------- setup / loop ----------------------
void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(HOME_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, CHANGE);

  Serial.begin(115200);

  g_state = State::IDLE;
  g_homed = false;
  g_lastCmdMs = millis();
  stopMotor();
}

void loop() {
  // ---- RX: feed bytes, parse frames ----
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (g_rx.feed(b)) {
      // frame ready
      handleFrame(g_rx);
      g_rx.reset(); // prepare for next frame
    }
  }

  // ---- 100Hz control loop ----
  static unsigned long lastMs = 0;
  const unsigned long now = millis();
  if (now - lastMs < CTRL_PERIOD_MS) return;
  lastMs = now;

  // Home switch is used only for homing (no limit switch behavior during moves).

  switch (g_state) {
    case State::IDLE:
    case State::READY:
      if (g_homed) {
        long cur = getCount();
        int pwmTarget = positionControllerCounts(g_targetCount, cur);
        int pwmOut = g_ramp.updateTo(pwmTarget);
        setMotorPWM(pwmOut);

        if (pwmTarget == 0 && pwmOut == 0) {
          stopMotor();
          if (g_actionActive) {
            endActionDone();
          }
        }

        if (!g_autoReturnActive && (millis() - g_lastCmdMs > AUTO_RETURN_IDLE_MS)) {
          startMoveToRadInternal(AUTO_RETURN_RAD);
          g_autoReturnActive = true;
        }
      }
      break;

    case State::HOMING_SEARCH: {
      setMotorPWM(HOME_DIR * HOME_PWM_FAST);
      if (homeTriggered()) {
        stopMotor();
        g_ramp.reset(0);
        g_homeCtx.startCount = getCount();
        g_state = State::HOMING_BACKOFF;
      } else {
        long moved = labs(getCount() - g_homeCtx.searchStartCount);
        if (moved > HOME_SEARCH_MAX_COUNTS) {
          latchError(g_activeReqSeq, ERR_HOME_NOT_FOUND, (uint16_t)moved);
        }
      }
    } break;

    case State::HOMING_BACKOFF: {
      setMotorPWM(-HOME_DIR * HOME_PWM_SLOW);
      long moved = labs(getCount() - g_homeCtx.startCount);
      if (!homeTriggered() && moved > BACKOFF_COUNTS) {
        stopMotor();
        g_ramp.reset(0);
        g_state = State::HOMING_APPROACH;
      }
    } break;

    case State::HOMING_APPROACH: {
      setMotorPWM(HOME_DIR * HOME_PWM_SLOW);
      if (homeTriggered()) {
        stopMotor();
        g_ramp.reset(0);
        setCount(HOME_OFFSET_COUNTS);  // home trigger position = offset
        g_homeCtx.startCount = getCount(); // 0
        g_state = State::HOMING_RELEASE;
      }
    } break;

    case State::HOMING_RELEASE: {
      setMotorPWM(-HOME_DIR * HOME_PWM_SLOW);
      long moved = labs(getCount() - g_homeCtx.startCount);
      if (!homeTriggered() && moved > HOME_RELEASE_COUNTS) {
        stopMotor();
        g_ramp.reset(0);
        g_state = State::READY;
        g_homed = true;

        // report DONE for the homing request
        endActionDone();
      }
    } break;

    case State::ERROR_LATCH:
      stopMotor();
      break;
  }
}
