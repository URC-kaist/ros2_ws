#ifndef ARDUINO_NUCLEO_G431KB
#error This sketch is intended for NUCLEO-G431KB
#endif

#include <ACANFD_STM32.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareTimer.h>

// ---------------------- NeoPixel config ----------------------
static const uint8_t NEOPIXEL_PIN = D3;  // <-- change to your data pin
static const uint16_t NUM_PIXELS = 256;   // <-- change to your LED count
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- CAN config ---------------------------
static const uint32_t CAN_LED_CONTROL_ID = 0x123;    // standard 11-bit ID
static const uint32_t CAN_SERVO_CONTROL_ID = 0x124;  // standard 11-bit ID

// ---------------------- Servo config -------------------------
static const uint8_t SERVO_X_PIN = PA8;   // TIM1_CH1
static const uint8_t SERVO_Y_PIN = PA9;   // TIM1_CH2
static const uint8_t SERVO_X_CHANNEL = 1;
static const uint8_t SERVO_Y_CHANNEL = 2;
static const uint32_t SERVO_PERIOD_US = 20000;  // 50 Hz
static const uint16_t SERVO_CENTER_RAW = 2048;
static const uint16_t SERVO_FULL_SCALE_RAW = 2048;
static const uint16_t SERVO_X_CENTER_US = 1500;
static const uint16_t SERVO_X_RANGE_US = 800;
static const uint16_t SERVO_Y_CENTER_US = 1700;
static const uint16_t SERVO_Y_RANGE_US = 600;
static const uint16_t SERVO_X_MIN_US = 700;
static const uint16_t SERVO_X_MAX_US = 2300;
static const uint16_t SERVO_Y_MIN_US = 1100;
static const uint16_t SERVO_Y_MAX_US = 2300;

HardwareTimer servoTimer(TIM1);

enum Mode : uint8_t {
  MODE_OFF = 0,
  MODE_RED = 1,
  MODE_BLUE = 2,
  MODE_GREEN_BLINK = 3
};

static volatile Mode gMode = MODE_GREEN_BLINK;

// Green blink @ 500 ms (fixed)
static const uint32_t BLINK_PERIOD_MS = 500;
static uint32_t gNextBlinkMs = 0;
static bool gGreenOn = false;

// ---------------------- Helpers ------------------------------
static void applySolid(uint32_t color) {
  for (uint16_t i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, color);
  strip.show();
}

static void setMode(Mode m) {
  gMode = m;
  gGreenOn = false;
  gNextBlinkMs = millis() + BLINK_PERIOD_MS;

  switch (gMode) {
    case MODE_OFF: applySolid(strip.Color(0, 0, 0)); break;
    case MODE_RED: applySolid(strip.Color(65, 0, 0)); break;
    case MODE_BLUE: applySolid(strip.Color(0, 0, 65)); break;
    case MODE_GREEN_BLINK: applySolid(strip.Color(0, 0, 0)); break;  // start off
    default: applySolid(strip.Color(0, 0, 0)); break;
  }
}

static uint16_t clampPulse(int32_t pulse, uint16_t minPulse, uint16_t maxPulse) {
  if (pulse < minPulse) return minPulse;
  if (pulse > maxPulse) return maxPulse;
  return (uint16_t)pulse;
}

static uint16_t rawToPulse(uint16_t raw, uint16_t centerPulse, uint16_t rangePulse,
                           uint16_t minPulse, uint16_t maxPulse) {
  const int32_t delta = (int32_t)raw - SERVO_CENTER_RAW;
  const int32_t pulse = centerPulse + (delta * rangePulse) / SERVO_FULL_SCALE_RAW;
  return clampPulse(pulse, minPulse, maxPulse);
}

static void applyServoRaw(uint16_t vrx, uint16_t vry) {
  const uint16_t xPulse = rawToPulse(vrx, SERVO_X_CENTER_US, SERVO_X_RANGE_US,
                                     SERVO_X_MIN_US, SERVO_X_MAX_US);
  const uint16_t yPulse = rawToPulse(vry, SERVO_Y_CENTER_US, SERVO_Y_RANGE_US,
                                     SERVO_Y_MIN_US, SERVO_Y_MAX_US);

  servoTimer.setCaptureCompare(SERVO_X_CHANNEL, xPulse, MICROSEC_COMPARE_FORMAT);
  servoTimer.setCaptureCompare(SERVO_Y_CHANNEL, yPulse, MICROSEC_COMPARE_FORMAT);
}

static void setupServoPwm() {
  servoTimer.pause();
  servoTimer.setOverflow(SERVO_PERIOD_US, MICROSEC_FORMAT);
  servoTimer.setMode(SERVO_X_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, SERVO_X_PIN);
  servoTimer.setMode(SERVO_Y_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, SERVO_Y_PIN);
  applyServoRaw(SERVO_CENTER_RAW, SERVO_CENTER_RAW);
  servoTimer.resume();
}

static void pollCanAndUpdateMode() {
  CANFDMessage msg;

  // Filters below route our accepted IDs to FIFO0
  while (fdcan1.receiveFD0(msg)) {
    // Standard frame only
    if (msg.ext) continue;

    if (msg.id == CAN_LED_CONTROL_ID) {
      if (msg.len < 1) continue;

      const uint8_t v = msg.data[0];
      if (v <= MODE_GREEN_BLINK) {
        setMode((Mode)v);
      }
    } else if (msg.id == CAN_SERVO_CONTROL_ID) {
      if (msg.len < 4) continue;

      const uint16_t vrx = msg.data[0] | ((uint16_t)msg.data[1] << 8);
      const uint16_t vry = msg.data[2] | ((uint16_t)msg.data[3] << 8);
      applyServoRaw(vrx, vry);
    }
  }
}

static void updateBlink() {
  if (gMode != MODE_GREEN_BLINK) return;

  const uint32_t now = millis();
  if ((int32_t)(now - gNextBlinkMs) >= 0) {
    gNextBlinkMs += BLINK_PERIOD_MS;
    gGreenOn = !gGreenOn;
    applySolid(gGreenOn ? strip.Color(0, 65, 0) : strip.Color(0, 0, 0));
  }
}

// ---------------------- Arduino setup/loop --------------------
void setup() {
  strip.begin();
  strip.setBrightness(255);
  strip.show();  // clear
  setMode(MODE_GREEN_BLINK);
  setupServoPwm();

  // Classic CAN @ 500 kbps (use x1 since we don't need a faster data phase)
  ACANFD_STM32_Settings settings(500 * 1000, DataBitRateFactor::x1);

  // Accept ONLY our control IDs into FIFO0
  ACANFD_STM32_StandardFilters filters;
  filters.addSingle(CAN_LED_CONTROL_ID, ACANFD_STM32_FilterAction::FIFO0);
  filters.addSingle(CAN_SERVO_CONTROL_ID, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  fdcan1.beginFD(settings, filters);
}

void loop() {
  pollCanAndUpdateMode();
  updateBlink();
}
