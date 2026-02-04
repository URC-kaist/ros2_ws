#ifndef ARDUINO_NUCLEO_G431KB
#error This sketch is intended for NUCLEO-G431KB
#endif

#include <ACANFD_STM32.h>
#include <Adafruit_NeoPixel.h>

// ---------------------- NeoPixel config ----------------------
static const uint8_t NEOPIXEL_PIN = D3;  // <-- change to your data pin
static const uint16_t NUM_PIXELS = 256;   // <-- change to your LED count
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- CAN config ---------------------------
static const uint32_t CAN_CONTROL_ID = 0x123;  // standard 11-bit ID

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
    case MODE_RED: applySolid(strip.Color(50, 0, 0)); break;
    case MODE_BLUE: applySolid(strip.Color(0, 0, 50)); break;
    case MODE_GREEN_BLINK: applySolid(strip.Color(0, 0, 0)); break;  // start off
    default: applySolid(strip.Color(0, 0, 0)); break;
  }
}

static void pollCanAndUpdateMode() {
  CANFDMessage msg;

  // Filters below route our ID to FIFO0
  while (fdcan1.receiveFD0(msg)) {
    // Standard frame only
    if (msg.ext) continue;
    if (msg.id != CAN_CONTROL_ID) continue;
    if (msg.len < 1) continue;

    const uint8_t v = msg.data[0];
    if (v <= MODE_GREEN_BLINK) {
      setMode((Mode)v);
    }
  }
}

static void updateBlink() {
  if (gMode != MODE_GREEN_BLINK) return;

  const uint32_t now = millis();
  if ((int32_t)(now - gNextBlinkMs) >= 0) {
    Serial.println("color change");
    gNextBlinkMs += BLINK_PERIOD_MS;
    gGreenOn = !gGreenOn;
    applySolid(gGreenOn ? strip.Color(0, 50, 0) : strip.Color(0, 0, 0));
  }
}

// ---------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  strip.begin();
  strip.setBrightness(255);
  strip.show();  // clear
  setMode(MODE_GREEN_BLINK);

  // Classic CAN @ 1 Mbps (use x1 since we don't need a faster data phase)
  ACANFD_STM32_Settings settings(1000 * 1000, DataBitRateFactor::x1);

  // Accept ONLY our control ID into FIFO0
  ACANFD_STM32_StandardFilters filters;
  filters.addSingle(CAN_CONTROL_ID, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  const uint32_t err = fdcan1.beginFD(settings, filters);
  if (err == 0) {
    Serial.println("FDCAN started OK");
  } else {
    Serial.print("FDCAN beginFD error: 0x");
    Serial.println(err, HEX);
  }

  Serial.println("Send StdID 0x123, DLC=1, data[0]=0..3");
}

void loop() {
  pollCanAndUpdateMode();
  updateBlink();
}
