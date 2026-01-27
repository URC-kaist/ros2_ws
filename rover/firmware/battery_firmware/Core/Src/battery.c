// Talks to Makita XGT battery over single-wire, 9600 8E1, inverted,
// half-duplex. USART1 = battery bus (half-duplex, inverted), USART2 = logs.

#include "battery.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// ========================== CONFIG / DEBUG ==========================
#define XGT_DBG 1 // 0: silent, 1: commands+results, 2: verbose (raw bytes)
#define XGT_RX_SILENCE_MS                                                      \
  500u // idle gap to treat frame as complete (short frames)
#define XGT_RX_SILENCE_L                                                       \
  800u // idle gap for long frames (not used but supported)

#define XGT_CAN_ID_SUMMARY 0x320u
#define XGT_CAN_ID_META 0x321u
#define XGT_CAN_ID_CELL_BASE 0x330u

// ====================== Private module state ======================
static UART_HandleTypeDef *s_huart_batt =
    NULL; // USART configured for Makita bus (half-duplex, inverted)
static UART_HandleTypeDef *s_huart_log = NULL; // Optional logging UART
static FDCAN_HandleTypeDef *s_hfdcan = NULL;   // Optional CAN telemetry bus

static uint32_t s_update_interval_ms = 10000; // poll every 10s
static uint32_t s_last_update_ms = 0;
static uint32_t s_state_start_ms = 0;
static uint8_t s_current_cell = 0;

static uint8_t s_rx_len = 0;
static uint8_t s_buf[32];
static uint32_t s_cycle = 0;

// Battery data
static uint16_t batt_health_ = 0;  // %
static uint16_t cell_size_ = 0;    // mAh (scaled)
static uint16_t parallel_cnt_ = 0; // count
static uint16_t charge_ = 0;       // %
static uint16_t num_charges_ = 0;  // count
static float temperature_ = 0.f;   // °C
static float pack_voltage_ = 0.f;  // V
static float cell_voltages_[10] = {0};

// ================= Nibble/byte bit-reversal lookup =================
static const uint8_t LOOKUP[16] = {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
                                   0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF};

static inline uint8_t reverse_byte(uint8_t v) {
  return (uint8_t)((LOOKUP[v & 0x0F] << 4) | LOOKUP[(v >> 4) & 0x0F]);
}
static inline uint8_t reverse_nibbles_via_lookup(uint8_t v) {
  return reverse_byte(v);
}
static inline uint16_t le16_at(const uint8_t *p) { // after bit-reverse
  return (uint16_t)((p[1] << 8) | p[0]);
}

// =========================== Commands ===========================
static const uint8_t CMD_NUM_CHARGES[8] = {0x33, 0xC8, 0x03, 0x00,
                                           0x2A, 0x00, 0x00, 0xCC};
static const uint8_t CMD_CELL_SIZE[8] = {0x33, 0x27, 0xBB, 0x10,
                                         0x00, 0x00, 0x00, 0xCC};
static const uint8_t CMD_PARALLEL_CNT[8] = {0x33, 0x67, 0xBB, 0x50,
                                            0x00, 0x00, 0x00, 0xCC};
static const uint8_t CMD_BATT_HEALTH[8] = {0x33, 0xC4, 0x03, 0x00,
                                           0x26, 0x00, 0x00, 0xCC};
static const uint8_t CMD_CHARGE[8] = {0x33, 0x13, 0x03, 0x80,
                                      0x10, 0x00, 0x00, 0xCC};
static const uint8_t CMD_TEMPERATURE[8] = {0x33, 0x3B, 0x03, 0xC0,
                                           0x58, 0x00, 0x00, 0xCC};
static const uint8_t CMD_PACK_VOLT[8] = {0x33, 0x43, 0x03, 0xC0,
                                         0x00, 0x00, 0x00, 0xCC};
static const uint8_t CMD_CELL_V_BASE[8] = {0x33, 0x23, 0x03, 0xC0,
                                           0x00, 0x00, 0x00, 0xCC};

// =========================== Logging ===========================
static void b_log(const char *fmt, ...) {
#if XGT_DBG >= 1
  if (!s_huart_log)
    return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0)
    return;
  if (n > (int)sizeof(buf))
    n = (int)sizeof(buf);
  HAL_UART_Transmit(s_huart_log, (uint8_t *)buf, (uint16_t)n, 50);
#endif
}
static void dump_bytes(const char *tag, const uint8_t *d, uint8_t n) {
#if XGT_DBG >= 2
  b_log("[XGT]%s len=%u: ", tag, n);
  for (uint8_t i = 0; i < n; i++)
    b_log("%02X ", d[i]);
  b_log("\r\n");
#else
  (void)tag;
  (void)d;
  (void)n;
#endif
}

// ======================= FDCAN telemetry ======================
static bool fdcan_wait_for_space(uint32_t timeout_ms) {
  if (!s_hfdcan)
    return false;

  uint32_t start = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0U) {
    if ((HAL_GetTick() - start) >= timeout_ms)
      return false;
  }
  return true;
}

static void send_fdcan_frame(uint32_t id, const uint8_t payload[8]) {
  if (!s_hfdcan)
    return;

  if (!fdcan_wait_for_space(5u)) {
    b_log("[XGT] CAN TX timeout (0x%03lX)\r\n", (unsigned long)id);
    return;
  }

  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &header, (uint8_t *)payload) !=
      HAL_OK) {
    b_log("[XGT] CAN TX failed (0x%03lX)\r\n", (unsigned long)id);
  }
}

static void publish_fdcan_state(void) {
  if (!s_hfdcan)
    return;

  uint8_t summary[8] = {0};
  int32_t temp_scaled =
      (int32_t)(temperature_ * 10.0f + (temperature_ >= 0.0f ? 0.5f : -0.5f));
  if (temp_scaled < -32768)
    temp_scaled = -32768;
  if (temp_scaled > 32767)
    temp_scaled = 32767;
  int16_t temp_t10 = (int16_t)temp_scaled;

  int32_t pack_scaled = (int32_t)(pack_voltage_ * 100.0f + 0.5f);
  if (pack_scaled < 0)
    pack_scaled = 0;
  if (pack_scaled > 0xFFFF)
    pack_scaled = 0xFFFF;
  uint16_t pack_centi = (uint16_t)pack_scaled;

  uint16_t charge_pct = (charge_ > 100u) ? 100u : charge_;
  summary[0] = (uint8_t)charge_pct;
  summary[1] = (uint8_t)((batt_health_ > 255u) ? 255u : batt_health_);
  summary[2] = (uint8_t)(temp_t10 & 0xFF);
  summary[3] = (uint8_t)((temp_t10 >> 8) & 0xFF);
  summary[4] = (uint8_t)(pack_centi & 0xFF);
  summary[5] = (uint8_t)(pack_centi >> 8);
  summary[6] = (uint8_t)(s_cycle & 0xFF);
  summary[7] = (uint8_t)((s_cycle >> 8) & 0xFF);
  send_fdcan_frame(XGT_CAN_ID_SUMMARY, summary);

  uint8_t meta[8] = {0};
  meta[0] = (uint8_t)(cell_size_ & 0xFF);
  meta[1] = (uint8_t)(cell_size_ >> 8);
  meta[2] = parallel_cnt_;
  meta[3] = 10u;
  meta[4] = (uint8_t)(num_charges_ & 0xFF);
  meta[5] = (uint8_t)(num_charges_ >> 8);
  meta[6] = (uint8_t)((s_cycle >> 16) & 0xFF);
  meta[7] = (uint8_t)((s_cycle >> 24) & 0xFF);
  send_fdcan_frame(XGT_CAN_ID_META, meta);

  for (uint8_t pair = 0; pair < 5u; ++pair) {
    uint8_t frame[8] = {0};
    for (uint8_t slot = 0; slot < 2u; ++slot) {
      uint8_t cell_index = (uint8_t)(pair * 2u + slot);
      uint8_t base = (uint8_t)(slot * 4u);
      if (cell_index < 10u) {
        float cell_v = cell_voltages_[cell_index];
        int32_t scaled_mv =
            (int32_t)(cell_v * 1000.0f + (cell_v >= 0.0f ? 0.5f : -0.5f));
        if (scaled_mv < 0)
          scaled_mv = 0;
        if (scaled_mv > 0xFFFF)
          scaled_mv = 0xFFFF;
        frame[base + 0] = (uint8_t)(cell_index + 1u);
        frame[base + 1] = (uint8_t)(scaled_mv & 0xFF);
        frame[base + 2] = (uint8_t)(scaled_mv >> 8);
        frame[base + 3] = (uint8_t)(cell_v > 0.0f ? 1u : 0u);
      }
    }
    send_fdcan_frame((uint32_t)(XGT_CAN_ID_CELL_BASE + pair), frame);
  }
}

// ======================= UART helpers ===========================
static void uart_clear_rx(UART_HandleTypeDef *huart) {
  __HAL_UART_CLEAR_OREFLAG(huart);
  while ((huart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
    volatile uint8_t d = (uint8_t)(huart->Instance->RDR & 0xFF);
    (void)d;
  }
}
static int uart_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms) {
  HAL_HalfDuplex_EnableTransmitter(s_huart_batt);
  HAL_StatusTypeDef st =
      HAL_UART_Transmit(s_huart_batt, (uint8_t *)data, len, timeout_ms);
  return (st == HAL_OK) ? 0 : -1;
}

// ============================ CRC ==============================
// CRC over bytes AFTER bit-reversal (matches working impl)
static bool check_crc(uint8_t *rx, uint8_t len) {
  if (len < 2)
    return false;

  if (len >= 8 && rx[0] == 0xA5 && rx[1] == 0xA5) { // long form (rare)
    uint16_t crc = 0;
    uint8_t pad = rx[3] & 0x0F;
    if (len < (uint8_t)(4 + 2 + pad))
      return false;
    for (uint8_t i = 2; i < (uint8_t)(len - 2 - pad); ++i)
      crc += rx[i];
    uint16_t rx_crc = (uint16_t)((rx[len - 2] << 8) | rx[len - 1]);
    return crc == rx_crc;
  }

  if (len >= 8 && rx[0] == 0xCC && rx[len - 1] == 0x33) { // short form
    uint16_t crc = rx[0];
    for (uint8_t i = 2; i < len; ++i)
      crc += rx[i]; // skip rx[1] (crc byte)
    return (uint8_t)(crc % 256) == rx[1];
  }
  return false;
}

// =========================== TX/RX =============================
// Drain RX immediately after TX; collect until idle gap; early-CRC for short
// frames.
static int8_t send_battery(uint8_t *rx_buf, const uint8_t *cmd, uint8_t cmd_len,
                           uint8_t *rx_len) {
  *rx_len = 0;
  const bool is_long = (cmd_len >= 2 && cmd[0] == 0xA5 && cmd[1] == 0xA5);
  const uint32_t tail_silence_ms =
      is_long ? XGT_RX_SILENCE_L : XGT_RX_SILENCE_MS;

  uart_clear_rx(s_huart_batt);
  dump_bytes(" TX", cmd, cmd_len);

  if (uart_tx(cmd, cmd_len, 20) < 0) {
    b_log("[XGT] TX failed\r\n");
    return -2;
  }
  HAL_HalfDuplex_EnableReceiver(s_huart_batt);

  uint32_t last_activity = HAL_GetTick();
  uint8_t *wp = rx_buf;
  const uint8_t *end = rx_buf + sizeof(s_buf);

  // Drain until idle gap passes
  for (;;) {
    uint32_t isr = s_huart_batt->Instance->ISR;

    if (isr & (USART_ISR_RXNE_RXFNE | USART_ISR_ORE | USART_ISR_FE |
               USART_ISR_NE | USART_ISR_PE)) {
      // Drain everything available immediately (also while errors are latched)
      do {
        uint8_t d = (uint8_t)(s_huart_batt->Instance->RDR & 0xFF);
        if (wp < end)
          *wp++ = d;

        // Clear sticky errors so bytes keep flowing
        if (isr & USART_ISR_ORE)
          __HAL_UART_CLEAR_OREFLAG(s_huart_batt);
        if (isr & USART_ISR_FE)
          __HAL_UART_CLEAR_FEFLAG(s_huart_batt);
        if (isr & USART_ISR_NE)
          __HAL_UART_CLEAR_NEFLAG(s_huart_batt);
        if (isr & USART_ISR_PE)
          __HAL_UART_CLEAR_PEFLAG(s_huart_batt);

        isr = s_huart_batt->Instance->ISR;
      } while (isr & (USART_ISR_RXNE_RXFNE | USART_ISR_ORE | USART_ISR_FE |
                      USART_ISR_NE | USART_ISR_PE));

      last_activity = HAL_GetTick();

      // Early completion for short frames: raw tail 0xCC
      size_t n = (size_t)(wp - rx_buf);
      if (!is_long && n >= 8 && rx_buf[n - 1] == 0xCC) {
        *rx_len = (uint8_t)((n > 255) ? 255 : n);
        dump_bytes(" RX(raw)   ", rx_buf, *rx_len);
        for (uint8_t i = 0; i < *rx_len; ++i)
          rx_buf[i] = reverse_byte(rx_buf[i]);
        dump_bytes(" RX(bitrev)", rx_buf, *rx_len);
        bool ok = check_crc(rx_buf, *rx_len);
        b_log("[XGT] early CRC %s, len=%u\r\n", ok ? "OK" : "FAIL", *rx_len);
        return ok ? 0 : -1;
      }
    } else {
      if ((HAL_GetTick() - last_activity) >= tail_silence_ms)
        break;
    }
  }

  size_t got = (size_t)(wp - rx_buf);
  if (got == 0) {
    b_log("[XGT] RX timeout (no bytes)\r\n");
    return 1;
  }

  *rx_len = (uint8_t)((got > 255) ? 255 : got);
  dump_bytes(" RX(raw)   ", rx_buf, *rx_len);
  for (uint8_t i = 0; i < *rx_len; ++i)
    rx_buf[i] = reverse_byte(rx_buf[i]);
  dump_bytes(" RX(bitrev)", rx_buf, *rx_len);

  bool ok = check_crc(rx_buf, *rx_len);
  b_log("[XGT] CRC %s, len=%u\r\n", ok ? "OK" : "FAIL", *rx_len);
  return ok ? 0 : -1;
}

// ========================= State machine ========================
typedef enum {
  S_IDLE = 0,
  S_WAKE,
  S_NUM_CHARGES,
  S_CELL_SIZE,
  S_PARALLEL_CNT,
  S_BATT_HEALTH,
  S_CHARGE,
  S_TEMPERATURE,
  S_PACK_VOLT,
  S_CELL_VOLTAGES,
  S_COMPLETE
} batt_state_t;

static batt_state_t s_state = S_IDLE;

static void log_summary(void) {
  int temp_t10 =
      (int)(temperature_ * 10.0f + (temperature_ >= 0 ? 0.5f : -0.5f));
  uint32_t pack_mv = (uint32_t)(pack_voltage_ * 1000.0f + 0.5f); // integer mV

  b_log("[XGT][cycle %lu] Chg:%u%% Health:%u%% Temp:%d.%dC Pack:%u.%03uV "
        "Charges:%u CellSize:%u mAh Parallel:%u\r\n",
        (unsigned long)s_cycle, charge_, batt_health_, temp_t10 / 10,
        (temp_t10 < 0 ? -temp_t10 % 10 : temp_t10 % 10), pack_mv / 1000,
        pack_mv % 1000, num_charges_, cell_size_, parallel_cnt_);
}

void battery_init(UART_HandleTypeDef *huart_batt,
                  UART_HandleTypeDef *huart_log,
                  FDCAN_HandleTypeDef *hfdcan) {
  s_huart_batt = huart_batt;
  s_huart_log = huart_log;
  s_hfdcan = hfdcan;
  s_last_update_ms = HAL_GetTick();
  s_state_start_ms = HAL_GetTick();
  s_state = S_IDLE;
  s_cycle = 0;

  b_log("[XGT] init (interval=%lums, FIFO ON)\r\n",
        (unsigned long)s_update_interval_ms);
}

void battery_task(void) {
  uint32_t now = HAL_GetTick();

  // Trigger a new cycle
  if (s_state == S_IDLE && (now - s_last_update_ms) > s_update_interval_ms) {
    s_state = S_WAKE;
    s_state_start_ms = now;
    s_current_cell = 0;
    s_cycle++;
  }
  if (s_state == S_IDLE)
    return;

  switch (s_state) {
  case S_WAKE:
    if ((now - s_state_start_ms) >= 10u) {
      // Send wake 0x00 and give the pack think-time
      uint8_t wake = 0x00;
      uart_clear_rx(s_huart_batt);
      HAL_HalfDuplex_EnableTransmitter(s_huart_batt);
      (void)HAL_UART_Transmit(s_huart_batt, &wake, 1, 5);
      HAL_HalfDuplex_EnableReceiver(s_huart_batt);
      HAL_Delay(70);
      s_state = S_NUM_CHARGES;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_NUM_CHARGES:
    if ((now - s_state_start_ms) >= 80u) {
      (void)send_battery(s_buf, CMD_NUM_CHARGES, sizeof(CMD_NUM_CHARGES),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        num_charges_ = le16_at(&s_buf[4]);
        b_log("[XGT] NumCharges=%u\r\n", num_charges_);
      }
      s_state = S_CELL_SIZE;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_CELL_SIZE:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_CELL_SIZE, sizeof(CMD_CELL_SIZE),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        uint16_t raw = s_buf[5]; // 1-byte raw, scaled to mAh
        cell_size_ = (uint16_t)(raw * 100u);
        b_log("[XGT] CellSize=%u mAh (raw=%u)\r\n", cell_size_, raw);
      }
      s_state = S_PARALLEL_CNT;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_PARALLEL_CNT:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_PARALLEL_CNT, sizeof(CMD_PARALLEL_CNT),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        parallel_cnt_ = s_buf[4];
        b_log("[XGT] Parallel=%u\r\n", parallel_cnt_);
      }
      s_state = S_BATT_HEALTH;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_BATT_HEALTH:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_BATT_HEALTH, sizeof(CMD_BATT_HEALTH),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        uint16_t health_raw = le16_at(&s_buf[4]);
        uint16_t raw_cell_size = (uint16_t)(cell_size_ / 100u);
        if (raw_cell_size > 0 && parallel_cnt_ > 0) {
          uint32_t denom = (uint32_t)raw_cell_size * (uint32_t)parallel_cnt_;
          batt_health_ = (uint16_t)((health_raw * 100u) / (denom ? denom : 1u));
        } else {
          batt_health_ = (uint16_t)((health_raw * 100u) / 255u);
        }
        if (batt_health_ > 100u)
          batt_health_ = 100u;
        b_log("[XGT] Health=%u%% (raw=%u)\r\n", batt_health_, health_raw);
      }
      s_state = S_CHARGE;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_CHARGE:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_CHARGE, sizeof(CMD_CHARGE), &s_rx_len);
      if (s_rx_len >= 8) {
        uint16_t charge_raw = le16_at(&s_buf[4]);
        // Makita reports SoC as an 8.8 fixed-point percent. Convert to whole
        // percent with rounding before saturating to 100.
        uint16_t percent = (uint16_t)((charge_raw + 0x80u) >> 8);
        if (percent > 100u)
          percent = 100u;
        charge_ = percent;
        b_log("[XGT] Charge=%u%% (raw=%u)\r\n", charge_, charge_raw);
      }
      s_state = S_TEMPERATURE;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_TEMPERATURE:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_TEMPERATURE, sizeof(CMD_TEMPERATURE),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        uint16_t temp_raw = le16_at(&s_buf[4]);
        // From original: T = -30 + (temp_raw - 2431)/10 → tenths: t10 =
        // temp_raw - 2731
        int32_t t10 = (int32_t)temp_raw - 2731;
        temperature_ = t10 / 10.0f;
        b_log("[XGT] Temp=%ld.%ldC (raw=%u)\r\n", t10 / 10,
              (t10 < 0 ? -t10 % 10 : t10 % 10), temp_raw);
      }
      s_state = S_PACK_VOLT;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_PACK_VOLT:
    if ((now - s_state_start_ms) >= 60u) {
      (void)send_battery(s_buf, CMD_PACK_VOLT, sizeof(CMD_PACK_VOLT),
                         &s_rx_len);
      if (s_rx_len >= 8) {
        uint16_t vraw = le16_at(&s_buf[4]); // mV
        pack_voltage_ = (float)vraw / 1000.0f;
        b_log("[XGT] Pack=%u mV (%u.%03u V)\r\n", vraw, vraw / 1000,
              vraw % 1000);
      }
      s_state = S_CELL_VOLTAGES;
      s_current_cell = 1;
      s_state_start_ms = HAL_GetTick();
    }
    break;

  case S_CELL_VOLTAGES:
    if ((now - s_state_start_ms) >= 50u) {
      if (s_current_cell <= 10) {
        uint8_t cmd[8];
        memcpy(cmd, CMD_CELL_V_BASE, 8);
        // Pre-apply nibble-reversal to address bytes (matches ESPHome)
        uint8_t addr = (uint8_t)(s_current_cell * 2);
        cmd[4] = reverse_nibbles_via_lookup(addr);
        cmd[1] = reverse_nibbles_via_lookup((uint8_t)(addr + 194));

        (void)send_battery(s_buf, cmd, 8, &s_rx_len);
        if (s_rx_len >= 8) {
          uint16_t mv = le16_at(&s_buf[4]);
          cell_voltages_[s_current_cell - 1] = (float)mv / 1000.0f;
          b_log("[XGT] Cell%u=%u mV\r\n", s_current_cell, mv);
        }
        s_current_cell++;
        s_state_start_ms = HAL_GetTick();
      } else {
        s_state = S_COMPLETE;
      }
    }
    break;

  case S_COMPLETE:
    b_log("[XGT] Cells: ");
    for (int i = 0; i < 10; i++) {
      uint16_t mv = (uint16_t)(cell_voltages_[i] * 1000.0f + 0.5f);
      b_log("%u%s", mv, (i == 9) ? "" : " ");
    }
    b_log("\r\n");
    log_summary();
    publish_fdcan_state();
    s_state = S_IDLE;
    s_last_update_ms = HAL_GetTick();
    break;

  default:
    s_state = S_IDLE;
    break;
  }
}
