#include "proto_emmc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "emmc_dat0_rx.pio.h"
#include "pico/stdlib.h"

#define EMMC_CMD_PIN 10
#define EMMC_CLK_PIN 11
#define EMMC_DAT0_PIN 12

#define EMMC_MIN_SPEED_HZ 100000u
#define EMMC_MAX_SPEED_HZ 4000000u
#define EMMC_MIN_IDPOLL_MS 100u
#define EMMC_MAX_IDPOLL_MS 5000u
#define EMMC_CMD_TIMEOUT_BITS 512u
#define EMMC_CMD1_RETRIES 40u
#define EMMC_CMD1_RETRY_DELAY_MS 1u
#define EMMC_CMDX_RETRIES 4u
#define EMMC_CMD7_RETRIES 6u
#define EMMC_DATA_TIMEOUT_BITS 20000u
#define EMMC_DUMP_BLOCK_SIZE 512u
#define EMMC_DUMP_CHUNK_BYTES 256u
#define EMMC_MAX_DUMP_CHUNK_BYTES 256u
#define EMMC_DUMP_READ_RETRIES 6u
#define EMMC_DUMP_AUTO_RETRIES_DEFAULT 4u
#define EMMC_DUMP_VERIFY_RETRIES_DEFAULT 1u
#define EMMC_DUMP_VERIFY_RETRIES_MAX 8u
#define EMMC_DUMP_AUTO_RETRIES_MAX 32u
#define EMMC_DUMP_WS_INTERVAL_MS 0u
#define EMMC_DUMP_WS_DATA_BYTES 256u
#define EMMC_DUMP_BURST_CHUNKS 4u
#define EMMC_DUMP_TX_FAIL_LIMIT 40u
#define EMMC_DUMP_HEARTBEAT_MS 1000u
#define EMMC_DUMP_STALL_TIMEOUT_MS 4000u
#define EMMC_PIO_RX_BYTES (EMMC_DUMP_BLOCK_SIZE + 2u)
#define EMMC_RESELECT_EVERY_BLOCKS 16u
#define EMMC_CMD17_R1_EXTRA_TRIES 3u
#define EMMC_DUMP_MIN_SPEED_HZ 100000u
#define EMMC_DUMP_INTER_BLOCK_IDLE_CLKS 8u
#define EMMC_CMD_RETRY_IDLE_CLKS_DEFAULT 8u
#define EMMC_READ_BLOCK_WATCHDOG_MS 6000u
#define EMMC_FULL_REPREPARE_MAX 2u
#define WS_BIN_MAGIC 0xB0u
#define WS_CH_EMMC_DUMP_DATA 0x04u
#define WS_CH_EMMC_DUMP_RUN 0x05u

typedef struct {
  bool tristate_default;
  bool pullups_enabled;
  bool detect_enabled;
  bool idpoll_enabled;
  bool idpoll_monitor_between;
  uint32_t speed_hz;
  uint32_t idpoll_interval_ms;
  uint8_t retry_idle_clks;
  uint32_t last_detect_ms;
  uint32_t last_idpoll_ms;
  uint32_t tr_cmd;
  uint32_t tr_clk;
  uint32_t tr_dat0;
  uint32_t base_tr_cmd;
  uint32_t base_tr_clk;
  uint32_t base_tr_dat0;
  uint32_t half_period_cycles;
  uint8_t lv_cmd;
  uint8_t lv_clk;
  uint8_t lv_dat0;
  bool cmd_output;
  bool cmd_open_drain;
  uint32_t poll_count;
  bool dump_active;
  bool dump_hc_addressing;
  uint16_t dump_rca;
  uint32_t dump_start_lba;
  uint32_t dump_total_blocks;
  uint32_t dump_done_blocks;
  uint16_t dump_chunk_bytes;
  bool dump_double_read;
  bool dump_use_pio;
  uint8_t dump_verify_retries;
  uint8_t dump_auto_retries;
  uint8_t dump_block_retry_count;
  uint8_t dump_tx_fail_streak;
  bool dump_block_loaded;
  uint16_t dump_chunk_off;
  uint32_t dump_stat_read_attempts;
  uint32_t dump_stat_read_success;
  uint32_t dump_stat_cmd17_tries;
  uint32_t dump_stat_no_r1;
  uint32_t dump_stat_data_fail;
  uint32_t dump_stat_crc_fail;
  uint32_t dump_stat_resync_fail;
  uint32_t dump_stat_tx_busy;
  uint32_t dump_stat_block_normal;
  uint32_t dump_stat_block_all00;
  uint32_t dump_stat_block_allff;
  uint8_t dump_block_buf[EMMC_DUMP_BLOCK_SIZE];
  uint8_t dump_check_buf[EMMC_DUMP_BLOCK_SIZE];
  uint32_t last_dump_ms;
  uint32_t dump_last_progress_ms;
  uint32_t dump_last_status_ms;
} emmc_state_t;

static emmc_state_t g_emmc = {
    .tristate_default = true,
    .pullups_enabled = true,
    .detect_enabled = false,
    .idpoll_enabled = false,
    .idpoll_monitor_between = true,
    .speed_hz = 400000,
    .idpoll_interval_ms = 500,
    .retry_idle_clks = EMMC_CMD_RETRY_IDLE_CLKS_DEFAULT,
    .cmd_output = false,
    .cmd_open_drain = false,
    .dump_active = false,
    .dump_hc_addressing = false,
    .dump_rca = 1u,
    .dump_start_lba = 0u,
    .dump_total_blocks = 0u,
    .dump_done_blocks = 0u,
    .dump_chunk_bytes = EMMC_DUMP_CHUNK_BYTES,
    .dump_double_read = false,
    .dump_use_pio = true,
    .dump_verify_retries = EMMC_DUMP_VERIFY_RETRIES_DEFAULT,
    .dump_auto_retries = EMMC_DUMP_AUTO_RETRIES_DEFAULT,
    .dump_block_retry_count = 0u,
    .dump_tx_fail_streak = 0u,
    .dump_block_loaded = false,
    .dump_chunk_off = 0u,
    .dump_stat_read_attempts = 0u,
    .dump_stat_read_success = 0u,
    .dump_stat_cmd17_tries = 0u,
    .dump_stat_no_r1 = 0u,
    .dump_stat_data_fail = 0u,
    .dump_stat_crc_fail = 0u,
    .dump_stat_resync_fail = 0u,
    .dump_stat_tx_busy = 0u,
    .dump_stat_block_normal = 0u,
    .dump_stat_block_all00 = 0u,
    .dump_stat_block_allff = 0u,
    .last_dump_ms = 0u,
    .dump_last_progress_ms = 0u,
    .dump_last_status_ms = 0u,
};

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }
static void emmc_dbg(uint8_t level, const char *msg) { (void)app_debug_log(level, "emmc", msg); }
static char g_emmc_data_err[96];
static void emmc_set_data_err(const char *msg) {
  if (!msg) msg = "";
  snprintf(g_emmc_data_err, sizeof(g_emmc_data_err), "%s", msg);
}
static void wr_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu); }
static void wr_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static bool starts_with(const char *s, const char *prefix) {
  size_t n = strlen(prefix);
  return strncmp(s, prefix, n) == 0;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint8_t clamp_retry_idle_clks(uint32_t v) {
  if (v == 0u || v == 1u || v == 4u || v == 8u) return (uint8_t)v;
  if (v <= 1u) return 1u;
  if (v <= 4u) return 4u;
  return 8u;
}

static bool json_extract_u32(const char *json, const char *key, uint32_t *out) {
  char pattern[40];
  const char *p;
  unsigned v;

  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  if (sscanf(p, "%u", &v) != 1) return false;
  *out = v;
  return true;
}

static inline void emmc_delay_half(void) {
  if (g_emmc.half_period_cycles > 0u) busy_wait_at_least_cycles(g_emmc.half_period_cycles);
  tight_loop_contents();
}

static void emmc_update_timing(void) {
  uint32_t speed = g_emmc.speed_hz;
  uint32_t sys_hz;
  uint32_t cycles;
  if (speed == 0u) speed = EMMC_MIN_SPEED_HZ;
  sys_hz = clock_get_hz(clk_sys);
  if (sys_hz == 0u) sys_hz = 125000000u;
  cycles = (sys_hz + speed) / (speed * 2u);
  if (cycles == 0u) cycles = 1u;
  g_emmc.half_period_cycles = cycles;
}

static bool json_extract_bool(const char *json, const char *key, bool *out) {
  char pattern[40];
  const char *p;
  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  if (strncmp(p, "true", 4) == 0) {
    *out = true;
    return true;
  }
  if (strncmp(p, "false", 5) == 0) {
    *out = false;
    return true;
  }
  return false;
}

static void pin_input(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_disable_pulls(pin);
  if (g_emmc.pullups_enabled) gpio_pull_up(pin);
}

static void pin_output(uint pin, bool value) {
  gpio_init(pin);
  // Preload output latch before driving the pin to avoid enable-edge glitches.
  gpio_put(pin, value ? 1u : 0u);
  gpio_set_dir(pin, GPIO_OUT);
}

static void emmc_apply_safe_io(void) {
  pin_input(EMMC_CMD_PIN);
  pin_input(EMMC_CLK_PIN);
  pin_input(EMMC_DAT0_PIN);
  g_emmc.cmd_output = false;
}

static void emmc_bus_prepare(void) {
  pin_output(EMMC_CLK_PIN, true);
  pin_input(EMMC_CMD_PIN);
  g_emmc.cmd_output = false;
  pin_input(EMMC_DAT0_PIN);
}

static inline void emmc_cmd_set_output(bool level) {
  if (g_emmc.cmd_open_drain) {
    // Open-drain CMD behavior:
    // - logic 0: drive low
    // - logic 1: release line (input + pull-up)
    if (level) {
      if (g_emmc.cmd_output) {
        gpio_set_dir(EMMC_CMD_PIN, GPIO_IN);
        g_emmc.cmd_output = false;
      }
      gpio_disable_pulls(EMMC_CMD_PIN);
      if (g_emmc.pullups_enabled) gpio_pull_up(EMMC_CMD_PIN);
      return;
    }
    if (!g_emmc.cmd_output) {
      gpio_put(EMMC_CMD_PIN, 0u);
      gpio_set_dir(EMMC_CMD_PIN, GPIO_OUT);
      g_emmc.cmd_output = true;
    } else {
      gpio_put(EMMC_CMD_PIN, 0u);
    }
    return;
  }

  // Push-pull CMD behavior for transfer phase.
  if (!g_emmc.cmd_output) {
    gpio_put(EMMC_CMD_PIN, level ? 1u : 0u);
    gpio_set_dir(EMMC_CMD_PIN, GPIO_OUT);
    g_emmc.cmd_output = true;
  } else {
    gpio_put(EMMC_CMD_PIN, level ? 1u : 0u);
  }
}

static inline void emmc_cmd_set_input(void) {
  if (g_emmc.cmd_output) {
    gpio_set_dir(EMMC_CMD_PIN, GPIO_IN);
    g_emmc.cmd_output = false;
  }
  gpio_disable_pulls(EMMC_CMD_PIN);
  if (g_emmc.pullups_enabled) gpio_pull_up(EMMC_CMD_PIN);
}

static inline void emmc_clock_bit_out(uint8_t bit) {
  // SD/eMMC samples command/data on CLK rising edge; update CMD during low phase
  // (after falling edge) to maximize setup/hold margin on analyzers and targets.
    //emmc_delay_half();
  emmc_cmd_set_output(bit ? true : false);
  gpio_put(EMMC_CLK_PIN, 0u);
  emmc_delay_half();
  gpio_put(EMMC_CLK_PIN, 1u);
  emmc_delay_half();
}

static inline uint8_t emmc_clock_bit_in_cmd(void) {
  uint8_t bit;
  bit = (uint8_t)gpio_get(EMMC_CMD_PIN);
  gpio_put(EMMC_CLK_PIN, 0u);
  emmc_delay_half();
  gpio_put(EMMC_CLK_PIN, 1u);
  emmc_delay_half();
  //bit = (uint8_t)gpio_get(EMMC_CMD_PIN);
  return bit;
}

static inline uint8_t emmc_clock_bit_in_dat0(void) {
  uint8_t bit;
  bit = (uint8_t)gpio_get(EMMC_DAT0_PIN);
  gpio_put(EMMC_CLK_PIN, 0u);
  emmc_delay_half();
  gpio_put(EMMC_CLK_PIN, 1u);
  emmc_delay_half();
  //bit = (uint8_t)gpio_get(EMMC_DAT0_PIN);
  return bit;
}

static uint8_t crc7_bytes(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  size_t i;
  for (i = 0; i < len; i++) {
    uint8_t d = data[i];
    uint8_t b;
    for (b = 0; b < 8; b++) {
      crc <<= 1u;
      if (((d ^ crc) & 0x80u) != 0u) crc ^= 0x09u;
      d <<= 1u;
    }
  }
  return (uint8_t)(crc & 0x7Fu);
}

static void bitbuf_set(uint8_t *buf, uint16_t bit_index, uint8_t bit) {
  uint16_t byte_index = (uint16_t)(bit_index >> 3);
  uint8_t mask = (uint8_t)(1u << (7u - (bit_index & 7u)));
  if (bit) buf[byte_index] |= mask;
}

static uint8_t bitbuf_get(const uint8_t *buf, uint16_t bit_index) {
  uint16_t byte_index = (uint16_t)(bit_index >> 3);
  uint8_t mask = (uint8_t)(1u << (7u - (bit_index & 7u)));
  return (buf[byte_index] & mask) ? 1u : 0u;
}

static uint32_t bitbuf_get_u32(const uint8_t *buf, uint16_t start_bit, uint8_t count) {
  uint32_t v = 0;
  uint8_t i;
  for (i = 0; i < count; i++) {
    v = (uint32_t)((v << 1u) | bitbuf_get(buf, (uint16_t)(start_bit + i)));
  }
  return v;
}

static uint32_t rd_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static uint32_t rd_le24(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u);
}

static uint8_t bitrev8(uint8_t x) {
  x = (uint8_t)(((x & 0xF0u) >> 4u) | ((x & 0x0Fu) << 4u));
  x = (uint8_t)(((x & 0xCCu) >> 2u) | ((x & 0x33u) << 2u));
  x = (uint8_t)(((x & 0xAAu) >> 1u) | ((x & 0x55u) << 1u));
  return x;
}

static uint16_t crc16_ccitt_bytes(const uint8_t *data, size_t len) {
  uint16_t crc = 0u;
  size_t i;
  if (!data) return 0u;
  for (i = 0; i < len; i++) {
    uint8_t bit;
    crc ^= (uint16_t)((uint16_t)data[i] << 8u);
    for (bit = 0; bit < 8u; bit++) {
      if (crc & 0x8000u) crc = (uint16_t)((crc << 1u) ^ 0x1021u);
      else crc <<= 1u;
    }
  }
  return crc;
}

static bool emmc_read_response_bits(uint16_t total_bits, uint8_t *out, size_t out_size) {
  uint16_t start_scan;
  uint16_t bit_pos = 0;
  size_t needed = (size_t)((total_bits + 7u) >> 3u);
  if (!out || out_size < needed || total_bits == 0u) return false;
  memset(out, 0, out_size);

  for (start_scan = 0; start_scan < EMMC_CMD_TIMEOUT_BITS; start_scan++) {
    uint8_t b = emmc_clock_bit_in_cmd();
    if (b == 0u) {
      bitbuf_set(out, bit_pos++, 0u);
      break;
    }
  }
  if (bit_pos == 0u) return false;

  while (bit_pos < total_bits) {
    bitbuf_set(out, bit_pos, emmc_clock_bit_in_cmd());
    bit_pos++;
  }
  return true;
}

static bool emmc_send_cmd_raw(uint8_t cmd, uint32_t arg, uint16_t resp_bits, uint8_t *resp, size_t resp_size) {
  uint8_t frame[6];
  uint8_t i;
  emmc_cmd_set_output(true);

  frame[0] = (uint8_t)(0x40u | (cmd & 0x3Fu));
  frame[1] = (uint8_t)(arg >> 24u);
  frame[2] = (uint8_t)(arg >> 16u);
  frame[3] = (uint8_t)(arg >> 8u);
  frame[4] = (uint8_t)(arg);
  frame[5] = (uint8_t)((crc7_bytes(frame, 5) << 1u) | 1u);

  for (i = 0; i < 6; i++) {
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
      emmc_clock_bit_out((uint8_t)((frame[i] >> (7u - bit)) & 1u));
    }
  }

  emmc_cmd_set_input();
  if (resp_bits == 0u) return true;
  return emmc_read_response_bits(resp_bits, resp, resp_size);
}

static void emmc_send_idle_clocks(uint32_t count) {
  uint32_t i;
  emmc_cmd_set_output(true);
  for (i = 0; i < count; i++) emmc_clock_bit_out(1u);
  emmc_cmd_set_input();
}

static inline void emmc_send_retry_idle(void) {
  // Software dump path is sensitive to added retry clocks; keep retries tight there.
  if (g_emmc.dump_active && !g_emmc.dump_use_pio) return;
  if (g_emmc.retry_idle_clks == 0u) return;
  emmc_send_idle_clocks((uint32_t)g_emmc.retry_idle_clks);
}

typedef struct {
  bool ok;
  uint32_t ocr;
  uint16_t rca;
  uint8_t cid[15];
  uint8_t csd[15];
  char msg[96];
} emmc_id_data_t;

static void hex_bytes(const uint8_t *buf, size_t len, char *out, size_t out_len) {
  static const char hex[] = "0123456789ABCDEF";
  size_t i;
  size_t p = 0;
  if (!out || out_len == 0u) return;
  for (i = 0; i < len && (p + 2u) < out_len; i++) {
    out[p++] = hex[(buf[i] >> 4) & 0x0Fu];
    out[p++] = hex[buf[i] & 0x0Fu];
  }
  out[p] = 0;
}

static void r2_extract_payload_120(const uint8_t *r2_136, uint8_t out15[15]) {
  uint16_t i;
  memset(out15, 0, 15);
  for (i = 0; i < 120u; i++) {
    bitbuf_set(out15, i, bitbuf_get(r2_136, (uint16_t)(2u + i)));
  }
}

static void snapshot_levels(void);
static bool emmc_prepare_card_for_data(uint16_t *out_rca, bool *out_hc, char *msg, size_t msg_len);
static bool emmc_resync_transfer(uint16_t rca, bool hc_addressing);

static bool emmc_try_read_ids(emmc_id_data_t *out) {
  uint8_t r1[6];
  uint8_t r2[17];
  static const uint32_t cmd1_args[] = {0x40FF8000u, 0x00FF8000u, 0x00000000u};
  uint32_t arg_i;
  uint32_t retries;
  uint32_t cmdx_try;
  uint32_t ocr = 0;
  uint16_t rca = 1u;
  bool saw_cmd1_response = false;
  bool timing_overridden = false;
  uint32_t saved_speed = 0;
  if (!out) return false;
  memset(out, 0, sizeof(*out));
  g_emmc.cmd_open_drain = false; //was true

#define EMMC_RESTORE_TIMING() \
  do { \
    if (timing_overridden) { \
      g_emmc.speed_hz = saved_speed; \
      emmc_update_timing(); \
    } \
  } while (0)

  // Use conservative init speed for CMD0/CMD1; many setups are marginal at higher rates.
  if (g_emmc.speed_hz > 200000u) {
    saved_speed = g_emmc.speed_hz;
    g_emmc.speed_hz = 200000u;
    emmc_update_timing();
    timing_overridden = true;
  }

  emmc_bus_prepare();
  emmc_send_idle_clocks(160u);

  (void)emmc_send_cmd_raw(0u, 0u, 0u, NULL, 0u);
  emmc_send_idle_clocks(32u);
  sleep_ms(1u);
  (void)emmc_send_cmd_raw(0u, 0u, 0u, NULL, 0u);
  emmc_send_idle_clocks(32u);

  for (arg_i = 0; arg_i < (sizeof(cmd1_args) / sizeof(cmd1_args[0])); arg_i++) {
    for (retries = 0; retries < EMMC_CMD1_RETRIES; retries++) {
      if (emmc_send_cmd_raw(1u, cmd1_args[arg_i], 48u, r1, sizeof(r1))) {
        saw_cmd1_response = true;
        ocr = bitbuf_get_u32(r1, 8u, 32u);
        if ((ocr & 0x80000000u) != 0u) break;
      }
      emmc_send_retry_idle();
      sleep_ms(EMMC_CMD1_RETRY_DELAY_MS);
    }
    if ((ocr & 0x80000000u) != 0u) break;
  }
  if ((ocr & 0x80000000u) == 0u) {
    if (!saw_cmd1_response) {
      snapshot_levels();
      snprintf(out->msg, sizeof(out->msg),
               "CMD1 timeout (no response on CMD line; CMD=%u CLK=%u DAT0=%u)",
               (unsigned)g_emmc.lv_cmd, (unsigned)g_emmc.lv_clk, (unsigned)g_emmc.lv_dat0);
    } else {
      snprintf(out->msg, sizeof(out->msg), "CMD1 timeout (OCR busy never set)");
    }
    EMMC_RESTORE_TIMING();
    return false;
  }
  out->ocr = ocr;

  // Give the card a short settle window after OCR ready.
  emmc_send_idle_clocks(16u);

  for (cmdx_try = 0; cmdx_try < EMMC_CMDX_RETRIES; cmdx_try++) {
    if (emmc_send_cmd_raw(2u, 0u, 136u, r2, sizeof(r2))) break;
    emmc_send_retry_idle();
  }
  if (cmdx_try >= EMMC_CMDX_RETRIES) {
    snprintf(out->msg, sizeof(out->msg), "CMD2 failed (no CID response)");
    EMMC_RESTORE_TIMING();
    return false;
  }
  r2_extract_payload_120(r2, out->cid);
  emmc_send_retry_idle();

  for (cmdx_try = 0; cmdx_try < EMMC_CMDX_RETRIES; cmdx_try++) {
    if (emmc_send_cmd_raw(3u, ((uint32_t)rca) << 16u, 48u, r1, sizeof(r1))) break;
    emmc_send_retry_idle();
  }
  if (cmdx_try >= EMMC_CMDX_RETRIES) {
    snprintf(out->msg, sizeof(out->msg), "CMD3 failed (no RCA response)");
    EMMC_RESTORE_TIMING();
    return false;
  }
  if (bitbuf_get_u32(r1, 2u, 6u) != 3u) {
    snprintf(out->msg, sizeof(out->msg), "CMD3 invalid response");
    EMMC_RESTORE_TIMING();
    return false;
  }
  emmc_send_retry_idle();

  for (cmdx_try = 0; cmdx_try < EMMC_CMDX_RETRIES; cmdx_try++) {
    if (emmc_send_cmd_raw(9u, ((uint32_t)rca) << 16u, 136u, r2, sizeof(r2))) break;
    emmc_send_retry_idle();
  }
  if (cmdx_try >= EMMC_CMDX_RETRIES) {
    snprintf(out->msg, sizeof(out->msg), "CMD9 failed (no CSD response)");
    EMMC_RESTORE_TIMING();
    return false;
  }
  r2_extract_payload_120(r2, out->csd);

  out->rca = rca;
  out->ok = true;
  EMMC_RESTORE_TIMING();
#undef EMMC_RESTORE_TIMING
  return true;
}

static void snapshot_levels(void) {
  g_emmc.lv_cmd = (uint8_t)gpio_get(EMMC_CMD_PIN);
  g_emmc.lv_clk = (uint8_t)gpio_get(EMMC_CLK_PIN);
  g_emmc.lv_dat0 = (uint8_t)gpio_get(EMMC_DAT0_PIN);
}

static void poll_levels_and_transitions(void) {
  uint8_t v;
  v = (uint8_t)gpio_get(EMMC_CMD_PIN);
  if (v != g_emmc.lv_cmd) g_emmc.tr_cmd++;
  g_emmc.lv_cmd = v;
  v = (uint8_t)gpio_get(EMMC_CLK_PIN);
  if (v != g_emmc.lv_clk) g_emmc.tr_clk++;
  g_emmc.lv_clk = v;
  v = (uint8_t)gpio_get(EMMC_DAT0_PIN);
  if (v != g_emmc.lv_dat0) g_emmc.tr_dat0++;
  g_emmc.lv_dat0 = v;
}

static void set_idpoll_baseline(void) {
  g_emmc.base_tr_cmd = g_emmc.tr_cmd;
  g_emmc.base_tr_clk = g_emmc.tr_clk;
  g_emmc.base_tr_dat0 = g_emmc.tr_dat0;
}

static void send_pins(void) {
  char out[320];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.pins\",\"cmd\":%u,\"clk\":%u,\"dat0\":%u,"
           "\"transitions\":{\"cmd\":%lu,\"clk\":%lu,\"dat0\":%lu}}",
           (unsigned)g_emmc.lv_cmd, (unsigned)g_emmc.lv_clk, (unsigned)g_emmc.lv_dat0,
           (unsigned long)g_emmc.tr_cmd, (unsigned long)g_emmc.tr_clk, (unsigned long)g_emmc.tr_dat0);
  (void)app_send_text(out);
}

static void send_config(void) {
  char out[512];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.config\",\"port\":\"emmc0\",\"status\":\"id-read\","
           "\"pins\":{\"cmd\":%u,\"clk\":%u,\"dat0\":%u},"
           "\"tristate_default\":%s,\"pullups_enabled\":%s,"
           "\"speed_hz\":%lu,\"idpoll_interval_ms\":%lu,\"retry_idle_clks\":%u,"
           "\"detect_enabled\":%s,\"idpoll_enabled\":%s,\"idpoll_monitor_between\":%s}",
           (unsigned)EMMC_CMD_PIN, (unsigned)EMMC_CLK_PIN, (unsigned)EMMC_DAT0_PIN,
           g_emmc.tristate_default ? "true" : "false",
           g_emmc.pullups_enabled ? "true" : "false",
           (unsigned long)g_emmc.speed_hz, (unsigned long)g_emmc.idpoll_interval_ms,
           (unsigned)g_emmc.retry_idle_clks,
           g_emmc.detect_enabled ? "true" : "false", g_emmc.idpoll_enabled ? "true" : "false",
           g_emmc.idpoll_monitor_between ? "true" : "false");
  (void)app_send_text(out);
}

static void send_idpoll_window(void) {
  uint32_t d_cmd = g_emmc.tr_cmd - g_emmc.base_tr_cmd;
  uint32_t d_clk = g_emmc.tr_clk - g_emmc.base_tr_clk;
  uint32_t d_dat0 = g_emmc.tr_dat0 - g_emmc.base_tr_dat0;
  uint32_t total = d_cmd + d_clk + d_dat0;
  char out[256];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.idpoll.window\",\"clear\":%s,\"total\":%lu,"
           "\"transitions\":{\"cmd\":%lu,\"clk\":%lu,\"dat0\":%lu}}",
           total == 0u ? "true" : "false", (unsigned long)total, (unsigned long)d_cmd,
           (unsigned long)d_clk, (unsigned long)d_dat0);
  (void)app_send_text(out);
}

static void send_cmd_test_result(uint32_t cycles) {
  char out[128];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.cmd_test.result\",\"ok\":true,\"cycles\":%lu}",
           (unsigned long)cycles);
  (void)app_send_text(out);
}

static bool send_dump_status(const char *state, const char *detail) {
  char out[512];
  uint32_t attempts = g_emmc.dump_stat_read_attempts;
  uint32_t ok = g_emmc.dump_stat_read_success;
  uint32_t err = (attempts >= ok) ? (attempts - ok) : 0u;
  uint32_t err_pct_x10 = (attempts > 0u) ? (uint32_t)((err * 1000u) / attempts) : 0u;
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.dump.status\",\"state\":\"%s\",\"detail\":\"%s\","
           "\"start_lba\":%lu,\"current_lba\":%lu,\"done_blocks\":%lu,\"total_blocks\":%lu,"
           "\"read_attempts\":%lu,\"read_ok\":%lu,\"read_err\":%lu,\"read_err_pct_x10\":%lu,"
           "\"cmd17_tries\":%lu,\"no_r1\":%lu,\"data_fail\":%lu,\"crc_fail\":%lu,"
           "\"resync_fail\":%lu,\"tx_busy\":%lu,"
           "\"block_normal\":%lu,\"block_all00\":%lu,\"block_allff\":%lu}",
           state ? state : "status", detail ? detail : "",
           (unsigned long)g_emmc.dump_start_lba,
           (unsigned long)(g_emmc.dump_start_lba + g_emmc.dump_done_blocks),
           (unsigned long)g_emmc.dump_done_blocks, (unsigned long)g_emmc.dump_total_blocks,
           (unsigned long)attempts, (unsigned long)ok, (unsigned long)err, (unsigned long)err_pct_x10,
           (unsigned long)g_emmc.dump_stat_cmd17_tries, (unsigned long)g_emmc.dump_stat_no_r1,
           (unsigned long)g_emmc.dump_stat_data_fail, (unsigned long)g_emmc.dump_stat_crc_fail,
           (unsigned long)g_emmc.dump_stat_resync_fail, (unsigned long)g_emmc.dump_stat_tx_busy,
           (unsigned long)g_emmc.dump_stat_block_normal,
           (unsigned long)g_emmc.dump_stat_block_all00,
           (unsigned long)g_emmc.dump_stat_block_allff);
  return app_send_text(out);
}

static bool send_dump_data_bin(uint32_t offset, const uint8_t *data, uint16_t count) {
  uint8_t fr[8u + EMMC_DUMP_WS_DATA_BYTES];
  if (!data || count == 0u || count > EMMC_DUMP_WS_DATA_BYTES) return false;
  fr[0] = WS_BIN_MAGIC;
  fr[1] = WS_CH_EMMC_DUMP_DATA;
  wr_le32(&fr[2], offset);
  wr_le16(&fr[6], count);
  memcpy(&fr[8], data, count);
  return app_send_binary(fr, (uint16_t)(8u + count));
}

static bool send_dump_run_bin(uint32_t offset, uint16_t count, uint8_t fill) {
  uint8_t fr[9];
  if (count == 0u) return false;
  fr[0] = WS_BIN_MAGIC;
  fr[1] = WS_CH_EMMC_DUMP_RUN;
  wr_le32(&fr[2], offset);
  wr_le16(&fr[6], count);
  fr[8] = fill;
  return app_send_binary(fr, (uint16_t)sizeof(fr));
}

static void send_cmd1_test_result(bool saw_response, bool ready, uint32_t responses,
                                  uint32_t retries_done, uint32_t arg, uint32_t ocr_last) {
  char out[224];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.cmd1_test.result\",\"ok\":true,\"saw_response\":%s,"
           "\"ready\":%s,\"responses\":%lu,\"retries\":%lu,\"arg\":%lu,\"ocr\":%lu}",
           saw_response ? "true" : "false", ready ? "true" : "false",
           (unsigned long)responses, (unsigned long)retries_done,
           (unsigned long)arg, (unsigned long)ocr_last);
  (void)app_send_text(out);
}

static void emmc_dbg_cmd1_attempt(uint32_t attempt_idx, bool got_resp, const uint8_t r1[6]) {
  char dbg[224];
  if (!got_resp) {
    snapshot_levels();
    snprintf(dbg, sizeof(dbg),
             "cmd1_test try %lu: no response (CMD=%u CLK=%u DAT0=%u)",
             (unsigned long)(attempt_idx + 1u),
             (unsigned)g_emmc.lv_cmd, (unsigned)g_emmc.lv_clk, (unsigned)g_emmc.lv_dat0);
    emmc_dbg(2, dbg);
    return;
  }

  if (r1) {
    char r1_hex[20];
    uint32_t cmdidx = bitbuf_get_u32(r1, 2u, 6u);
    uint32_t ocr = bitbuf_get_u32(r1, 8u, 32u);
    uint32_t crc = bitbuf_get_u32(r1, 40u, 7u);
    uint32_t start = bitbuf_get(r1, 0u);
    uint32_t tx = bitbuf_get(r1, 1u);
    uint32_t end = bitbuf_get(r1, 47u);
    hex_bytes(r1, 6u, r1_hex, sizeof(r1_hex));
    snprintf(dbg, sizeof(dbg),
             "cmd1_test try %lu: r1=%s start=%lu tx=%lu cmd=%lu ocr=%08lX crc7=%02lX end=%lu",
             (unsigned long)(attempt_idx + 1u), r1_hex,
             (unsigned long)start, (unsigned long)tx, (unsigned long)cmdidx,
             (unsigned long)ocr, (unsigned long)crc, (unsigned long)end);
    emmc_dbg(2, dbg);
  }
}

static void send_sd_test_result(bool cmd8_ok, uint32_t cmd8_echo, uint32_t cmd8_r1,
                                bool cmd55_ok, bool acmd41_ok, bool ready, uint32_t attempts,
                                uint32_t acmd41_arg, uint32_t ocr) {
  char out[320];
  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.sd_test.result\",\"ok\":true,\"cmd8_ok\":%s,"
           "\"cmd8_echo\":%lu,\"cmd8_r1\":%lu,\"cmd55_ok\":%s,\"acmd41_ok\":%s,"
           "\"ready\":%s,\"attempts\":%lu,\"acmd41_arg\":%lu,\"ocr\":%lu}",
           cmd8_ok ? "true" : "false", (unsigned long)cmd8_echo, (unsigned long)cmd8_r1,
           cmd55_ok ? "true" : "false", acmd41_ok ? "true" : "false", ready ? "true" : "false",
           (unsigned long)attempts, (unsigned long)acmd41_arg, (unsigned long)ocr);
  (void)app_send_text(out);
}

static void emmc_emit_cmd_test(uint32_t cycles) {
  static const uint8_t pattern[6] = {0x5Au, 0xA5u, 0x3Cu, 0xC3u, 0xF0u, 0x0Fu};
  uint32_t i;
  uint8_t b;

  if (cycles == 0u) cycles = 1u;
  if (cycles > 64u) cycles = 64u;

  emmc_bus_prepare();
  emmc_send_idle_clocks(32u);
  for (i = 0; i < cycles; i++) {
    for (b = 0; b < 6u; b++) {
      uint8_t bit;
      for (bit = 0; bit < 8u; bit++) {
        emmc_clock_bit_out((uint8_t)((pattern[b] >> (7u - bit)) & 1u));
      }
    }
    emmc_send_idle_clocks(8u);
  }
  emmc_cmd_set_input();
  if (g_emmc.tristate_default) emmc_apply_safe_io();
  send_cmd_test_result(cycles);
}

static void emmc_emit_cmd1_test(uint32_t retries, uint32_t arg) {
  uint8_t r1[6];
  uint32_t i;
  uint32_t responses = 0;
  uint32_t ocr = 0;
  bool saw_response = false;
  bool ready = false;
  bool timing_overridden = false;
  uint32_t saved_speed = 0;

  if (retries == 0u) retries = 16u;
  if (retries > 200u) retries = 200u;
  g_emmc.cmd_open_drain = false; // push-pull for this hardware path

  if (g_emmc.speed_hz > 200000u) {
    saved_speed = g_emmc.speed_hz;
    g_emmc.speed_hz = 200000u;
    emmc_update_timing();
    timing_overridden = true;
  }

  emmc_bus_prepare();
  emmc_send_idle_clocks(160u);
  (void)emmc_send_cmd_raw(0u, 0u, 0u, NULL, 0u);
  emmc_send_idle_clocks(32u);
  sleep_ms(1u);
  (void)emmc_send_cmd_raw(0u, 0u, 0u, NULL, 0u);
  emmc_send_idle_clocks(32u);

  for (i = 0; i < retries; i++) {
    if (emmc_send_cmd_raw(1u, arg, 48u, r1, sizeof(r1))) {
      if (app_debug_level() >= 2u) emmc_dbg_cmd1_attempt(i, true, r1);
      saw_response = true;
      responses++;
      ocr = bitbuf_get_u32(r1, 8u, 32u);
      if ((ocr & 0x80000000u) != 0u) {
        ready = true;
        i++;
        break;
      }
    } else if (app_debug_level() >= 2u) {
      emmc_dbg_cmd1_attempt(i, false, NULL);
    }
    // Provide explicit Ncc clocks between repeated CMD1 attempts.
    emmc_send_retry_idle();
    sleep_ms(EMMC_CMD1_RETRY_DELAY_MS);
  }

  if (timing_overridden) {
    g_emmc.speed_hz = saved_speed;
    emmc_update_timing();
  }
  emmc_cmd_set_input();
  if (g_emmc.tristate_default) emmc_apply_safe_io();
  send_cmd1_test_result(saw_response, ready, responses, i, arg, ocr);
}

static void emmc_emit_sd_test(uint32_t retries, uint32_t acmd41_arg) {
  uint8_t r1[6];
  uint32_t i;
  bool cmd8_ok = false;
  uint32_t cmd8_echo = 0;
  uint32_t cmd8_r1 = 0;
  bool cmd55_ok = false;
  bool acmd41_ok = false;
  bool ready = false;
  uint32_t ocr = 0;
  bool timing_overridden = false;
  uint32_t saved_speed = 0;

  if (retries == 0u) retries = 24u;
  if (retries > 200u) retries = 200u;
  g_emmc.cmd_open_drain = false; //was true;

  if (g_emmc.speed_hz > 200000u) {
    saved_speed = g_emmc.speed_hz;
    g_emmc.speed_hz = 200000u;
    emmc_update_timing();
    timing_overridden = true;
  }

  emmc_bus_prepare();
  emmc_send_idle_clocks(160u);
  (void)emmc_send_cmd_raw(0u, 0u, 0u, NULL, 0u);
  emmc_send_idle_clocks(32u);
  sleep_ms(1u);

  if (emmc_send_cmd_raw(8u, 0x000001AAu, 48u, r1, sizeof(r1))) {
    cmd8_ok = true;
    cmd8_r1 = bitbuf_get_u32(r1, 8u, 32u);
    cmd8_echo = bitbuf_get_u32(r1, 40u, 8u);
  }
  emmc_send_retry_idle();

  for (i = 0; i < retries; i++) {
    if (emmc_send_cmd_raw(55u, 0u, 48u, r1, sizeof(r1))) cmd55_ok = true;
    if (emmc_send_cmd_raw(41u, acmd41_arg, 48u, r1, sizeof(r1))) {
      acmd41_ok = true;
      ocr = bitbuf_get_u32(r1, 8u, 32u);
      if ((ocr & 0x80000000u) != 0u) {
        ready = true;
        i++;
        break;
      }
    }
    emmc_send_retry_idle();
    sleep_ms(EMMC_CMD1_RETRY_DELAY_MS);
  }

  if (timing_overridden) {
    g_emmc.speed_hz = saved_speed;
    emmc_update_timing();
  }
  emmc_cmd_set_input();
  if (g_emmc.tristate_default) emmc_apply_safe_io();
  send_sd_test_result(cmd8_ok, cmd8_echo, cmd8_r1, cmd55_ok, acmd41_ok, ready, i, acmd41_arg, ocr);
}

static bool emmc_wait_dat0_start(void) {
  uint32_t i;
  emmc_cmd_set_input();
  for (i = 0; i < EMMC_DATA_TIMEOUT_BITS; i++) {
    if (emmc_clock_bit_in_dat0() == 0u) return true;
  }
  return false;
}

static bool emmc_read_data_block_512_sw(uint8_t out[EMMC_DUMP_BLOCK_SIZE]) {
  uint32_t i;
  if (!emmc_wait_dat0_start()) {
    emmc_set_data_err("SW wait DAT0 start timeout");
    return false;
  }
  for (i = 0; i < EMMC_DUMP_BLOCK_SIZE; i++) {
    uint8_t b = 0u;
    uint8_t bit;
    for (bit = 0; bit < 8u; bit++) {
      b = (uint8_t)((b << 1u) | emmc_clock_bit_in_dat0());
    }
    out[i] = b;
  }
  // Consume CRC16 + end bit.
  for (i = 0; i < 17u; i++) (void)emmc_clock_bit_in_dat0();
  emmc_set_data_err("ok");
  return true;
}

static bool emmc_read_data_block_512(uint8_t out[EMMC_DUMP_BLOCK_SIZE]) {
  static bool pio_init_done = false;
  static bool pio_available = false;
  static PIO pio = pio0;
  static uint sm = 0u;
  static uint pio_off = 0u;
  static uint8_t raw[EMMC_PIO_RX_BYTES];
  static uint8_t rev_data[EMMC_DUMP_BLOCK_SIZE];
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint16_t crc_rx_rev;
  uint16_t crc_calc_rev = 0u;
  uint32_t i;
  bool got = false;
  char dmsg[160];

  if (!out) return false;
  if (!g_emmc.dump_use_pio) {
    // Explicit non-PIO mode for dump path.
    return emmc_read_data_block_512_sw(out);
  }
  // Start-bit acquisition is handled in-PIO (wait_start loop) for lower overhead.
  if (app_debug_level() >= 3u) emmc_dbg(3, "pio: wait+capture start");

  if (!pio_init_done) {
    pio_init_done = true;
    // Prefer pio1 first to avoid contention with stacks that commonly use pio0.
    if (pio_can_add_program(pio1, &emmc_dat0_rx_program)) pio = pio1;
    else if (pio_can_add_program(pio0, &emmc_dat0_rx_program)) pio = pio0;
    else pio = NULL;
    if (pio) {
      sm = pio_claim_unused_sm(pio, false);
      if (sm != (uint)-1) {
        pio_off = pio_add_program(pio, &emmc_dat0_rx_program);
        pio_available = true;
        if (app_debug_level() >= 2u) {
          snprintf(dmsg, sizeof(dmsg), "pio: init ok pio=%s sm=%u off=%u",
                   (pio == pio0) ? "pio0" : "pio1", (unsigned)sm, (unsigned)pio_off);
          emmc_dbg(2, dmsg);
        }
      }
    }
    if (!pio_available && app_debug_level() >= 1u) emmc_dbg(1, "pio: init failed, unavailable");
    if (!pio_available) emmc_set_data_err("PIO unavailable");
  }

  if (pio_available) {
    pio_sm_config c = emmc_dat0_rx_program_get_default_config(pio_off);
    // PIO loop is 3 instructions/bit; tune divider so resulting bit clock matches target.
    float clkdiv = (float)clock_get_hz(clk_sys) / ((float)g_emmc.speed_hz * 3.0f);
    uint32_t timeout_us;
    uint64_t t_start_us;
    uint64_t tx_wait_start_us;
    uint32_t clkdiv_x1000;

    if (clkdiv < 1.0f) clkdiv = 1.0f;
    sm_config_set_in_pins(&c, EMMC_DAT0_PIN);
    sm_config_set_jmp_pin(&c, EMMC_DAT0_PIN);
    sm_config_set_sideset_pins(&c, EMMC_CLK_PIN);
    sm_config_set_in_shift(&c, false, true, 8);
    // Keep independent TX/RX FIFOs: TX is required for the 'pull' bit-count value.
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_gpio_init(pio, EMMC_CLK_PIN);
    pio_gpio_init(pio, EMMC_DAT0_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, EMMC_CLK_PIN, 1u, true);
    pio_sm_set_consecutive_pindirs(pio, sm, EMMC_DAT0_PIN, 1u, false);
    pio_sm_init(pio, sm, pio_off, &c);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    // Avoid blocking TX FIFO write; guard with timeout.
    tx_wait_start_us = time_us_64();
    while (pio_sm_is_tx_fifo_full(pio, sm)) {
      if ((time_us_64() - tx_wait_start_us) > 2000u) {
        emmc_set_data_err("PIO tx fifo full timeout");
        if (app_debug_level() >= 1u) emmc_dbg(1, "pio: tx fifo full timeout");
        return false;
      }
      tight_loop_contents();
    }
    pio_sm_put(pio, sm, (EMMC_PIO_RX_BYTES * 8u) - 1u);

    pio_sm_set_enabled(pio, sm, true);
    timeout_us = (uint32_t)(((uint64_t)(EMMC_PIO_RX_BYTES * 8u + EMMC_DATA_TIMEOUT_BITS + 256u) * 1000000ull) /
                            (uint64_t)g_emmc.speed_hz);
    if (timeout_us < 5000u) timeout_us = 5000u;
    if (app_debug_level() >= 3u) {
      clkdiv_x1000 = (uint32_t)(clkdiv * 1000.0f);
      snprintf(dmsg, sizeof(dmsg), "pio: start clkdiv_x1000=%lu timeout_us=%lu",
               (unsigned long)clkdiv_x1000, (unsigned long)timeout_us);
      emmc_dbg(3, dmsg);
    }
    t_start_us = time_us_64();
    for (i = 0; i < EMMC_PIO_RX_BYTES; i++) {
      while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if ((time_us_64() - t_start_us) > timeout_us) {
          pio_sm_set_enabled(pio, sm, false);
          pio_sm_clear_fifos(pio, sm);
          pio_sm_restart(pio, sm);
          got = false;
          emmc_set_data_err("PIO rx timeout");
          if (app_debug_level() >= 1u) {
            snprintf(dmsg, sizeof(dmsg), "pio: rx timeout at byte %lu/%u",
                     (unsigned long)i, (unsigned)EMMC_PIO_RX_BYTES);
            emmc_dbg(1, dmsg);
          }
          goto pio_restore;
        }
        tight_loop_contents();
      }
      // Non-blocking pop after explicit empty check: avoids any chance of wedging
      // on blocking FIFO helpers if state gets out of sync.
      raw[i] = (uint8_t)pio->rxf[sm];
      if ((i & 31u) == 0u) tight_loop_contents();
    }
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    got = true;

pio_restore:
    gpio_set_function(EMMC_CLK_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(EMMC_CLK_PIN, GPIO_OUT);
    gpio_put(EMMC_CLK_PIN, 1u);
    gpio_set_function(EMMC_DAT0_PIN, GPIO_FUNC_SIO);
    pin_input(EMMC_DAT0_PIN);

    if (got) {
      crc_calc = crc16_ccitt_bytes(raw, EMMC_DUMP_BLOCK_SIZE);
      crc_rx = (uint16_t)(((uint16_t)raw[EMMC_DUMP_BLOCK_SIZE] << 8u) | raw[EMMC_DUMP_BLOCK_SIZE + 1u]);
      if (crc_calc != crc_rx) {
        for (i = 0; i < EMMC_DUMP_BLOCK_SIZE; i++) rev_data[i] = bitrev8(raw[i]);
        crc_calc_rev = crc16_ccitt_bytes(rev_data, EMMC_DUMP_BLOCK_SIZE);
        crc_rx_rev = (uint16_t)(((uint16_t)bitrev8(raw[EMMC_DUMP_BLOCK_SIZE]) << 8u) |
                                bitrev8(raw[EMMC_DUMP_BLOCK_SIZE + 1u]));
        if (crc_calc_rev == crc_rx_rev) {
          if (app_debug_level() >= 2u) {
            snprintf(dmsg, sizeof(dmsg), "pio: crc ok (bit-reversed), crc=%04X", (unsigned)crc_calc_rev);
            emmc_dbg(2, dmsg);
          }
          memcpy(out, rev_data, EMMC_DUMP_BLOCK_SIZE);
          (void)emmc_clock_bit_in_dat0();  // consume end bit
          emmc_set_data_err("ok");
          return true;
        }
        if (app_debug_level() >= 2u) {
          snprintf(dmsg, sizeof(dmsg), "pio: crc mismatch raw=%04X rx=%04X rev=%04X rx_rev=%04X",
                   (unsigned)crc_calc, (unsigned)crc_rx, (unsigned)crc_calc_rev, (unsigned)crc_rx_rev);
          emmc_dbg(2, dmsg);
        }
      } else {
        if (app_debug_level() >= 2u) {
          snprintf(dmsg, sizeof(dmsg), "pio: crc ok raw=%04X", (unsigned)crc_calc);
          emmc_dbg(2, dmsg);
        }
        memcpy(out, raw, EMMC_DUMP_BLOCK_SIZE);
        (void)emmc_clock_bit_in_dat0();  // consume end bit
        emmc_set_data_err("ok");
        return true;
      }
      // If CRC check did not match either orientation, fall back to software path.
      g_emmc.dump_stat_crc_fail++;
      emmc_set_data_err("PIO CRC mismatch");
      pio_sm_set_enabled(pio, sm, false);
      pio_sm_clear_fifos(pio, sm);
      pio_sm_restart(pio, sm);
    }
  }
  emmc_set_data_err("PIO block read failed");
  if (app_debug_level() >= 2u) emmc_dbg(2, "pio: block read failed");
  return false;
}

static bool emmc_read_ext_csd(uint8_t out[EMMC_DUMP_BLOCK_SIZE], char *msg, size_t msg_len) {
  uint16_t rca = 1u;
  bool hc = false;
  uint8_t r1[6];
  uint32_t i;
  uint32_t attempt;
  bool cmd8_ok;
  bool data_ok;
  if (!out) return false;
  if (!emmc_prepare_card_for_data(&rca, &hc, msg, msg_len)) return false;

  for (attempt = 0; attempt < EMMC_DUMP_READ_RETRIES; attempt++) {
    cmd8_ok = false;
    data_ok = false;

    for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
      if (emmc_send_cmd_raw(8u, 0u, 48u, r1, sizeof(r1))) {
        cmd8_ok = true;
        break;
      }
      emmc_send_retry_idle();
    }
    if (cmd8_ok) {
      // EXT_CSD path: use conservative software receiver for maximum robustness.
      data_ok = emmc_read_data_block_512_sw(out);
      if (data_ok) return true;
      if (msg && msg_len > 0u) snprintf(msg, msg_len, "EXT_CSD data timeout");
    } else if (msg && msg_len > 0u) {
      snprintf(msg, msg_len, "CMD8 (EXT_CSD) no response");
    }

    // Recovery path: re-select transfer state and retry.
    for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
      if (emmc_send_cmd_raw(7u, ((uint32_t)rca) << 16u, 48u, r1, sizeof(r1))) break;
      emmc_send_retry_idle();
    }
    if (!hc) {
      emmc_send_retry_idle();
      for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
        if (emmc_send_cmd_raw(16u, EMMC_DUMP_BLOCK_SIZE, 48u, r1, sizeof(r1))) break;
        emmc_send_retry_idle();
      }
    }
    emmc_send_idle_clocks(16u);
  }
  return false;
}

static bool send_layout_result(void) {
  uint8_t ext[EMMC_DUMP_BLOCK_SIZE];
  char msg[96];
  char out[1152];
  uint32_t sec_count;
  uint64_t user_bytes;
  uint32_t boot_size_mult;
  uint32_t boot_bytes_each;
  uint32_t part_cfg;
  uint32_t part_sup;
  uint32_t erase_grp;
  uint32_t wp_grp;
  uint64_t unit_bytes;
  uint32_t gp1_mul;
  uint32_t gp2_mul;
  uint32_t gp3_mul;
  uint32_t gp4_mul;
  uint64_t gp1_bytes;
  uint64_t gp2_bytes;
  uint64_t gp3_bytes;
  uint64_t gp4_bytes;
  bool ok = emmc_read_ext_csd(ext, msg, sizeof(msg));
  if (!ok) {
    snprintf(out, sizeof(out),
             "{\"type\":\"emmc.layout.result\",\"ok\":false,\"msg\":\"%s\"}",
             msg[0] ? msg : "EXT_CSD read failed");
    return app_send_text(out);
  }

  sec_count = rd_le32(&ext[212]);
  user_bytes = ((uint64_t)sec_count) * 512ull;
  boot_size_mult = ext[226];
  boot_bytes_each = boot_size_mult * (128u * 1024u);
  part_cfg = ext[179];
  part_sup = ext[160];
  erase_grp = ext[224];
  wp_grp = ext[221];
  unit_bytes = ((uint64_t)erase_grp) * ((uint64_t)wp_grp) * 512ull * 1024ull;
  gp1_mul = rd_le24(&ext[143]);
  gp2_mul = rd_le24(&ext[146]);
  gp3_mul = rd_le24(&ext[149]);
  gp4_mul = rd_le24(&ext[152]);
  gp1_bytes = ((uint64_t)gp1_mul) * unit_bytes;
  gp2_bytes = ((uint64_t)gp2_mul) * unit_bytes;
  gp3_bytes = ((uint64_t)gp3_mul) * unit_bytes;
  gp4_bytes = ((uint64_t)gp4_mul) * unit_bytes;

  snprintf(out, sizeof(out),
           "{\"type\":\"emmc.layout.result\",\"ok\":true,"
           "\"sec_count\":%lu,\"user_bytes\":%llu,"
           "\"boot_size_mult\":%lu,\"boot_bytes_each\":%lu,"
           "\"partition_config\":%lu,\"partitioning_support\":%lu,"
           "\"hc_erase_grp_size\":%lu,\"hc_wp_grp_size\":%lu,\"gp_unit_bytes\":%llu,"
           "\"gp1_mult\":%lu,\"gp2_mult\":%lu,\"gp3_mult\":%lu,\"gp4_mult\":%lu,"
           "\"gp1_bytes\":%llu,\"gp2_bytes\":%llu,\"gp3_bytes\":%llu,\"gp4_bytes\":%llu}",
           (unsigned long)sec_count, (unsigned long long)user_bytes,
           (unsigned long)boot_size_mult, (unsigned long)boot_bytes_each,
           (unsigned long)part_cfg, (unsigned long)part_sup,
           (unsigned long)erase_grp, (unsigned long)wp_grp, (unsigned long long)unit_bytes,
           (unsigned long)gp1_mul, (unsigned long)gp2_mul, (unsigned long)gp3_mul, (unsigned long)gp4_mul,
           (unsigned long long)gp1_bytes, (unsigned long long)gp2_bytes,
           (unsigned long long)gp3_bytes, (unsigned long long)gp4_bytes);
  return app_send_text(out);
}

static bool emmc_prepare_card_for_data(uint16_t *out_rca, bool *out_hc, char *msg, size_t msg_len) {
  emmc_id_data_t id;
  uint8_t r1[6];
  uint32_t i;
  bool selected = false;
  if (!out_rca || !out_hc) return false;
  if (!emmc_try_read_ids(&id)) {
    if (msg && msg_len > 0u) snprintf(msg, msg_len, "%s", id.msg[0] ? id.msg : "ID init failed");
    return false;
  }
  *out_rca = id.rca;
  *out_hc = ((id.ocr & 0x40000000u) != 0u);
  // Use push-pull CMD for selected/transfer commands.
  g_emmc.cmd_open_drain = false;
  for (i = 0; i < EMMC_CMD7_RETRIES; i++) {
    if (emmc_send_cmd_raw(7u, ((uint32_t)(*out_rca)) << 16u, 48u, r1, sizeof(r1))) {
      selected = true;
      break;
    }
    emmc_send_retry_idle();
    // Reassert RCA, then try select again.
    (void)emmc_send_cmd_raw(3u, ((uint32_t)(*out_rca)) << 16u, 48u, r1, sizeof(r1));
    emmc_send_idle_clocks(16u);
  }
  if (!selected) {
    // Some targets can still answer CMD17 in this state; continue and let CMD17 decide.
    if (msg && msg_len > 0u) snprintf(msg, msg_len, "CMD7 select failed, trying direct CMD17");
  }
  if (!(*out_hc)) {
    bool blklen_ok = false;
    for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
      if (emmc_send_cmd_raw(16u, EMMC_DUMP_BLOCK_SIZE, 48u, r1, sizeof(r1))) {
        blklen_ok = true;
        break;
      }
      emmc_send_retry_idle();
    }
    if (!blklen_ok) {
      if (msg && msg_len > 0u) snprintf(msg, msg_len, "CMD16 failed (set block len)");
      return false;
    }
  }
  return true;
}

static bool emmc_read_block(uint32_t lba, bool hc_addressing, uint8_t out[EMMC_DUMP_BLOCK_SIZE], char *msg, size_t msg_len) {
  uint8_t r1[6];
  uint16_t rca = g_emmc.dump_rca ? g_emmc.dump_rca : 1u;
  bool local_hc = hc_addressing;
  uint32_t arg = local_hc ? lba : (lba * EMMC_DUMP_BLOCK_SIZE);
  uint32_t attempt;
  uint32_t cmd_no_r1_count = 0u;
  uint32_t orig_speed = g_emmc.speed_hz;
  bool speed_floor_forced = false;
  uint32_t full_reprepare_count = 0u;
  uint32_t block_start_ms = now_ms();
  bool cmd_ok;
  bool data_ok;
  bool resync_ok;
  uint32_t cmd_try;
  emmc_set_data_err("");
  if (orig_speed < EMMC_DUMP_MIN_SPEED_HZ) {
    g_emmc.speed_hz = EMMC_DUMP_MIN_SPEED_HZ;
    emmc_update_timing();
    speed_floor_forced = true;
    if (app_debug_level() >= 1u) emmc_dbg(1, "cmd: forcing minimum dump speed 100k");
  }
  for (attempt = 0; attempt < EMMC_DUMP_READ_RETRIES; attempt++) {
    g_emmc.dump_stat_read_attempts++;
    if ((now_ms() - block_start_ms) > EMMC_READ_BLOCK_WATCHDOG_MS) {
      emmc_set_data_err("block read watchdog timeout");
      if (app_debug_level() >= 1u) emmc_dbg(1, "cmd: block read watchdog timeout");
      break;
    }
    arg = local_hc ? lba : (lba * EMMC_DUMP_BLOCK_SIZE);
    cmd_ok = false;
    for (cmd_try = 0; cmd_try < EMMC_CMD17_R1_EXTRA_TRIES; cmd_try++) {
      g_emmc.dump_stat_cmd17_tries++;
      if (emmc_send_cmd_raw(17u, arg, 48u, r1, sizeof(r1))) {
        cmd_ok = true;
        break;
      }
      emmc_send_retry_idle();
      tight_loop_contents();
    }
    data_ok = false;
    if (cmd_ok) data_ok = emmc_read_data_block_512(out);
    if (cmd_ok && data_ok) {
      g_emmc.dump_stat_read_success++;
      if (speed_floor_forced) {
        g_emmc.speed_hz = orig_speed;
        emmc_update_timing();
      }
      return true;
    }
    if (cmd_ok && !data_ok) g_emmc.dump_stat_data_fail++;

    if (!cmd_ok) {
      uint16_t new_rca;
      bool new_hc;
      char rec_msg[96];
      cmd_no_r1_count++;
      g_emmc.dump_stat_no_r1++;
      emmc_set_data_err("no R1 response");
      if (cmd_no_r1_count >= 2u) {
        if (full_reprepare_count < EMMC_FULL_REPREPARE_MAX) {
          full_reprepare_count++;
          emmc_bus_prepare();
          emmc_send_idle_clocks(32u);
          if (emmc_prepare_card_for_data(&new_rca, &new_hc, rec_msg, sizeof(rec_msg))) {
            g_emmc.dump_rca = new_rca;
            g_emmc.dump_hc_addressing = new_hc;
            rca = new_rca;
            local_hc = new_hc;
            cmd_no_r1_count = 0u;
            if (app_debug_level() >= 1u) emmc_dbg(1, "cmd: full re-prepare after repeated no-R1");
          } else if (app_debug_level() >= 1u) {
            emmc_dbg(1, "cmd: re-prepare failed after repeated no-R1");
          }
        } else if (app_debug_level() >= 1u) {
          emmc_dbg(1, "cmd: re-prepare limit reached for block");
        }
      }
    } else {
      cmd_no_r1_count = 0u;
    }

    emmc_send_retry_idle();
    // Re-sync transfer state and retry.
    resync_ok = emmc_resync_transfer(rca, local_hc);
    if (!resync_ok) {
      g_emmc.dump_stat_resync_fail++;
      emmc_set_data_err("resync failed (no R1 response)");
      if (app_debug_level() >= 2u) emmc_dbg(2, "cmd: resync failed (no R1 response)");
    }
  }
  if (speed_floor_forced) {
    g_emmc.speed_hz = orig_speed;
    emmc_update_timing();
  }
  if (msg && msg_len > 0u) {
    snprintf(msg, msg_len, "CMD17 failed at LBA %lu after %lu tries (%s)",
             (unsigned long)lba, (unsigned long)EMMC_DUMP_READ_RETRIES,
             g_emmc_data_err[0] ? g_emmc_data_err : "no R1 response");
  }
  return false;
}

static bool emmc_resync_transfer(uint16_t rca, bool hc_addressing) {
  uint8_t r1[6];
  uint32_t i;
  emmc_send_idle_clocks(16u);
  for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
    if (emmc_send_cmd_raw(7u, ((uint32_t)rca) << 16u, 48u, r1, sizeof(r1))) break;
    emmc_send_retry_idle();
  }
  if (i >= EMMC_CMDX_RETRIES) return false;
  if (!hc_addressing) {
    for (i = 0; i < EMMC_CMDX_RETRIES; i++) {
      if (emmc_send_cmd_raw(16u, EMMC_DUMP_BLOCK_SIZE, 48u, r1, sizeof(r1))) break;
      emmc_send_retry_idle();
    }
    if (i >= EMMC_CMDX_RETRIES) return false;
  }
      emmc_send_retry_idle();
  return true;
}

static uint16_t clamp_dump_chunk_bytes(uint32_t req) {
  if (req == 0u) req = EMMC_DUMP_CHUNK_BYTES;
  if (req > EMMC_MAX_DUMP_CHUNK_BYTES) req = EMMC_MAX_DUMP_CHUNK_BYTES;
  if (req < 16u) req = 16u;
  return (uint16_t)req;
}

static uint8_t clamp_dump_verify_retries(uint32_t req) {
  if (req > EMMC_DUMP_VERIFY_RETRIES_MAX) req = EMMC_DUMP_VERIFY_RETRIES_MAX;
  return (uint8_t)req;
}

static uint8_t clamp_dump_auto_retries(uint32_t req) {
  if (req > EMMC_DUMP_AUTO_RETRIES_MAX) req = EMMC_DUMP_AUTO_RETRIES_MAX;
  return (uint8_t)req;
}

static bool emmc_read_block_checked(uint32_t lba, bool hc_addressing,
                                    uint8_t out[EMMC_DUMP_BLOCK_SIZE], char *msg, size_t msg_len) {
  uint32_t verify_try;
  bool need_verify = g_emmc.dump_double_read;

  if (!need_verify) return emmc_read_block(lba, hc_addressing, out, msg, msg_len);

  for (verify_try = 0; verify_try <= g_emmc.dump_verify_retries; verify_try++) {
    uint32_t i;
    uint32_t mismatch_idx = 0xFFFFFFFFu;
    uint8_t exp_b = 0u;
    uint8_t got_b = 0u;

    if (!emmc_read_block(lba, hc_addressing, out, msg, msg_len)) continue;
    if (!emmc_read_block(lba, hc_addressing, g_emmc.dump_check_buf, msg, msg_len)) continue;
    if (memcmp(out, g_emmc.dump_check_buf, EMMC_DUMP_BLOCK_SIZE) == 0) return true;

    for (i = 0; i < EMMC_DUMP_BLOCK_SIZE; i++) {
      if (out[i] != g_emmc.dump_check_buf[i]) {
        mismatch_idx = i;
        exp_b = out[i];
        got_b = g_emmc.dump_check_buf[i];
        break;
      }
    }
    if (msg && msg_len > 0u) {
      snprintf(msg, msg_len, "verify mismatch LBA %lu idx %lu exp %02X got %02X (attempt %lu/%lu)",
               (unsigned long)lba,
               (unsigned long)(mismatch_idx == 0xFFFFFFFFu ? 0u : mismatch_idx),
               (unsigned)exp_b, (unsigned)got_b,
               (unsigned long)(verify_try + 1u),
               (unsigned long)(g_emmc.dump_verify_retries + 1u));
    }
  }
  if (msg && msg_len > 0u && msg[0] == 0) {
    snprintf(msg, msg_len, "verify failed at LBA %lu", (unsigned long)lba);
  }
  return false;
}

void proto_emmc_init(void) {
  uint32_t now = now_ms();
  g_emmc.last_detect_ms = now;
  g_emmc.last_idpoll_ms = now;
  emmc_update_timing();
  emmc_apply_safe_io();
  snapshot_levels();
}

void proto_emmc_on_client_open(ws_conn_t *conn) { (void)conn; }

void proto_emmc_on_client_close(ws_conn_t *conn) {
  (void)conn;
  g_emmc.detect_enabled = false;
  g_emmc.idpoll_enabled = false;
  g_emmc.dump_active = false;
}

bool proto_emmc_handle_text(const char *type, const char *json) {
  uint32_t v;
  bool b;

  if (!starts_with(type, "emmc.")) return false;

  if (strcmp(type, "emmc.get_config") == 0) {
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.set_io") == 0) {
    if (json_extract_bool(json, "tristate_default", &b)) g_emmc.tristate_default = b;
    if (json_extract_bool(json, "pullups_enabled", &b)) g_emmc.pullups_enabled = b;
    if (g_emmc.tristate_default) emmc_apply_safe_io();
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.set_speed") == 0) {
    if (json_extract_u32(json, "speed_hz", &v)) g_emmc.speed_hz = clamp_u32(v, EMMC_MIN_SPEED_HZ, EMMC_MAX_SPEED_HZ);
    if (json_extract_u32(json, "idpoll_interval_ms", &v)) {
      g_emmc.idpoll_interval_ms = clamp_u32(v, EMMC_MIN_IDPOLL_MS, EMMC_MAX_IDPOLL_MS);
    }
    if (json_extract_u32(json, "retry_idle_clks", &v)) g_emmc.retry_idle_clks = clamp_retry_idle_clks(v);
    emmc_update_timing();
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.detect.start") == 0) {
    g_emmc.detect_enabled = true;
    send_config();
    send_pins();
    return true;
  }
  if (strcmp(type, "emmc.detect.stop") == 0) {
    g_emmc.detect_enabled = false;
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.idpoll.start") == 0) {
    if (json_extract_u32(json, "interval_ms", &v)) {
      g_emmc.idpoll_interval_ms = clamp_u32(v, EMMC_MIN_IDPOLL_MS, EMMC_MAX_IDPOLL_MS);
    }
    if (json_extract_u32(json, "speed_hz", &v)) g_emmc.speed_hz = clamp_u32(v, EMMC_MIN_SPEED_HZ, EMMC_MAX_SPEED_HZ);
    if (json_extract_u32(json, "retry_idle_clks", &v)) g_emmc.retry_idle_clks = clamp_retry_idle_clks(v);
    if (json_extract_bool(json, "monitor_between", &b)) g_emmc.idpoll_monitor_between = b;
    emmc_update_timing();
    g_emmc.idpoll_enabled = true;
    g_emmc.poll_count = 0u;
    set_idpoll_baseline();
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.idpoll.stop") == 0) {
    g_emmc.idpoll_enabled = false;
    if (g_emmc.tristate_default) emmc_apply_safe_io();
    send_config();
    return true;
  }
  if (strcmp(type, "emmc.layout.read") == 0) {
    g_emmc.idpoll_enabled = false;
    g_emmc.detect_enabled = false;
    g_emmc.dump_active = false;
    (void)send_layout_result();
    if (g_emmc.tristate_default) emmc_apply_safe_io();
    return true;
  }
  if (strcmp(type, "emmc.dump.start") == 0) {
    char msg[96];
    uint16_t rca;
    bool hc;
    uint32_t blocks = 0u;
    uint32_t chunk_req = 0u;
    uint32_t verify_retries_req = EMMC_DUMP_VERIFY_RETRIES_DEFAULT;
    uint32_t auto_retries_req = EMMC_DUMP_AUTO_RETRIES_DEFAULT;
    g_emmc.idpoll_enabled = false;
    g_emmc.detect_enabled = false;
    g_emmc.dump_use_pio = true;
    if (json_extract_u32(json, "start_lba", &v)) g_emmc.dump_start_lba = v;
    if (json_extract_u32(json, "block_count", &v)) blocks = v;
    if (json_extract_u32(json, "chunk_bytes", &v)) chunk_req = v;
    if (json_extract_bool(json, "double_read", &b)) g_emmc.dump_double_read = b;
    if (json_extract_bool(json, "use_pio", &b)) g_emmc.dump_use_pio = b;
    if (json_extract_u32(json, "retry_idle_clks", &v)) g_emmc.retry_idle_clks = clamp_retry_idle_clks(v);
    if (json_extract_u32(json, "verify_retries", &v)) verify_retries_req = v;
    if (json_extract_u32(json, "auto_retries", &v)) auto_retries_req = v;
    if (blocks == 0u) blocks = 1u;
    g_emmc.dump_total_blocks = blocks;
    g_emmc.dump_done_blocks = 0u;
    g_emmc.dump_chunk_bytes = clamp_dump_chunk_bytes(chunk_req);
    g_emmc.dump_verify_retries = clamp_dump_verify_retries(verify_retries_req);
    g_emmc.dump_auto_retries = clamp_dump_auto_retries(auto_retries_req);
    g_emmc.dump_block_retry_count = 0u;
    g_emmc.dump_tx_fail_streak = 0u;
    g_emmc.dump_block_loaded = false;
    g_emmc.dump_chunk_off = 0u;
    g_emmc.dump_stat_read_attempts = 0u;
    g_emmc.dump_stat_read_success = 0u;
    g_emmc.dump_stat_cmd17_tries = 0u;
    g_emmc.dump_stat_no_r1 = 0u;
    g_emmc.dump_stat_data_fail = 0u;
    g_emmc.dump_stat_crc_fail = 0u;
    g_emmc.dump_stat_resync_fail = 0u;
    g_emmc.dump_stat_tx_busy = 0u;
    g_emmc.dump_stat_block_normal = 0u;
    g_emmc.dump_stat_block_all00 = 0u;
    g_emmc.dump_stat_block_allff = 0u;
    g_emmc.last_dump_ms = now_ms();
    g_emmc.dump_last_progress_ms = g_emmc.last_dump_ms;
    g_emmc.dump_last_status_ms = g_emmc.last_dump_ms;
    if (!emmc_prepare_card_for_data(&rca, &hc, msg, sizeof(msg))) {
      g_emmc.dump_active = false;
      (void)send_dump_status("error", msg);
      return true;
    }
    g_emmc.dump_rca = rca;
    g_emmc.dump_hc_addressing = hc;
    g_emmc.dump_active = true;
    (void)send_dump_status("running", "dump started");
    return true;
  }
  if (strcmp(type, "emmc.dump.stop") == 0) {
    g_emmc.dump_active = false;
    if (g_emmc.tristate_default) emmc_apply_safe_io();
    (void)send_dump_status("stopped", "dump stopped by user");
    return true;
  }
  if (strcmp(type, "emmc.cmd_test") == 0) {
    uint32_t cycles = 6u;
    if (json_extract_u32(json, "cycles", &v)) cycles = v;
    emmc_emit_cmd_test(cycles);
    return true;
  }
  if (strcmp(type, "emmc.cmd1_test") == 0) {
    uint32_t retries = 16u;
    uint32_t arg = 0x40FF8000u;
    if (json_extract_u32(json, "retries", &v)) retries = v;
    if (json_extract_u32(json, "arg", &v)) arg = v;
    emmc_emit_cmd1_test(retries, arg);
    return true;
  }
  if (strcmp(type, "emmc.sd_test") == 0) {
    uint32_t retries = 24u;
    uint32_t arg = 0x40FF8000u;
    if (json_extract_u32(json, "retries", &v)) retries = v;
    if (json_extract_u32(json, "arg", &v)) arg = v;
    emmc_emit_sd_test(retries, arg);
    return true;
  }

  return app_send_text("{\"type\":\"error\",\"code\":\"EMMC_UNSUPPORTED\",\"msg\":\"emmc command not implemented\"}");
}

void proto_emmc_poll(void) {
  uint32_t now = now_ms();
  emmc_id_data_t id;
  char dump_msg[96];
  uint32_t lba;
  poll_levels_and_transitions();

  if (g_emmc.detect_enabled && (now - g_emmc.last_detect_ms) >= 250u) {
    g_emmc.last_detect_ms = now;
    send_pins();
  }

  if (g_emmc.idpoll_enabled && (now - g_emmc.last_idpoll_ms) >= g_emmc.idpoll_interval_ms) {
    char out[640];
    char cid_hex[31];
    char csd_hex[31];
    char id_hex[24];
    uint8_t cid_mid;
    uint16_t cid_oid;
    g_emmc.last_idpoll_ms = now;
    g_emmc.poll_count++;

    if (g_emmc.idpoll_monitor_between) {
      send_idpoll_window();
      set_idpoll_baseline();
    }

    if (emmc_try_read_ids(&id)) {
      cid_mid = id.cid[0];
      cid_oid = (uint16_t)(((uint16_t)id.cid[1] << 8u) | id.cid[2]);
      snprintf(id_hex, sizeof(id_hex), "%02X %04X", (unsigned)cid_mid, (unsigned)cid_oid);
      hex_bytes(id.cid, sizeof(id.cid), cid_hex, sizeof(cid_hex));
      hex_bytes(id.csd, sizeof(id.csd), csd_hex, sizeof(csd_hex));

      snprintf(out, sizeof(out),
               "{\"type\":\"emmc.id.result\",\"ok\":true,\"poll_count\":%lu,"
               "\"id_hex\":\"%s\",\"mid\":%u,\"oid\":%u,\"ocr\":%lu,\"rca\":%u,"
               "\"cid\":\"%s\",\"csd\":\"%s\"}",
               (unsigned long)g_emmc.poll_count, id_hex, (unsigned)cid_mid, (unsigned)cid_oid,
               (unsigned long)id.ocr, (unsigned)id.rca, cid_hex, csd_hex);
    } else {
      snprintf(out, sizeof(out),
               "{\"type\":\"emmc.id.result\",\"ok\":false,\"poll_count\":%lu,"
               "\"msg\":\"%s\"}",
               (unsigned long)g_emmc.poll_count, id.msg[0] ? id.msg : "ID read failed");
    }
    (void)app_send_text(out);

    if (g_emmc.tristate_default) emmc_apply_safe_io();

    if (g_emmc.idpoll_monitor_between) {
      set_idpoll_baseline();
    }
  }

  if (g_emmc.dump_active) {
    uint32_t burst;
    uint32_t burst_limit = g_emmc.dump_use_pio ? 1u : EMMC_DUMP_BURST_CHUNKS;
    if ((now - g_emmc.last_dump_ms) < EMMC_DUMP_WS_INTERVAL_MS) return;
    g_emmc.last_dump_ms = now;
    if ((now - g_emmc.dump_last_status_ms) >= EMMC_DUMP_HEARTBEAT_MS) {
      (void)send_dump_status("running", "in progress");
      g_emmc.dump_last_status_ms = now;
    }
    if ((now - g_emmc.dump_last_progress_ms) >= EMMC_DUMP_STALL_TIMEOUT_MS) {
      char rec_msg[96];
      uint16_t rec_rca;
      bool rec_hc;
      if (emmc_prepare_card_for_data(&rec_rca, &rec_hc, rec_msg, sizeof(rec_msg))) {
        g_emmc.dump_rca = rec_rca;
        g_emmc.dump_hc_addressing = rec_hc;
        g_emmc.dump_last_progress_ms = now;
        (void)send_dump_status("running", "stall detected; link recovered");
        g_emmc.dump_last_status_ms = now;
      } else {
        g_emmc.dump_active = false;
        if (g_emmc.tristate_default) emmc_apply_safe_io();
        (void)send_dump_status("error", rec_msg[0] ? rec_msg : "stall detected; recovery failed");
      }
      return;
    }
    for (burst = 0; burst < burst_limit; burst++) {
      uint16_t remain;
      uint16_t n_send;
      if (g_emmc.dump_done_blocks >= g_emmc.dump_total_blocks) {
        g_emmc.dump_active = false;
        if (g_emmc.tristate_default) emmc_apply_safe_io();
        (void)send_dump_status("complete", "dump complete");
        return;
      }
      if (!g_emmc.dump_block_loaded) {
        char recover_msg[96];
        uint16_t rca;
        bool hc;
        lba = g_emmc.dump_start_lba + g_emmc.dump_done_blocks;
        if (g_emmc.dump_done_blocks > 0u && (g_emmc.dump_done_blocks % EMMC_RESELECT_EVERY_BLOCKS) == 0u) {
          emmc_resync_transfer(g_emmc.dump_rca ? g_emmc.dump_rca : 1u, g_emmc.dump_hc_addressing);
        }
        if (!emmc_read_block_checked(lba, g_emmc.dump_hc_addressing, g_emmc.dump_block_buf, dump_msg, sizeof(dump_msg))) {
          if (g_emmc.dump_block_retry_count < g_emmc.dump_auto_retries) {
            char retry_detail[160];
            g_emmc.dump_block_retry_count++;
            snprintf(retry_detail, sizeof(retry_detail),
                     "retrying LBA %lu (%u/%u): %s",
                     (unsigned long)lba,
                     (unsigned)g_emmc.dump_block_retry_count,
                     (unsigned)g_emmc.dump_auto_retries,
                     dump_msg);
            (void)send_dump_status("running", retry_detail);
            if (emmc_prepare_card_for_data(&rca, &hc, recover_msg, sizeof(recover_msg))) {
              g_emmc.dump_rca = rca;
              g_emmc.dump_hc_addressing = hc;
            }
            return;
          }
          g_emmc.dump_active = false;
          if (g_emmc.tristate_default) emmc_apply_safe_io();
          (void)send_dump_status("error", dump_msg);
          return;
        }
        g_emmc.dump_block_retry_count = 0u;
        g_emmc.dump_block_loaded = true;
        g_emmc.dump_chunk_off = 0u;
      }

      remain = (uint16_t)(EMMC_DUMP_BLOCK_SIZE - g_emmc.dump_chunk_off);
      n_send = g_emmc.dump_chunk_bytes;
      if (n_send > remain) n_send = remain;
      if (n_send > EMMC_DUMP_WS_DATA_BYTES) n_send = EMMC_DUMP_WS_DATA_BYTES;

      {
        uint32_t abs_off = (g_emmc.dump_done_blocks * EMMC_DUMP_BLOCK_SIZE) + g_emmc.dump_chunk_off;
        const uint8_t *p = &g_emmc.dump_block_buf[g_emmc.dump_chunk_off];
        uint16_t i;
        bool all00 = true;
        bool allff = true;
        bool sent_ok;

        for (i = 0; i < n_send; i++) {
          uint8_t b = p[i];
          if (b != 0x00u) all00 = false;
          if (b != 0xFFu) allff = false;
          if (!all00 && !allff) break;
        }

        if (all00 || allff) sent_ok = send_dump_run_bin(abs_off, n_send, allff ? 0xFFu : 0x00u);
        else sent_ok = send_dump_data_bin(abs_off, p, n_send);

        if (!sent_ok) {
          g_emmc.dump_tx_fail_streak++;
          g_emmc.dump_stat_tx_busy++;
          if ((g_emmc.dump_tx_fail_streak % 8u) == 0u) {
            (void)send_dump_status("running", "tx busy; retrying");
            g_emmc.dump_last_status_ms = now;
          }
          if (g_emmc.dump_tx_fail_streak > EMMC_DUMP_TX_FAIL_LIMIT) {
            g_emmc.dump_active = false;
            if (g_emmc.tristate_default) emmc_apply_safe_io();
            (void)send_dump_status("error", "stream stalled (tx busy)");
          }
          return;
        }
      }

      g_emmc.dump_tx_fail_streak = 0u;
      g_emmc.dump_last_progress_ms = now;
      g_emmc.dump_chunk_off = (uint16_t)(g_emmc.dump_chunk_off + n_send);
      if (g_emmc.dump_chunk_off >= EMMC_DUMP_BLOCK_SIZE) {
        bool blk_all00 = true;
        bool blk_allff = true;
        uint32_t bi;
        for (bi = 0; bi < EMMC_DUMP_BLOCK_SIZE; bi++) {
          uint8_t bb = g_emmc.dump_block_buf[bi];
          if (bb != 0x00u) blk_all00 = false;
          if (bb != 0xFFu) blk_allff = false;
          if (!blk_all00 && !blk_allff) break;
        }
        if (blk_all00) g_emmc.dump_stat_block_all00++;
        else if (blk_allff) g_emmc.dump_stat_block_allff++;
        else g_emmc.dump_stat_block_normal++;
        g_emmc.dump_block_loaded = false;
        g_emmc.dump_done_blocks++;
        emmc_send_idle_clocks(EMMC_DUMP_INTER_BLOCK_IDLE_CLKS);
        g_emmc.dump_last_progress_ms = now;
      }
    }
  }
}
