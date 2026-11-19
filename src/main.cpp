#include <Arduino.h>
#include "qr_code.h"
#include "reset_button_policy.h"
#include <DNSServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <WebServer.h>
#include <WiFi.h>
#include <time.h>
#include <driver/i2s.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/md5.h>

namespace {
constexpr char AP_SSID[] = "ELRS-Backpack";
constexpr char DEFAULT_AP_PASSWORD[] = "12345678";
String ap_password;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t CRSF_MAX_FRAME_LEN = 64;
constexpr uint8_t ESPNOW_MAX_PAYLOAD = 250;
constexpr uint8_t ESPNOW_QUEUE_DEPTH = 8;
constexpr uint8_t MAX_BIND_PROFILES = 8;
constexpr uint32_t DRONE_TELEMETRY_STALE_MS = 3000;
constexpr bool DEBUG_CRSF_FRAMES = false;
constexpr int RESET_BUTTON_PIN = 0;  // ESP32 DevKit BOOT, active low
constexpr uint32_t RESET_BUTTON_DEBOUNCE_MS = 50;

// Power input monitoring (use resistor divider for external battery -> ADC pin).
constexpr int BOARD_BATTERY_ADC_PIN = 34;
constexpr float BOARD_BATTERY_DIVIDER_RATIO = 2.0f;  // e.g. 100k/100k
constexpr float BOARD_BATTERY_MIN_V = 3.3f;
constexpr float BOARD_BATTERY_MAX_V = 4.2f;

// SD card (VSPI)
constexpr int SD_CS_PIN = 5;
constexpr int SD_SCK_PIN = 18;
constexpr int SD_MISO_PIN = 19;
constexpr int SD_MOSI_PIN = 23;

// Active buzzer pin (for tone alarms)
constexpr int BUZZER_PIN = 25;

// MAX98357A over I2S
constexpr int I2S_BCLK_PIN = 26;
constexpr int I2S_LRCLK_PIN = 27;
constexpr int I2S_DOUT_PIN = 22;

// Status LED (fallback to GPIO2 when board variant does not define LED_BUILTIN)
#ifdef LED_BUILTIN
constexpr int STATUS_LED_PIN = LED_BUILTIN;
#else
constexpr int STATUS_LED_PIN = 2;
#endif

struct TelemetryData {
  uint32_t gps_ms = 0, battery_ms = 0, link_ms = 0, attitude_ms = 0, mode_ms = 0;
  bool board_battery_valid = false;
  bool gps_fix = false;
  int gps_sats = 0;
  double gps_lat = 0.0;
  double gps_lon = 0.0;
  float gps_speed_kmh = 0.0f;
  float gps_heading_deg = 0.0f;
  int gps_altitude_m = 0;

  float battery_voltage = 0.0f;
  float battery_current = 0.0f;
  uint32_t battery_capacity_mah = 0;
  int battery_remaining_pct = 0;

  int link_rssi_1_dbm = 0;
  int link_rssi_2_dbm = 0;
  int link_quality = 0;
  int link_snr_db = 0;
  int link_active_antenna = 0;
  int link_rf_mode = 0;
  int link_tx_power = 0;
  int link_down_rssi_dbm = 0;
  int link_down_quality = 0;
  int link_down_snr_db = 0;

  float attitude_pitch_deg = 0.0f;
  float attitude_roll_deg = 0.0f;
  float attitude_yaw_deg = 0.0f;
  float vario_ms = 0.0f;
  String flight_mode = "N/A";
  bool is_armed = false;
  bool arm_known = false;

  float board_voltage = 0.0f;
  int board_percent = 0;

  uint32_t frames_total = 0;
  uint32_t crc_errors = 0;
  uint32_t unknown_frames = 0;
  uint32_t last_frame_ms = 0;
  uint32_t last_drone_telem_ms = 0;

  uint32_t espnow_packets = 0;
  uint32_t espnow_drop_short = 0;
  uint32_t espnow_drop_long = 0;
  uint32_t espnow_drop_queue = 0;
  uint32_t espnow_no_crsf = 0;
};

struct FrameStat {
  uint32_t count = 0;
  uint8_t last_payload_len = 0;
  String last_payload_hex = "";
};

struct BindProfile {
  String note;
  String bind_phrase;
  String uid_text;
};

struct LogConfig {
  bool enabled = true;
  uint8_t keep_files = 10;
  bool log_gps = true;      // required by user
  bool log_link = true;
  bool log_battery = true;
  bool log_attitude = false;
  bool log_unknown = false;
};

struct BuzzerConfig {
  bool enabled = true;

  bool alarm_uplink_rssi_enabled = true;
  int alarm_uplink_rssi_min_dbm = -95;
  String alarm_uplink_sound = "uplink_low.wav";

  bool alarm_downlink_rssi_enabled = true;
  int alarm_downlink_rssi_min_dbm = -95;
  String alarm_downlink_sound = "downlink_low.wav";

  bool alarm_lq_enabled = true;
  int alarm_lq_min = 70;
  String alarm_lq_sound = "lq_low.wav";

  bool alarm_telem_loss_enabled = true;
  int alarm_telem_loss_sec = 3;
  String alarm_telem_loss_sound = "telem_lost.wav";

  bool alarm_board_batt_enabled = false;
  int alarm_board_batt_min_pct = 30;
  String alarm_board_batt_sound = "board_batt_low.wav";

  bool alarm_drone_batt_enabled = false;
  int alarm_drone_batt_min_pct = 30;
  String alarm_drone_batt_sound = "drone_batt_low.wav";
};

struct AppConfig {
  String bind_phrase;
  String uid_text;
  uint8_t uid[6] = {106, 19, 19, 206, 193, 30};
  BindProfile profiles[MAX_BIND_PROFILES];
  LogConfig log;
  BuzzerConfig buzzer;
};

TelemetryData telemetry;
AppConfig config;
FrameStat frame_stats[256];

WebServer server(80);
DNSServer dns_server;
Preferences prefs;
SPIClass sd_spi(VSPI);

portMUX_TYPE espnow_queue_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t espnow_queue_head = 0;
volatile uint8_t espnow_queue_tail = 0;
volatile uint8_t espnow_queue_count = 0;
uint8_t espnow_queue_len[ESPNOW_QUEUE_DEPTH] = {0};
uint8_t espnow_queue[ESPNOW_QUEUE_DEPTH][ESPNOW_MAX_PAYLOAD] = {};
bool espnow_ready = false;
bool espnow_restart_pending = false;

bool sd_available = false;
String sd_status = "not initialized";
File active_log;
String active_log_name;
unsigned long last_log_flush_ms = 0;

bool last_alarm_uplink = false;
bool last_alarm_downlink = false;
bool last_alarm_lq = false;
bool last_alarm_telem = false;
bool last_alarm_board_batt = false;
bool last_alarm_drone_batt = false;
uint32_t alarm_event_id = 0;
String last_alarm_name;
String last_alarm_sound;
unsigned long last_alarm_ms = 0;
unsigned long last_led_blink_ms = 0;
bool led_blink_state = false;
reset_button::State reset_button_state;
bool reset_button_raw = false;
bool reset_button_stable = false;
uint32_t reset_button_changed_ms = 0;

struct AudioPlayer {
  bool i2s_ready = false;
  bool playing = false;
  File file;
  uint32_t data_remaining = 0;
  uint32_t sample_rate = 16000;
  String current_file;
} audio_player;

bool clock_synced = false;
time_t clock_epoch_base = 0;
unsigned long clock_millis_base = 0;
enum TimeSourcePriority { TS_NONE = 0, TS_WEB = 1, TS_REAL = 2 };
TimeSourcePriority clock_source_priority = TS_NONE;
String clock_source_name = "none";

String wifi_note = "home wifi disabled";

uint8_t crc8_dvb_s2(const uint8_t *ptr, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= ptr[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint16_t read_be16(const uint8_t *p) {
  return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

uint32_t read_be24(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 16) |
         (static_cast<uint32_t>(p[1]) << 8) | p[2];
}

uint32_t read_be32u(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

int32_t read_be32s(const uint8_t *p) {
  return (static_cast<int32_t>(p[0]) << 24) |
         (static_cast<int32_t>(p[1]) << 16) |
         (static_cast<int32_t>(p[2]) << 8) | p[3];
}

String json_escape(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    const char c = in[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (static_cast<unsigned char>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

String bytes_to_hex(const uint8_t *data, uint8_t len) {
  String out;
  out.reserve(len * 3);
  for (uint8_t i = 0; i < len; i++) {
    if (i) out += ' ';
    if (data[i] < 16) out += '0';
    out += String(data[i], HEX);
  }
  out.toUpperCase();
  return out;
}

String format_uptime(unsigned long ms) {
  const unsigned long total_s = ms / 1000UL;
  const unsigned long day = total_s / 86400UL;
  const unsigned long hh = (total_s % 86400UL) / 3600UL;
  const unsigned long mm = (total_s % 3600UL) / 60UL;
  const unsigned long ss = total_s % 60UL;
  char buf[40] = {0};
  snprintf(buf, sizeof(buf), "D+%lu %02lu:%02lu:%02lu.%03lu", day, hh, mm, ss, ms % 1000UL);
  return String(buf);
}

String current_timestamp_string() {
  if (!clock_synced) return "";

  const unsigned long now_ms = millis();
  const time_t now = clock_epoch_base + static_cast<time_t>((now_ms - clock_millis_base) / 1000UL);
  struct tm tmv = {};
#if defined(_MSC_VER)
  gmtime_s(&tmv, &now);
#else
  gmtime_r(&now, &tmv);
#endif
  char buf[32] = {0};
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
  return String(buf);
}

void set_clock_base(time_t epoch, TimeSourcePriority src, const char *name) {
  if (epoch < 946684800) return;  // 2000-01-01
  if (src < clock_source_priority) return;
  clock_epoch_base = epoch;
  clock_millis_base = millis();
  clock_synced = true;
  clock_source_priority = src;
  clock_source_name = String(name);
}

bool profile_is_empty(const BindProfile &p) {
  return p.note.isEmpty() && p.bind_phrase.isEmpty() && p.uid_text.isEmpty();
}

int find_profile_by_bind(const String &bind_phrase, const String &uid_text) {
  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    if (config.profiles[i].bind_phrase == bind_phrase && config.profiles[i].uid_text == uid_text &&
        !profile_is_empty(config.profiles[i])) {
      return i;
    }
  }
  return -1;
}

int first_empty_profile_slot() {
  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    if (profile_is_empty(config.profiles[i])) return i;
  }
  return -1;
}

void upsert_current_profile(const String &note) {
  int idx = find_profile_by_bind(config.bind_phrase, config.uid_text);
  if (idx < 0) idx = first_empty_profile_slot();
  if (idx < 0) idx = 0;

  if (!note.isEmpty()) {
    config.profiles[idx].note = note;
  } else if (config.profiles[idx].note.isEmpty()) {
    config.profiles[idx].note = "Profile " + String(idx);
  }
  config.profiles[idx].bind_phrase = config.bind_phrase;
  config.profiles[idx].uid_text = config.uid_text;
}

void i2s_init_once() {
  if (audio_player.i2s_ready) return;

  const i2s_config_t cfg = {
      .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 6,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0,
  };
  const i2s_pin_config_t pins = {
      .bck_io_num = I2S_BCLK_PIN,
      .ws_io_num = I2S_LRCLK_PIN,
      .data_out_num = I2S_DOUT_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE,
  };

  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  audio_player.i2s_ready = true;
}

uint32_t le_u32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t le_u16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

bool start_wav_playback(const String &file_name) {
  if (!sd_available || file_name.isEmpty()) return false;
  i2s_init_once();

  String path = "/sound/" + file_name;
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) return false;

  uint8_t hdr[12];
  if (f.read(hdr, sizeof(hdr)) != static_cast<int>(sizeof(hdr))) {
    f.close();
    return false;
  }
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(&hdr[8], "WAVE", 4) != 0) {
    f.close();
    return false;
  }

  uint16_t audio_fmt = 0;
  uint16_t channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits = 0;
  uint32_t data_size = 0;
  size_t data_pos = 0;

  while (f.available() >= 8) {
    uint8_t ch[8];
    if (f.read(ch, 8) != 8) break;
    const uint32_t chunk_size = le_u32(&ch[4]);

    if (memcmp(ch, "fmt ", 4) == 0) {
      uint8_t fmt[16] = {0};
      const size_t take = min(static_cast<uint32_t>(16), chunk_size);
      if (f.read(fmt, take) != static_cast<int>(take)) break;
      if (chunk_size > take) f.seek(f.position() + (chunk_size - take));
      audio_fmt = le_u16(&fmt[0]);
      channels = le_u16(&fmt[2]);
      sample_rate = le_u32(&fmt[4]);
      bits = le_u16(&fmt[14]);
    } else if (memcmp(ch, "data", 4) == 0) {
      data_size = chunk_size;
      data_pos = f.position();
      break;
    } else {
      f.seek(f.position() + chunk_size);
    }
  }

  if (audio_fmt != 1 || channels != 1 || bits != 16 || data_size == 0 || sample_rate < 8000 ||
      sample_rate > 48000) {
    f.close();
    return false;
  }

  f.seek(data_pos);
  i2s_set_clk(I2S_NUM_0, sample_rate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  if (audio_player.playing && audio_player.file) {
    audio_player.file.close();
  }
  audio_player.file = f;
  audio_player.data_remaining = data_size;
  audio_player.sample_rate = sample_rate;
  audio_player.playing = true;
  audio_player.current_file = file_name;
  return true;
}

void audio_poll() {
  if (!audio_player.playing || !audio_player.file) return;
  static uint8_t buf[512];
  const size_t to_read = min(static_cast<uint32_t>(sizeof(buf)), audio_player.data_remaining);
  if (to_read == 0) {
    audio_player.file.close();
    audio_player.playing = false;
    return;
  }

  const int n = audio_player.file.read(buf, to_read);
  if (n <= 0) {
    audio_player.file.close();
    audio_player.playing = false;
    return;
  }

  size_t written = 0;
  const TickType_t timeout_ticks = pdMS_TO_TICKS(20);
  esp_err_t err = i2s_write(I2S_NUM_0, buf, n, &written, timeout_ticks);
  if (err != ESP_OK || written != static_cast<size_t>(n)) {
    audio_player.file.close();
    audio_player.playing = false;
    audio_player.data_remaining = 0;
    return;
  }
  audio_player.data_remaining -= static_cast<uint32_t>(written);
  if (audio_player.data_remaining == 0) {
    audio_player.file.close();
    audio_player.playing = false;
  }
}

bool parse_uid_text(const String &uid_text, uint8_t out_uid[6]) {
  String t = uid_text;
  t.trim();
  if (t.isEmpty()) {
    return false;
  }

  t.replace(",", " ");
  t.replace(";", " ");
  t.replace(":", " ");

  int v[6] = {0};
  if (sscanf(t.c_str(), "%d %d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (v[i] < 0 || v[i] > 255) {
      return false;
    }
    out_uid[i] = static_cast<uint8_t>(v[i]);
  }
  out_uid[0] &= static_cast<uint8_t>(~0x01);
  return true;
}

void uid_from_bind_phrase(const String &phrase, uint8_t out_uid[6]) {
  uint8_t digest[16] = {0};
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts_ret(&ctx);
  mbedtls_md5_update_ret(&ctx, reinterpret_cast<const unsigned char *>(phrase.c_str()), phrase.length());
  mbedtls_md5_finish_ret(&ctx, digest);
  mbedtls_md5_free(&ctx);

  memcpy(out_uid, digest, 6);
  out_uid[0] &= static_cast<uint8_t>(~0x01);
}

String uid_to_text(const uint8_t uid[6]) {
  char buf[48] = {0};
  snprintf(buf, sizeof(buf), "%u,%u,%u,%u,%u,%u", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
  return String(buf);
}

void recalc_effective_uid() {
  uint8_t uid[6] = {0};

  if (parse_uid_text(config.uid_text, uid)) {
    memcpy(config.uid, uid, sizeof(config.uid));
    return;
  }

  if (!config.bind_phrase.isEmpty()) {
    uid_from_bind_phrase(config.bind_phrase, uid);
    memcpy(config.uid, uid, sizeof(config.uid));
    return;
  }

  uint8_t fallback[6] = {106, 19, 19, 206, 193, 30};
  memcpy(config.uid, fallback, sizeof(config.uid));
}

void load_config() {
  prefs.begin("cfg", true);

  config.bind_phrase = prefs.getString("bind_phrase", "");
  config.uid_text = prefs.getString("uid_text", "");

  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    const String p = "p" + String(i) + "_";
    config.profiles[i].note = prefs.getString((p + "note").c_str(), "");
    config.profiles[i].bind_phrase = prefs.getString((p + "phrase").c_str(), "");
    config.profiles[i].uid_text = prefs.getString((p + "uid").c_str(), "");
  }

  config.log.enabled = prefs.getBool("log_en", true);
  config.log.keep_files = prefs.getUChar("log_keep", 10);
  config.log.log_gps = prefs.getBool("log_gps", true);
  config.log.log_link = prefs.getBool("log_link", true);
  config.log.log_battery = prefs.getBool("log_bat", true);
  config.log.log_attitude = prefs.getBool("log_att", false);
  config.log.log_unknown = prefs.getBool("log_unk", false);

  config.buzzer.enabled = prefs.getBool("bz_en", true);
  config.buzzer.alarm_uplink_rssi_enabled = prefs.getBool("bz_u_en", true);
  config.buzzer.alarm_uplink_rssi_min_dbm = prefs.getInt("bz_u_min", -95);
  config.buzzer.alarm_uplink_sound = prefs.getString("bz_u_snd", "uplink_low.wav");

  config.buzzer.alarm_downlink_rssi_enabled = prefs.getBool("bz_d_en", true);
  config.buzzer.alarm_downlink_rssi_min_dbm = prefs.getInt("bz_d_min", -95);
  config.buzzer.alarm_downlink_sound = prefs.getString("bz_d_snd", "downlink_low.wav");

  config.buzzer.alarm_lq_enabled = prefs.getBool("bz_lq_en", true);
  config.buzzer.alarm_lq_min = prefs.getInt("bz_lq_min", 70);
  config.buzzer.alarm_lq_sound = prefs.getString("bz_lq_snd", "lq_low.wav");

  config.buzzer.alarm_telem_loss_enabled = prefs.getBool("bz_t_en", true);
  config.buzzer.alarm_telem_loss_sec = prefs.getInt("bz_t_sec", 3);
  config.buzzer.alarm_telem_loss_sound = prefs.getString("bz_t_snd", "telem_lost.wav");
  config.buzzer.alarm_board_batt_enabled = prefs.getBool("bz_bb_en", false);
  config.buzzer.alarm_board_batt_min_pct = prefs.getInt("bz_bb_min", 30);
  config.buzzer.alarm_board_batt_sound = prefs.getString("bz_bb_snd", "board_batt_low.wav");
  config.buzzer.alarm_drone_batt_enabled = prefs.getBool("bz_db_en", false);
  config.buzzer.alarm_drone_batt_min_pct = prefs.getInt("bz_db_min", 30);
  config.buzzer.alarm_drone_batt_sound = prefs.getString("bz_db_snd", "drone_batt_low.wav");

  prefs.end();

  if (config.log.keep_files < 1) config.log.keep_files = 1;
  if (config.log.keep_files > 100) config.log.keep_files = 100;
  recalc_effective_uid();
}

void save_config() {
  prefs.begin("cfg", false);

  prefs.putString("bind_phrase", config.bind_phrase);
  prefs.putString("uid_text", config.uid_text);

  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    const String p = "p" + String(i) + "_";
    prefs.putString((p + "note").c_str(), config.profiles[i].note);
    prefs.putString((p + "phrase").c_str(), config.profiles[i].bind_phrase);
    prefs.putString((p + "uid").c_str(), config.profiles[i].uid_text);
  }

  prefs.putBool("log_en", config.log.enabled);
  prefs.putUChar("log_keep", config.log.keep_files);
  prefs.putBool("log_gps", config.log.log_gps);
  prefs.putBool("log_link", config.log.log_link);
  prefs.putBool("log_bat", config.log.log_battery);
  prefs.putBool("log_att", config.log.log_attitude);
  prefs.putBool("log_unk", config.log.log_unknown);

  prefs.putBool("bz_en", config.buzzer.enabled);
  prefs.putBool("bz_u_en", config.buzzer.alarm_uplink_rssi_enabled);
  prefs.putInt("bz_u_min", config.buzzer.alarm_uplink_rssi_min_dbm);
  prefs.putString("bz_u_snd", config.buzzer.alarm_uplink_sound);

  prefs.putBool("bz_d_en", config.buzzer.alarm_downlink_rssi_enabled);
  prefs.putInt("bz_d_min", config.buzzer.alarm_downlink_rssi_min_dbm);
  prefs.putString("bz_d_snd", config.buzzer.alarm_downlink_sound);

  prefs.putBool("bz_lq_en", config.buzzer.alarm_lq_enabled);
  prefs.putInt("bz_lq_min", config.buzzer.alarm_lq_min);
  prefs.putString("bz_lq_snd", config.buzzer.alarm_lq_sound);

  prefs.putBool("bz_t_en", config.buzzer.alarm_telem_loss_enabled);
  prefs.putInt("bz_t_sec", config.buzzer.alarm_telem_loss_sec);
  prefs.putString("bz_t_snd", config.buzzer.alarm_telem_loss_sound);
  prefs.putBool("bz_bb_en", config.buzzer.alarm_board_batt_enabled);
  prefs.putInt("bz_bb_min", config.buzzer.alarm_board_batt_min_pct);
  prefs.putString("bz_bb_snd", config.buzzer.alarm_board_batt_sound);
  prefs.putBool("bz_db_en", config.buzzer.alarm_drone_batt_enabled);
  prefs.putInt("bz_db_min", config.buzzer.alarm_drone_batt_min_pct);
  prefs.putString("bz_db_snd", config.buzzer.alarm_drone_batt_sound);

  prefs.end();
}

bool is_known_frame_type(uint8_t t) {
  return t == 0x02 || t == 0x08 || t == 0x14 || t == 0x1E || t == 0x21 || t == 0x07;
}

bool should_log_type(uint8_t type) {
  if (type == 0x02) return config.log.log_gps;
  if (type == 0x14) return config.log.log_link;
  if (type == 0x08) return config.log.log_battery;
  if (type == 0x1E) return config.log.log_attitude;
  if (type == 0x21) return true;
  if (type == 0x07) return config.log.log_attitude;
  return config.log.log_unknown;
}

void serial_log_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
  if (!DEBUG_CRSF_FRAMES) return;
  Serial.printf("[CRSF] type=0x%02X payload=%u\n", type, payload_len);
  Serial.printf("       data=%s\n", bytes_to_hex(payload, payload_len).c_str());
}

bool fresh(uint32_t last, uint32_t max_age = DRONE_TELEMETRY_STALE_MS) {
  return last != 0 && static_cast<uint32_t>(millis() - last) <= max_age;
}

void update_armed_state() {
  String m = telemetry.flight_mode;
  m.toUpperCase();
  telemetry.arm_known = false;
  // Betaflight suffixes encode disarmed; !FS! is ambiguous.
  if (m == "!FS!" || m.isEmpty()) return;
  const char suffix = m[m.length()-1];
  String base = m;
  if (suffix == '*' || suffix == '!' || suffix == '?') base.remove(base.length()-1);
  const char *modes[] = {"ACRO", "AIR", "ANGL", "HOR", "RTH", "PASS", "POSH", "ALTH", "CHIR", "PHFL", "STAB"};
  for (const char *known : modes) {
    if (base == known) {
      telemetry.arm_known = true;
      telemetry.is_armed = suffix != '*' && suffix != '!' && suffix != '?';
      return;
    }
  }
}

void ensure_sd_card() {
  if (sd_available) {
    return;
  }

  sd_spi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, sd_spi)) {
    sd_available = false;
    sd_status = "not available";
    return;
  }
  sd_available = true;
  sd_status = "available";

  if (!SD.exists("/logs")) {
    SD.mkdir("/logs");
  }
  if (!SD.exists("/sound")) {
    SD.mkdir("/sound");
  }
}

void close_log_file() {
  if (active_log) {
    active_log.flush();
    active_log.close();
  }
  active_log_name = "";
}

void rotate_logs_if_needed() {
  if (!sd_available) return;

  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) return;

  String names[128];
  int count = 0;

  File f = dir.openNextFile();
  while (f && count < 128) {
    if (!f.isDirectory()) {
      names[count++] = String(f.name());
    }
    f = dir.openNextFile();
  }
  dir.close();

  if (count <= config.log.keep_files) return;

  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (names[j] < names[i]) {
        String tmp = names[i];
        names[i] = names[j];
        names[j] = tmp;
      }
    }
  }

  const int to_delete = count - config.log.keep_files;
  for (int i = 0; i < to_delete; i++) {
    String path = names[i];
    if (!path.startsWith("/")) path = "/" + path;
    SD.remove(path);
  }
}

String build_new_log_name() {
  prefs.begin("cfg", false);
  uint32_t seq = prefs.getULong("log_seq", 0);
  seq++;
  prefs.putULong("log_seq", seq);
  prefs.end();

  char name[64] = {0};
  snprintf(name, sizeof(name), "/logs/log_%06lu.csv", static_cast<unsigned long>(seq));
  return String(name);
}

void open_log_if_needed() {
  if (!config.log.enabled || !telemetry.arm_known || !fresh(telemetry.mode_ms, 10000) || !telemetry.is_armed || !sd_available || active_log) {
    return;
  }

  active_log_name = build_new_log_name();
  active_log = SD.open(active_log_name.c_str(), FILE_WRITE);
  if (!active_log) {
    sd_status = "log open failed";
    active_log_name = "";
    return;
  }

  active_log.println("ms,timestamp,uptime,type,payload_hex,lat,lon,gps_fix,gps_sats,bat_v,bat_a,lq,rssi1,rssi2,down_rssi,down_lq,down_snr,mode");
  active_log.flush();
  rotate_logs_if_needed();
  sd_status = "logging";
}

void log_frame_csv(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
  if (!active_log || !should_log_type(type)) return;

  active_log.print(millis());
  active_log.print(',');
  const String ts = current_timestamp_string();
  active_log.print(ts.isEmpty() ? "-" : ts);
  active_log.print(',');
  active_log.print(format_uptime(millis()));
  active_log.print(',');
  active_log.print(type);
  active_log.print(',');
  active_log.print('"');
  active_log.print(bytes_to_hex(payload, payload_len));
  active_log.print('"');
  active_log.print(',');
  active_log.print(telemetry.gps_lat, 7);
  active_log.print(',');
  active_log.print(telemetry.gps_lon, 7);
  active_log.print(',');
  active_log.print(telemetry.gps_fix ? 1 : 0);
  active_log.print(',');
  active_log.print(telemetry.gps_sats);
  active_log.print(',');
  active_log.print(telemetry.battery_voltage, 2);
  active_log.print(',');
  active_log.print(telemetry.battery_current, 2);
  active_log.print(',');
  active_log.print(telemetry.link_quality);
  active_log.print(',');
  active_log.print(telemetry.link_rssi_1_dbm);
  active_log.print(',');
  active_log.print(telemetry.link_rssi_2_dbm);
  active_log.print(',');
  active_log.print(telemetry.link_down_rssi_dbm);
  active_log.print(',');
  active_log.print(telemetry.link_down_quality);
  active_log.print(',');
  active_log.print(telemetry.link_down_snr_db);
  active_log.print(',');
  String mode = telemetry.flight_mode;
  mode.replace("\"", "\"\"");
  active_log.print('"');
  active_log.print(mode);
  active_log.println('"');

  if (millis() - last_log_flush_ms > 1000) {
    active_log.flush();
    last_log_flush_ms = millis();
  }
}

void update_logging_state() {
  if (!telemetry.arm_known || !fresh(telemetry.mode_ms, 10000) || !telemetry.is_armed || !config.log.enabled || !sd_available) {
    if (active_log) {
      close_log_file();
      sd_status = sd_available ? "available" : "not available";
    }
    return;
  }
  open_log_if_needed();
}

void trigger_alarm(const String &name, const String &sound_file) {
  alarm_event_id++;
  last_alarm_name = name;
  last_alarm_sound = sound_file;
  last_alarm_ms = millis();

  if (config.buzzer.enabled) {
    tone(BUZZER_PIN, 2400, 180);
  }
  start_wav_playback(sound_file);
}

void check_alarms() {
  const bool uplink_bad = fresh(telemetry.link_ms, 10000) && config.buzzer.alarm_uplink_rssi_enabled &&
                          telemetry.link_rssi_1_dbm < config.buzzer.alarm_uplink_rssi_min_dbm;
  const bool downlink_bad = fresh(telemetry.link_ms, 10000) && config.buzzer.alarm_downlink_rssi_enabled &&
                            telemetry.link_down_rssi_dbm < config.buzzer.alarm_downlink_rssi_min_dbm;
  const bool lq_bad = fresh(telemetry.link_ms, 10000) && config.buzzer.alarm_lq_enabled &&
                      telemetry.link_quality < config.buzzer.alarm_lq_min;
  const bool telem_bad = config.buzzer.alarm_telem_loss_enabled &&
                         telemetry.last_drone_telem_ms > 0 &&
                         (millis() - telemetry.last_drone_telem_ms) >
                             static_cast<unsigned long>(config.buzzer.alarm_telem_loss_sec) * 1000UL;
  const bool board_batt_bad = config.buzzer.alarm_board_batt_enabled &&
                              telemetry.board_battery_valid &&
                              telemetry.board_percent < config.buzzer.alarm_board_batt_min_pct;
  const bool drone_batt_bad = config.buzzer.alarm_drone_batt_enabled &&
                              fresh(telemetry.battery_ms, 10000) &&
                              telemetry.battery_remaining_pct < config.buzzer.alarm_drone_batt_min_pct;

  if (uplink_bad && !last_alarm_uplink) {
    trigger_alarm("uplink_rssi_low", config.buzzer.alarm_uplink_sound);
  }
  if (downlink_bad && !last_alarm_downlink) {
    trigger_alarm("downlink_rssi_low", config.buzzer.alarm_downlink_sound);
  }
  if (lq_bad && !last_alarm_lq) {
    trigger_alarm("lq_low", config.buzzer.alarm_lq_sound);
  }
  if (telem_bad && !last_alarm_telem) {
    trigger_alarm("telemetry_lost", config.buzzer.alarm_telem_loss_sound);
  }
  if (board_batt_bad && !last_alarm_board_batt) {
    trigger_alarm("board_battery_low", config.buzzer.alarm_board_batt_sound);
  }
  if (drone_batt_bad && !last_alarm_drone_batt) {
    trigger_alarm("drone_battery_low", config.buzzer.alarm_drone_batt_sound);
  }

  last_alarm_uplink = uplink_bad;
  last_alarm_downlink = downlink_bad;
  last_alarm_lq = lq_bad;
  last_alarm_telem = telem_bad;
  last_alarm_board_batt = board_batt_bad;
  last_alarm_drone_batt = drone_batt_bad;
}

void update_status_led() {
  if (reset_button_state.phase == reset_button::Phase::Ready) {
    digitalWrite(STATUS_LED_PIN, (millis() / 150U) % 2U ? HIGH : LOW);
    return;
  }
  const bool connected = (millis() - telemetry.last_frame_ms) < 2000UL && telemetry.frames_total > 0;
  if (connected) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }

  if (millis() - last_led_blink_ms > 600UL) {
    last_led_blink_ms = millis();
    led_blink_state = !led_blink_state;
    digitalWrite(STATUS_LED_PIN, led_blink_state ? HIGH : LOW);
  }
}

void update_reset_button() {
  const uint32_t now = millis();
  const bool pressed = digitalRead(RESET_BUTTON_PIN) == LOW;
  if (pressed != reset_button_raw) {
    reset_button_raw = pressed;
    reset_button_changed_ms = now;
  }
  if (reset_button_stable != reset_button_raw &&
      static_cast<uint32_t>(now - reset_button_changed_ms) >= RESET_BUTTON_DEBOUNCE_MS) {
    reset_button_stable = reset_button_raw;
  }
  const reset_button::Phase previous = reset_button_state.phase;
  reset_button_state = reset_button::step(reset_button_state, reset_button_stable, now);
  if (previous != reset_button::Phase::Ready && reset_button_state.phase == reset_button::Phase::Ready) {
    Serial.println("Wi-Fi reset ready: release BOOT to reset password and restart");
  }
  if (reset_button_state.phase != reset_button::Phase::Fire) return;

  // Preserve all other NVS namespaces and settings. Only replace the AP password.
  bool saved = false;
  if (prefs.begin("backpack-ap", false)) {
    saved = prefs.putString("password", DEFAULT_AP_PASSWORD) > 0 &&
            prefs.getString("password", "") == DEFAULT_AP_PASSWORD;
    prefs.end();
  }
  if (!saved) {
    Serial.println("Wi-Fi password reset failed: storage error");
    reset_button_state = reset_button::State(reset_button::Phase::Idle);
    return;
  }
  Serial.println("Wi-Fi password reset; restarting");
  if (active_log) close_log_file();
  ESP.restart();
}

void parse_crsf_frame(const uint8_t *frame, uint8_t len) {
  if (len < 4) {
    return;
  }

  const uint8_t crsf_len = frame[1];
  if ((crsf_len + 2) != len) {
    telemetry.crc_errors++;
    return;
  }

  const uint8_t crc_in = frame[len - 1];
  const uint8_t crc_calc = crc8_dvb_s2(&frame[2], len - 3);
  if (crc_in != crc_calc) {
    telemetry.crc_errors++;
    return;
  }

  const uint8_t type = frame[2];
  const uint8_t payload_len = static_cast<uint8_t>(len - 4);
  const uint8_t *payload = &frame[3];
  const uint32_t now_ms = millis();

  telemetry.frames_total++;
  telemetry.last_frame_ms = now_ms;
  frame_stats[type].count++;
  frame_stats[type].last_payload_len = payload_len;
  frame_stats[type].last_payload_hex = bytes_to_hex(payload, payload_len);

  serial_log_frame(type, payload, payload_len);

  switch (type) {
    case 0x02: {
      if (payload_len < 15) break;
      const int32_t lat = read_be32s(payload + 0);
      const int32_t lon = read_be32s(payload + 4);
      const uint16_t speed = read_be16(payload + 8);
      const uint16_t heading = read_be16(payload + 10);
      const uint16_t altitude = read_be16(payload + 12);
      const uint8_t sats = payload[14];

      telemetry.gps_lat = static_cast<double>(lat) / 10000000.0;
      telemetry.gps_lon = static_cast<double>(lon) / 10000000.0;
      telemetry.gps_speed_kmh = static_cast<float>(speed) / 10.0f;
      telemetry.gps_heading_deg = static_cast<float>(heading) / 100.0f;
      telemetry.gps_altitude_m = static_cast<int>(altitude) - 1000;
      telemetry.gps_sats = sats;
      telemetry.gps_fix = (sats > 0);
      telemetry.gps_ms = now_ms;
      telemetry.last_drone_telem_ms = now_ms;
      break;
    }
    case 0x08: {
      if (payload_len < 8) break;
      const uint16_t v = read_be16(payload + 0);
      const uint16_t c = read_be16(payload + 2);
      const uint32_t mah = read_be24(payload + 4);
      const uint8_t rem = payload[7];

      telemetry.battery_voltage = static_cast<float>(v) / 10.0f;
      telemetry.battery_current = static_cast<float>(c) / 10.0f;
      telemetry.battery_capacity_mah = mah;
      telemetry.battery_remaining_pct = rem;
      telemetry.battery_ms = now_ms;
      telemetry.last_drone_telem_ms = now_ms;
      break;
    }
    case 0x14: {
      if (payload_len < 10) break;
      telemetry.link_rssi_1_dbm = static_cast<int8_t>(payload[0]);
      telemetry.link_rssi_2_dbm = static_cast<int8_t>(payload[1]);
      telemetry.link_quality = payload[2];
      telemetry.link_snr_db = static_cast<int8_t>(payload[3]);
      telemetry.link_active_antenna = payload[4];
      telemetry.link_rf_mode = payload[5];
      telemetry.link_tx_power = payload[6];
      telemetry.link_down_rssi_dbm = static_cast<int8_t>(payload[7]);
      telemetry.link_down_quality = payload[8];
      telemetry.link_down_snr_db = static_cast<int8_t>(payload[9]);
      telemetry.link_ms = now_ms;
      break;
    }
    case 0x1E: {
      if (payload_len < 6) break;
      constexpr float kRadToDeg = 57.2957795f;
      const int16_t pitch = static_cast<int16_t>(read_be16(payload + 0));
      const int16_t roll = static_cast<int16_t>(read_be16(payload + 2));
      const int16_t yaw = static_cast<int16_t>(read_be16(payload + 4));
      telemetry.attitude_pitch_deg = (static_cast<float>(pitch) / 10000.0f) * kRadToDeg;
      telemetry.attitude_roll_deg = (static_cast<float>(roll) / 10000.0f) * kRadToDeg;
      telemetry.attitude_yaw_deg = (static_cast<float>(yaw) / 10000.0f) * kRadToDeg;
      telemetry.attitude_ms = now_ms;
      telemetry.last_drone_telem_ms = now_ms;
      break;
    }
    case 0x21: {
      if (payload_len == 0) break;
      char buf[20] = {0};
      const size_t copy_len = min(static_cast<size_t>(payload_len), sizeof(buf) - 1);
      memcpy(buf, payload, copy_len);
      telemetry.flight_mode = String(buf);
      telemetry.mode_ms = now_ms;
      update_armed_state();
      telemetry.last_drone_telem_ms = now_ms;
      break;
    }
    case 0x07: {
      if (payload_len < 2) break;
      const int16_t vario_cms = static_cast<int16_t>(read_be16(payload));
      telemetry.vario_ms = static_cast<float>(vario_cms) / 100.0f;
      telemetry.last_drone_telem_ms = now_ms;
      break;
    }
    case 0x16: {  // Time source frame (if provided by telemetry source)
      if (payload_len >= 4) {
        const uint32_t epoch_candidate = read_be32u(payload);
        if (epoch_candidate > 1577836800UL && epoch_candidate < 4102444800UL) {  // 2020..2100
          set_clock_base(static_cast<time_t>(epoch_candidate), TS_REAL, "telemetry");
        }
      }
      break;
    }
    default:
      telemetry.unknown_frames++;
      break;
  }

  log_frame_csv(type, payload, payload_len);
}

bool is_valid_crsf_frame(const uint8_t *frame, uint8_t len) {
  if (len < 4 || len > CRSF_MAX_FRAME_LEN) {
    return false;
  }

  const uint8_t sync = frame[0];
  if (sync != 0xC8 && sync != 0xEE) {
    return false;
  }

  const uint8_t crsf_len = frame[1];
  if ((crsf_len + 2) != len) {
    return false;
  }

  const uint8_t crc_in = frame[len - 1];
  const uint8_t crc_calc = crc8_dvb_s2(&frame[2], len - 3);
  return crc_in == crc_calc;
}

void on_espnow_recv(const uint8_t *mac, const uint8_t *incoming_data, int len) {
  (void)mac;

  if (len <= 0) {
    portENTER_CRITICAL(&espnow_queue_mux);
    telemetry.espnow_packets++;
    telemetry.espnow_drop_short++;
    portEXIT_CRITICAL(&espnow_queue_mux);
    return;
  }

  if (len > ESPNOW_MAX_PAYLOAD) {
    portENTER_CRITICAL(&espnow_queue_mux);
    telemetry.espnow_packets++;
    telemetry.espnow_drop_long++;
    portEXIT_CRITICAL(&espnow_queue_mux);
    return;
  }

  portENTER_CRITICAL(&espnow_queue_mux);
  telemetry.espnow_packets++;
  if (espnow_queue_count >= ESPNOW_QUEUE_DEPTH) {
    telemetry.espnow_drop_queue++;
    portEXIT_CRITICAL(&espnow_queue_mux);
    return;
  }

  const uint8_t slot = espnow_queue_head;
  memcpy(espnow_queue[slot], incoming_data, len);
  espnow_queue_len[slot] = static_cast<uint8_t>(len);
  espnow_queue_head = static_cast<uint8_t>((espnow_queue_head + 1) % ESPNOW_QUEUE_DEPTH);
  espnow_queue_count++;
  portEXIT_CRITICAL(&espnow_queue_mux);
}

void process_espnow_queue() {
  while (true) {
    uint8_t len = 0;
    uint8_t local[ESPNOW_MAX_PAYLOAD] = {0};

    portENTER_CRITICAL(&espnow_queue_mux);
    if (espnow_queue_count == 0) {
      portEXIT_CRITICAL(&espnow_queue_mux);
      return;
    }
    const uint8_t slot = espnow_queue_tail;
    len = espnow_queue_len[slot];
    memcpy(local, espnow_queue[slot], len);
    espnow_queue_tail = static_cast<uint8_t>((espnow_queue_tail + 1) % ESPNOW_QUEUE_DEPTH);
    espnow_queue_count--;
    portEXIT_CRITICAL(&espnow_queue_mux);

    bool parsed_any = false;
    for (uint8_t i = 0; i + 4 <= len; i++) {
      const uint8_t sync = local[i];
      if (sync != 0xC8 && sync != 0xEE) {
        continue;
      }

      const uint8_t declared = local[i + 1];
      const uint8_t total = static_cast<uint8_t>(declared + 2);
      if (total < 4 || total > CRSF_MAX_FRAME_LEN) {
        continue;
      }
      if (i + total > len) {
        continue;
      }
      if (!is_valid_crsf_frame(&local[i], total)) {
        continue;
      }

      parse_crsf_frame(&local[i], total);
      parsed_any = true;
      i = static_cast<uint8_t>(i + total - 1);
    }

    if (!parsed_any) {
      telemetry.espnow_no_crsf++;
    }
  }
}

void update_board_battery() {
  static unsigned long last = 0;
  if (millis() - last < 500) {
    return;
  }
  last = millis();

  const int mv = analogReadMilliVolts(BOARD_BATTERY_ADC_PIN);
  const float v = (static_cast<float>(mv) / 1000.0f) * BOARD_BATTERY_DIVIDER_RATIO;
  telemetry.board_voltage = v;
  // Below 2 V is not a valid reading for the configured 1S supply.
  telemetry.board_battery_valid = v >= 2.0f && v <= 5.5f;

  float pct = (v - BOARD_BATTERY_MIN_V) / (BOARD_BATTERY_MAX_V - BOARD_BATTERY_MIN_V) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  telemetry.board_percent = static_cast<int>(pct + 0.5f);
}

String wifi_status_text() {
  wl_status_t st = WiFi.status();
  switch (st) {
    case WL_CONNECTED:
      return "connected";
    case WL_IDLE_STATUS:
      return "idle";
    case WL_NO_SSID_AVAIL:
      return "no_ssid";
    case WL_CONNECT_FAILED:
      return "connect_failed";
    case WL_CONNECTION_LOST:
      return "connection_lost";
    case WL_DISCONNECTED:
      return "disconnected";
    default:
      return "unknown";
  }
}

String telemetry_json() {
  String out;
  out.reserve(8192);
  out += "{";
  out += "\"uptime_ms\":" + String(millis()) + ",";
  out += "\"last_frame_ms\":" + String(telemetry.last_frame_ms) + ",";
  out += "\"tx_stream\":{";
  out += "\"received\":" + String(telemetry.frames_total > 0 ? "true" : "false") + ",";
  out += "\"active\":" + String(telemetry.frames_total > 0 && (uint32_t)(millis() - telemetry.last_frame_ms) <= 3000 ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(telemetry.frames_total > 0 ? (uint32_t)(millis() - telemetry.last_frame_ms) : 0) + "},";
  out += "\"last_drone_telem_ms\":" + String(telemetry.last_drone_telem_ms) + ",";
  const uint32_t drone_age_ms = telemetry.last_drone_telem_ms == 0
                                    ? 0
                                    : static_cast<uint32_t>(millis() - telemetry.last_drone_telem_ms);
  const bool drone_telem_active = telemetry.last_drone_telem_ms > 0 && drone_age_ms <= DRONE_TELEMETRY_STALE_MS;
  out += "\"drone_telem\":{";
  out += "\"active\":" + String(drone_telem_active ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(drone_age_ms);
  out += "},";
  out += "\"frames_total\":" + String(telemetry.frames_total) + ",";
  out += "\"crc_errors\":" + String(telemetry.crc_errors) + ",";
  out += "\"unknown_frames\":" + String(telemetry.unknown_frames) + ",";

  out += "\"espnow\":{";
  out += "\"packets\":" + String(telemetry.espnow_packets) + ",";
  out += "\"drop_short\":" + String(telemetry.espnow_drop_short) + ",";
  out += "\"drop_long\":" + String(telemetry.espnow_drop_long) + ",";
  out += "\"drop_queue\":" + String(telemetry.espnow_drop_queue) + ",";
  out += "\"no_crsf\":" + String(telemetry.espnow_no_crsf);
  out += "},";

  out += "\"wifi\":{";
  out += "\"sta_status\":\"" + wifi_status_text() + "\",";
  out += "\"sta_ip\":\"" + WiFi.localIP().toString() + "\",";
  out += "\"note\":\"" + json_escape(wifi_note) + "\"";
  out += "},";
  out += "\"time\":{";
  out += "\"synced\":" + String(clock_synced ? "true" : "false") + ",";
  out += "\"source\":\"" + json_escape(clock_source_name) + "\"";
  out += "},";

  out += "\"sd\":{";
  out += "\"available\":" + String(sd_available ? "true" : "false") + ",";
  out += "\"status\":\"" + json_escape(sd_status) + "\",";
  out += "\"active_log\":\"" + json_escape(active_log_name) + "\",";
  const char *logging_state = !sd_available ? "no_sd" : !config.log.enabled ? "disabled" :
    active_log ? "recording" : !telemetry.arm_known || !fresh(telemetry.mode_ms, 10000) ? "arm_unknown" :
    !telemetry.is_armed ? "waiting_arm" : "not_recording";
  out += "\"logging_state\":\"" + String(logging_state) + "\"";
  out += "},";

  out += "\"board_battery\":{";
  out += "\"valid\":" + String(telemetry.board_battery_valid ? "true" : "false") + ",";
  out += "\"voltage\":" + String(telemetry.board_voltage, 2) + ",";
  out += "\"percent\":" + String(telemetry.board_percent);
  out += "},";

  out += "\"gps\":{";
  out += "\"seen\":" + String(telemetry.gps_ms ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(telemetry.gps_ms ? (uint32_t)(millis() - telemetry.gps_ms) : 0) + ",";
  out += "\"fresh\":" + String(fresh(telemetry.gps_ms, 10000) ? "true" : "false") + ",";
  out += "\"fix\":" + String(telemetry.gps_fix ? "true" : "false") + ",";
  out += "\"sats\":" + String(telemetry.gps_sats) + ",";
  out += "\"lat\":" + String(telemetry.gps_lat, 7) + ",";
  out += "\"lon\":" + String(telemetry.gps_lon, 7) + ",";
  out += "\"speed_kmh\":" + String(telemetry.gps_speed_kmh, 1) + ",";
  out += "\"heading_deg\":" + String(telemetry.gps_heading_deg, 1) + ",";
  out += "\"altitude_m\":" + String(telemetry.gps_altitude_m);
  out += "},";

  out += "\"battery\":{";
  out += "\"seen\":" + String(telemetry.battery_ms ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(telemetry.battery_ms ? (uint32_t)(millis() - telemetry.battery_ms) : 0) + ",";
  out += "\"fresh\":" + String(fresh(telemetry.battery_ms, 10000) ? "true" : "false") + ",";
  out += "\"voltage\":" + String(telemetry.battery_voltage, 2) + ",";
  out += "\"current\":" + String(telemetry.battery_current, 2) + ",";
  out += "\"capacity_mah\":" + String(telemetry.battery_capacity_mah) + ",";
  out += "\"remaining_pct\":" + String(telemetry.battery_remaining_pct);
  out += "},";

  out += "\"link\":{";
  out += "\"seen\":" + String(telemetry.link_ms ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(telemetry.link_ms ? (uint32_t)(millis() - telemetry.link_ms) : 0) + ",";
  out += "\"fresh\":" + String(fresh(telemetry.link_ms, 10000) ? "true" : "false") + ",";
  out += "\"rssi1_dbm\":" + String(telemetry.link_rssi_1_dbm) + ",";
  out += "\"rssi2_dbm\":" + String(telemetry.link_rssi_2_dbm) + ",";
  out += "\"lq\":" + String(telemetry.link_quality) + ",";
  out += "\"snr_db\":" + String(telemetry.link_snr_db) + ",";
  out += "\"active_antenna\":" + String(telemetry.link_active_antenna) + ",";
  out += "\"rf_mode\":" + String(telemetry.link_rf_mode) + ",";
  out += "\"tx_power\":" + String(telemetry.link_tx_power) + ",";
  out += "\"down_rssi_dbm\":" + String(telemetry.link_down_rssi_dbm) + ",";
  out += "\"down_lq\":" + String(telemetry.link_down_quality) + ",";
  out += "\"down_snr_db\":" + String(telemetry.link_down_snr_db);
  out += "},";

  out += "\"attitude\":{";
  out += "\"seen\":" + String(telemetry.attitude_ms ? "true" : "false") + ",";
  out += "\"age_ms\":" + String(telemetry.attitude_ms ? (uint32_t)(millis() - telemetry.attitude_ms) : 0) + ",";
  out += "\"fresh\":" + String(fresh(telemetry.attitude_ms, 10000) ? "true" : "false") + ",";
  out += "\"pitch_deg\":" + String(telemetry.attitude_pitch_deg, 2) + ",";
  out += "\"roll_deg\":" + String(telemetry.attitude_roll_deg, 2) + ",";
  out += "\"yaw_deg\":" + String(telemetry.attitude_yaw_deg, 2) + ",";
  out += "\"vario_ms\":" + String(telemetry.vario_ms, 2);
  out += "},";

  out += "\"mode_seen\":" + String(telemetry.mode_ms ? "true" : "false") + ",";
  out += "\"mode_fresh\":" + String(fresh(telemetry.mode_ms, 10000) ? "true" : "false") + ",";
  out += "\"mode_age_ms\":" + String(telemetry.mode_ms ? (uint32_t)(millis() - telemetry.mode_ms) : 0) + ",";
  out += "\"flight_mode\":\"" + json_escape(telemetry.flight_mode) + "\",";
  out += "\"is_armed\":" + String(telemetry.arm_known && fresh(telemetry.mode_ms, 10000) ? (telemetry.is_armed ? "true" : "false") : "null") + ",";

  const bool alarm_active = last_alarm_uplink || last_alarm_downlink || last_alarm_lq || last_alarm_telem ||
                            last_alarm_board_batt || last_alarm_drone_batt;
  out += "\"alarm\":{";
  out += "\"active\":" + String(alarm_active ? "true" : "false") + ",";
  out += "\"active_names\":[";
  bool alarm_first = true;
  auto append_alarm = [&](bool active, const char *name) {
    if (!active) return;
    if (!alarm_first) out += ",";
    alarm_first = false;
    out += "\"" + String(name) + "\"";
  };
  append_alarm(last_alarm_uplink, "uplink_rssi_low");
  append_alarm(last_alarm_downlink, "downlink_rssi_low");
  append_alarm(last_alarm_lq, "lq_low");
  append_alarm(last_alarm_telem, "telemetry_lost");
  append_alarm(last_alarm_board_batt, "board_battery_low");
  append_alarm(last_alarm_drone_batt, "drone_battery_low");
  out += "],";
  out += "\"event_id\":" + String(alarm_event_id) + ",";
  out += "\"name\":\"" + json_escape(last_alarm_name) + "\",";
  out += "\"sound\":\"" + json_escape(last_alarm_sound) + "\",";
  out += "\"ms\":" + String(last_alarm_ms);
  out += "},";

  out += "\"frame_types\":[";
  bool first = true;
  for (int i = 0; i < 256; i++) {
    if (frame_stats[i].count == 0) continue;
    if (!first) out += ",";
    first = false;
    out += "{";
    out += "\"type\":" + String(i) + ",";
    out += "\"count\":" + String(frame_stats[i].count) + ",";
    out += "\"payload_len\":" + String(frame_stats[i].last_payload_len) + ",";
    out += "\"last_hex\":\"" + json_escape(frame_stats[i].last_payload_hex) + "\"";
    out += "}";
  }
  out += "],";

  out += "\"unknown_types\":[";
  first = true;
  for (int i = 0; i < 256; i++) {
    if (frame_stats[i].count == 0 || is_known_frame_type(static_cast<uint8_t>(i))) continue;
    if (!first) out += ",";
    first = false;
    out += "{";
    out += "\"type\":" + String(i) + ",";
    out += "\"count\":" + String(frame_stats[i].count) + ",";
    out += "\"last_hex\":\"" + json_escape(frame_stats[i].last_payload_hex) + "\"";
    out += "}";
  }
  out += "]";

  out += "}";
  return out;
}

String config_json() {
  String out;
  out.reserve(4096);
  out += "{";

  out += "\"wifi\":{\"default_password\":" + String(ap_password == DEFAULT_AP_PASSWORD ? "true" : "false") + "},";

  out += "\"bind\":{";
  out += "\"bind_phrase\":\"" + json_escape(config.bind_phrase) + "\",";
  out += "\"uid_text\":\"" + json_escape(config.uid_text) + "\",";
  out += "\"effective_uid\":\"" + json_escape(uid_to_text(config.uid)) + "\"";
  out += "},";

  out += "\"profiles\":[";
  bool pf_first = true;
  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    if (profile_is_empty(config.profiles[i])) continue;
    if (!pf_first) out += ",";
    pf_first = false;
    out += "{";
    out += "\"index\":" + String(i) + ",";
    out += "\"note\":\"" + json_escape(config.profiles[i].note) + "\",";
    out += "\"bind_phrase\":\"" + json_escape(config.profiles[i].bind_phrase) + "\",";
    out += "\"uid_text\":\"" + json_escape(config.profiles[i].uid_text) + "\"";
    out += "}";
  }
  out += "],";

  out += "\"logging\":{";
  out += "\"enabled\":" + String(config.log.enabled ? "true" : "false") + ",";
  out += "\"keep_files\":" + String(config.log.keep_files) + ",";
  out += "\"log_gps\":" + String(config.log.log_gps ? "true" : "false") + ",";
  out += "\"log_link\":" + String(config.log.log_link ? "true" : "false") + ",";
  out += "\"log_battery\":" + String(config.log.log_battery ? "true" : "false") + ",";
  out += "\"log_attitude\":" + String(config.log.log_attitude ? "true" : "false") + ",";
  out += "\"log_unknown\":" + String(config.log.log_unknown ? "true" : "false");
  out += "},";

  out += "\"buzzer\":{";
  out += "\"enabled\":" + String(config.buzzer.enabled ? "true" : "false") + ",";
  out += "\"uplink_en\":" + String(config.buzzer.alarm_uplink_rssi_enabled ? "true" : "false") + ",";
  out += "\"uplink_min\":" + String(config.buzzer.alarm_uplink_rssi_min_dbm) + ",";
  out += "\"uplink_sound\":\"" + json_escape(config.buzzer.alarm_uplink_sound) + "\",";
  out += "\"down_en\":" + String(config.buzzer.alarm_downlink_rssi_enabled ? "true" : "false") + ",";
  out += "\"down_min\":" + String(config.buzzer.alarm_downlink_rssi_min_dbm) + ",";
  out += "\"down_sound\":\"" + json_escape(config.buzzer.alarm_downlink_sound) + "\",";
  out += "\"lq_en\":" + String(config.buzzer.alarm_lq_enabled ? "true" : "false") + ",";
  out += "\"lq_min\":" + String(config.buzzer.alarm_lq_min) + ",";
  out += "\"lq_sound\":\"" + json_escape(config.buzzer.alarm_lq_sound) + "\",";
  out += "\"telem_en\":" + String(config.buzzer.alarm_telem_loss_enabled ? "true" : "false") + ",";
  out += "\"telem_sec\":" + String(config.buzzer.alarm_telem_loss_sec) + ",";
  out += "\"telem_sound\":\"" + json_escape(config.buzzer.alarm_telem_loss_sound) + "\",";
  out += "\"board_batt_en\":" + String(config.buzzer.alarm_board_batt_enabled ? "true" : "false") + ",";
  out += "\"board_batt_min\":" + String(config.buzzer.alarm_board_batt_min_pct) + ",";
  out += "\"board_batt_sound\":\"" + json_escape(config.buzzer.alarm_board_batt_sound) + "\",";
  out += "\"drone_batt_en\":" + String(config.buzzer.alarm_drone_batt_enabled ? "true" : "false") + ",";
  out += "\"drone_batt_min\":" + String(config.buzzer.alarm_drone_batt_min_pct) + ",";
  out += "\"drone_batt_sound\":\"" + json_escape(config.buzzer.alarm_drone_batt_sound) + "\"";
  out += "},";

  out += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"";
  out += "}";

  return out;
}

String logs_json() {
  String out;
  out.reserve(2048);
  out += "{";
  out += "\"sd_available\":" + String(sd_available ? "true" : "false") + ",";
  out += "\"files\":[";

  if (sd_available) {
    File dir = SD.open("/logs");
    bool first = true;
    if (dir && dir.isDirectory()) {
      File f = dir.openNextFile();
      while (f) {
        if (!f.isDirectory()) {
          if (!first) out += ",";
          first = false;
          out += "{";
          out += "\"name\":\"" + json_escape(String(f.name())) + "\",";
          out += "\"size\":" + String(f.size());
          out += "}";
        }
        f = dir.openNextFile();
      }
    }
    if (dir) dir.close();
  }

  out += "]}";
  return out;
}

String sounds_json() {
  String out;
  out.reserve(2048);
  out += "{";
  out += "\"sd_available\":" + String(sd_available ? "true" : "false") + ",";
  out += "\"files\":[";

  if (sd_available) {
    File dir = SD.open("/sound");
    bool first = true;
    if (dir && dir.isDirectory()) {
      File f = dir.openNextFile();
      while (f) {
        if (!f.isDirectory()) {
          if (!first) out += ",";
          first = false;
          out += "\"" + json_escape(String(f.name())) + "\"";
        }
        f = dir.openNextFile();
      }
    }
    if (dir) dir.close();
  }

  out += "]}";
  return out;
}

String gps_points_json() {
  String out;
  out.reserve(8192);
  out += "{\"points\":[";

  if (!sd_available) {
    out += "]}";
    return out;
  }

  File dir = SD.open("/logs");
  String names[128];
  int count = 0;
  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f && count < 128) {
      if (!f.isDirectory()) names[count++] = String(f.name());
      f = dir.openNextFile();
    }
  }
  if (dir) dir.close();

  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (names[j] > names[i]) {
        String t = names[i];
        names[i] = names[j];
        names[j] = t;
      }
    }
  }

  bool first = true;
  int emitted = 0;
  const int max_points = server.hasArg("max") ? max(1, min(200, static_cast<int>(server.arg("max").toInt()))) : 80;
  for (int i = 0; i < count && emitted < max_points; i++) {
    String path = names[i];
    if (!path.startsWith("/")) path = "/" + path;
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) continue;

    while (f.available() && emitted < max_points) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.isEmpty() || line.startsWith("ms,")) continue;

      int p1 = line.indexOf(',');
      int p2 = line.indexOf(',', p1 + 1);
      int p3 = line.indexOf(',', p2 + 1);
      int p4 = line.indexOf(',', p3 + 1);
      int p5 = line.indexOf(',', p4 + 1);
      int p6 = line.indexOf(',', p5 + 1);
      int p7 = line.indexOf(',', p6 + 1);
      if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0 || p6 < 0 || p7 < 0) continue;

      const String ms_s = line.substring(0, p1);
      const String ts_s = line.substring(p1 + 1, p2);
      const String uptime_s = line.substring(p2 + 1, p3);
      const String type_s = line.substring(p3 + 1, p4);
      if (type_s.toInt() != 2) continue;
      const String lat_s = line.substring(p5 + 1, p6);
      const String lon_s = line.substring(p6 + 1, p7);
      const double lat = lat_s.toDouble();
      const double lon = lon_s.toDouble();
      if (lat == 0.0 && lon == 0.0) continue;

      if (!first) out += ",";
      first = false;
      out += "{";
      out += "\"ms\":" + ms_s + ",";
      out += "\"timestamp\":\"" + json_escape(ts_s) + "\",";
      out += "\"uptime\":\"" + json_escape(uptime_s) + "\",";
      out += "\"lat\":" + String(lat, 7) + ",";
      out += "\"lon\":" + String(lon, 7) + ",";
      out += "\"log\":\"" + json_escape(path) + "\"";
      out += "}";
      emitted++;
    }
    f.close();
  }

  out += "]}";
  return out;
}

bool apply_uid_to_station_mac() {
  WiFi.mode(WIFI_AP_STA);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  return esp_wifi_set_mac(WIFI_IF_STA, config.uid) == ESP_OK;
}

bool is_safe_log_path(const String &name) {
  if (!name.startsWith("/logs/")) return false;
  if (name.indexOf("..") >= 0 || name.indexOf('\\') >= 0) return false;
  const int leaf = name.lastIndexOf('/');
  if (leaf != 5 || leaf == static_cast<int>(name.length()) - 1) return false;
  return name.endsWith(".csv");
}

bool init_espnow_for_elrs();

bool restart_espnow_for_elrs() {
  if (espnow_ready) {
    esp_now_deinit();
    espnow_ready = false;
  }

  portENTER_CRITICAL(&espnow_queue_mux);
  espnow_queue_head = 0;
  espnow_queue_tail = 0;
  espnow_queue_count = 0;
  portEXIT_CRITICAL(&espnow_queue_mux);

  return init_espnow_for_elrs();
}

void schedule_espnow_restart() {
  espnow_restart_pending = true;
}

void service_pending_espnow_restart() {
  if (!espnow_restart_pending) return;
  espnow_restart_pending = false;
  if (!restart_espnow_for_elrs()) {
    Serial.println("scheduled ESP-NOW restart failed");
  }
}

void handle_root() {
  static const char HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ELRS Backpack Panel</title>
  <script src="/qrcode.js"></script>
  <style>
    :root{font-family:system-ui,-apple-system,sans-serif;color:#e6ebf2;background:#0d1219;color-scheme:dark}
    *{box-sizing:border-box}html,body{width:100%;max-width:100%;overflow-x:hidden}
    body{margin:0;min-height:100vh;background:radial-gradient(ellipse at top,#1a2937,#0d1219 65%)}
    main{width:100%;max-width:720px;min-width:0;margin:auto;padding:30px 20px 60px}
    header{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:28px}
    .brand{font-size:12px;font-weight:750;letter-spacing:.18em;color:#9dabbc}
    .badge{font-size:12px;color:#8ce1bf;background:#16342c;padding:7px 10px;border-radius:99px;white-space:nowrap}
    h1{font-size:30px;letter-spacing:-.035em;margin:0 0 8px} .intro{color:#a5b2c2;line-height:1.55;margin:0 0 20px}
    .tabs{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:5px;background:#111a24;border:1px solid #2a3949;border-radius:13px;padding:4px;margin:0 0 18px}
    .tab{min-width:0;padding:11px 3px;background:transparent;border:0;border-radius:9px;color:#aab9ca;font-size:12px;font-weight:700;cursor:pointer}
    .tab.active{background:#2a3b49;color:#f1f8fa}.pane{display:none}.pane.active{display:block}
    .card,.bar-wrap{min-width:0;background:#151e29;border:1px solid #2a3949;border-radius:18px;padding:20px;margin:14px 0}
    .card h3{font-size:17px;letter-spacing:-.01em;margin:0 0 14px}.card h4{font-size:14px;color:#e6ebf2;margin:22px 0 10px;padding-top:16px;border-top:1px solid #2a3949}
    .row{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:10px 0;min-width:0}.row>span{min-width:0;overflow-wrap:anywhere}.k,.muted{color:#9dabbc}.muted{font-size:12px;line-height:1.5;overflow-wrap:anywhere}.v{font-weight:700;text-align:right;overflow-wrap:anywhere}
    input:not([type=checkbox]),select{width:100%;min-width:0;background:#0d151e;color:#f0f5fb;border:1px solid #3c4c5e;border-radius:10px;padding:12px;font:inherit;outline:none}
    .row input[type=number],.row select{width:min(55%,220px)}input:focus,select:focus{border-color:#64d2ae;box-shadow:0 0 0 3px #64d2ae22}
    input[type=checkbox]{accent-color:#64d2ae;width:19px!important;height:19px;flex:none}
    button{border:0;border-radius:11px;padding:12px 15px;background:#293a47;color:#e6f1f1;font:inherit;font-size:13px;font-weight:700;cursor:pointer}
    button:hover{filter:brightness(1.12)}button:focus-visible,.tab:focus-visible{outline:2px solid #75deb9;outline-offset:2px}
    #save_bind,#save_log_cfg,#save_bz_cfg,#upload_sound_btn,#wifi_save{width:100%;background:#75deb9;color:#0b2119}
    .status-card .row{align-items:flex-start}.status-card .v{font-size:13px}.sensor-age{font-size:12px;color:#a5b2c2;margin:-6px 0 12px}.alert-list{border-color:#b84c45;color:#fecaca}.alert-list[hidden]{display:none}.ok{color:#8ce1bf}.warn{color:#ffca83}#state{padding:0 2px;font-size:13px;font-weight:650}
    .grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.grid .card{margin:0}
    code{display:block;white-space:pre-wrap;overflow-wrap:anywhere;color:#a5b2c2;font-size:12px}hr{border:0;border-top:1px solid #2a3949;margin:16px 0}
    .bar-wrap{margin-top:0}.bar-row{display:flex;align-items:center;gap:10px;margin:8px 0}.emoji{font-size:18px;width:24px;text-align:center}
    .bar{flex:1;min-width:0;height:10px;border-radius:99px;background:#293a47;overflow:hidden}.fill{height:100%;width:0%;transition:width .2s;background:#75deb9}.fill.yellow{background:#facc15}.fill.red{background:#ef4444}
    .telemetry-stale{opacity:.65}.telemetry-stale h3,.telemetry-stale .k,.telemetry-stale .v{text-decoration:line-through}.telemetry-stale .v{font-weight:600}#drone_bar_row.telemetry-stale .bar{opacity:.45}#drone_bar_row.telemetry-stale #txt_drone{text-decoration:line-through}
    body.alarm-flash{background:#7f1d1d!important;transition:background .15s}#alarm_overlay{position:fixed;inset:0;display:none;align-items:center;justify-content:center;pointer-events:none;z-index:10}#alarm_box{background:#151e29;border:1px solid #ef4444;color:#fecaca;padding:16px 22px;border-radius:12px;font-weight:700;font-size:20px}
    #qr{margin-top:14px}#qr img,#qr canvas{max-width:100%}a{color:#75deb9}input[type=file]{font-size:12px}
    @media(max-width:520px){main{padding:24px 16px 48px}.grid{grid-template-columns:minmax(0,1fr)}.tabs{grid-template-columns:repeat(2,minmax(0,1fr))}.card,.bar-wrap{padding:18px}.row{gap:8px}}
  </style>
</head>
<body>
<main>
  <header><span class="brand">BACKPACK / CONTROL</span><span class="badge">ESP-NOW</span></header>
  <h1>Backpack</h1>
  <p class="intro">Telemetry, logs and alerts.</p>
  <div id="default_password_warning" class="card warn" hidden>Default Wi-Fi password is in use. Change it in Settings.</div>

  <div id="alarm_overlay"><div id="alarm_box">ALARM</div></div>

  <div class="bar-wrap">
    <div class="bar-row" id="board_bar_row">
      <div class="emoji">⚙️</div>
      <div class="bar"><div id="bar_board" class="fill"></div></div>
      <div id="txt_board" class="muted">0%</div>
    </div>
    <div class="bar-row" id="drone_bar_row">
      <div class="emoji">✈️</div>
      <div class="bar"><div id="bar_drone" class="fill"></div></div>
      <div id="txt_drone" class="muted">0%</div>
    </div>
  </div>

  <nav class="tabs" aria-label="Sections">
    <button type="button" class="tab active" data-tab="telemetry">Telemetry</button>
    <button type="button" class="tab" data-tab="bind">Bind</button>
    <button type="button" class="tab" data-tab="logging">Logging</button>
    <button type="button" class="tab" data-tab="buzzer">Buzzer</button>
    <button type="button" class="tab" data-tab="settings">Settings</button>
  </nav>

  <div id="telemetry" class="pane active">
    <div class="card status-card">
      <h3>Connection</h3>
      <div class="row"><span class="k">Backpack frames</span><span id="tx_stream_status" class="v">Waiting</span></div>
      <div class="row"><span class="k">Model telemetry</span><span id="drone_status" class="v">No data</span></div>
      <div class="row"><span class="k">Logging</span><span id="recording_status" class="v">Checking</span></div>
      <div id="state" class="muted">Waiting for data</div>
    </div>
    <div id="active_alarms" class="card alert-list" role="status" hidden></div>
    <div class="grid">
      <div class="card" id="gps"></div>
      <div class="card" id="bat"></div>
      <div class="card" id="link"></div>
      <div class="card" id="att"></div>
      <div class="card" id="sys"></div>
      <div class="card" id="unknown"></div>
    </div>
  </div>

  <div id="bind" class="pane">
    <div class="card">
      <h3>Current Bind</h3>
      <div class="row"><span class="k">Profile note for save</span></div>
      <input id="bind_note" placeholder="TX #1" />
      <div class="row"><span class="k">Bind Phrase</span></div>
      <input id="bind_phrase" placeholder="your bind phrase" />
      <div class="row"><span class="k">UID Digits (6 numbers)</span></div>
      <input id="uid_text" placeholder="106,19,19,206,193,30" />
      <div class="row"><button id="save_bind">Save + Apply (as profile)</button></div>
      <div class="row"><span class="k">Effective UID</span><span id="effective_uid" class="v">-</span></div>
      <div id="bind_msg" class="muted"></div>
    </div>

    <div class="card">
      <h3>Saved Profiles</h3>
      <div class="muted">Only saved profiles are shown.</div>
      <div id="profiles_list" class="muted"></div>
    </div>
  </div>

  <div id="logging" class="pane">
    <div class="card">
      <h3>SD + Logging</h3>
      <div id="sd_status" class="muted">SD status...</div>
      <div class="row"><span class="k">Enable logging (only when armed)</span><input id="log_enabled" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Keep files</span><input id="log_keep" type="number" min="1" max="100" value="10"></div>
      <div class="row"><span class="k">Log GPS</span><input id="log_gps" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Log Link</span><input id="log_link" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Log Battery</span><input id="log_battery" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Log Attitude</span><input id="log_attitude" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Log Unknown</span><input id="log_unknown" type="checkbox" style="width:auto"></div>
      <div class="row"><button id="save_log_cfg">Save Log Config</button></div>
      <div id="log_msg" class="muted"></div>
    </div>

    <div class="card">
      <h3>Logs Browser</h3>
      <div id="logs_list" class="muted">No logs</div>
      <code id="log_preview"></code>
    </div>

    <div class="card">
      <h3>GPS QR History</h3>
      <div class="row"><button id="btn_load_points">Load GPS Points</button></div>
      <div class="row">
        <button id="btn_prev_qr">Prev</button>
        <button id="btn_next_qr">Next</button>
      </div>
      <div id="qr_meta" class="muted">No points loaded</div>
      <div id="qr"></div>
    </div>
  </div>

  <div id="buzzer" class="pane">
    <div class="card">
      <h3>Buzzer Alarms</h3>
      <div class="row"><span class="k">Enable buzzer</span><input id="bz_enabled" type="checkbox" style="width:auto"></div>

      <h4>Uplink RSSI low</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_u_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Min dBm</span><input id="bz_u_min" type="number" value="-95"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_u_snd"></select></div>

      <h4>Downlink RSSI low</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_d_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Min dBm</span><input id="bz_d_min" type="number" value="-95"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_d_snd"></select></div>

      <h4>LQ low</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_lq_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Min LQ</span><input id="bz_lq_min" type="number" value="70"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_lq_snd"></select></div>

      <h4>Telemetry loss</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_t_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Timeout sec</span><input id="bz_t_sec" type="number" value="3"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_t_snd"></select></div>

      <h4>Board battery low</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_bb_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Min %</span><input id="bz_bb_min" type="number" value="30"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_bb_snd"></select></div>

      <h4>Drone battery low</h4>
      <div class="row"><span class="k">Enable</span><input id="bz_db_en" type="checkbox" style="width:auto"></div>
      <div class="row"><span class="k">Min %</span><input id="bz_db_min" type="number" value="30"></div>
      <div class="row"><span class="k">Sound file</span><select id="bz_db_snd"></select></div>

      <div class="row"><button id="save_bz_cfg">Save Buzzer Config</button></div>
      <div id="bz_msg" class="muted"></div>
    </div>
    <div class="card">
      <h3>Sound Upload</h3>
      <div class="muted">Upload alarm files to SD `/sound`.</div>
      <input id="sound_file" type="file" accept=".wav,.mp3,.ogg" />
      <div class="row"><button id="upload_sound_btn">Upload Sound</button></div>
      <div id="upload_msg" class="muted"></div>
    </div>
  </div>

  <div id="settings" class="pane">
    <form id="wifi_form" class="card">
      <h3>Wi-Fi password</h3>
      <p class="muted">Enter 8–63 printable ASCII characters. The new password takes effect after restarting Backpack.</p>
      <p class="muted">To reset the Wi-Fi password to 12345678, hold BOOT for more than 15 seconds while running, then release it. Backpack restarts.</p>
      <input id="wifi_password" type="password" autocomplete="new-password" maxlength="63" placeholder="New password" aria-label="New Wi-Fi password">
      <input id="wifi_confirm" type="password" autocomplete="new-password" maxlength="63" placeholder="Confirm password" aria-label="Confirm Wi-Fi password" style="margin-top:12px">
      <div class="row"><button id="wifi_save" type="submit">Save password</button></div>
      <div id="wifi_msg" class="muted" role="status"></div>
    </form>
  </div>

<script>
const by = id => document.getElementById(id);
const f = (n, d = 1) => Number.isFinite(n) ? Number(n).toFixed(d) : "-";
const h = v => String(v ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let cfg = null;
let sounds = [];
let lastAlarmId = 0;
let alarmPopupUntilMs = 0;
let gpsPoints = [];
let gpsIdx = -1;

function setTab(name){
  document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active', t.dataset.tab===name));
  document.querySelectorAll('.pane').forEach(p=>p.classList.toggle('active', p.id===name));
}
document.querySelectorAll('.tab').forEach(t=>t.onclick=()=>setTab(t.dataset.tab));

function fillSoundSelect(id, value){
  const s = by(id);
  s.innerHTML = '';
  for (const n of sounds){
    const o = document.createElement('option');
    o.value = n;
    o.textContent = n;
    s.appendChild(o);
  }
  if (!sounds.includes(value)) {
    const o = document.createElement('option');
    o.value = value || '';
    o.textContent = value || '(none)';
    s.appendChild(o);
  }
  s.value = value || '';
}

function drawUnknown(arr){
  let html = '<h3>Unknown Telemetry Types</h3>';
  if (!arr || arr.length === 0) {
    html += '<div class="muted">No unknown types.</div>';
  } else {
    for (const u of arr){
      html += `<div class="row"><span class="k">0x${Number(u.type).toString(16).toUpperCase().padStart(2,'0')}</span><span class="v">count ${u.count}</span></div>`;
      html += `<div class="muted">${h(u.last_hex)}</div>`;
    }
  }
  by('unknown').innerHTML = html;
}

function drawQR(gps){
  const node = by('qr');
  node.innerHTML = '';
  if (!gps || !gps.fix || !gps.lat || !gps.lon) {
    node.textContent = 'No GPS fix';
    return;
  }
  const txt = `https://maps.google.com/?q=${gps.lat},${gps.lon}`;
  if (typeof QRCode === 'function') {
    new QRCode(node, {text: txt, width: 160, height: 160});
  } else {
    const a = document.createElement('a');
    a.href = txt;
    a.textContent = txt;
    node.appendChild(a);
  }
}

function setBar(id, txtId, pct){
  if (pct === null || pct === undefined) { by(id).style.width = '0%'; by(txtId).textContent = '—'; return; }
  const p = Math.max(0, Math.min(100, Number(pct)));
  const el = by(id);
  el.style.width = `${p}%`;
  el.classList.remove('yellow','red');
  if (p < 40) el.classList.add('red');
  else if (p < 80) el.classList.add('yellow');
  by(txtId).textContent = `${p}%`;
}

function maybePlayAlarm(t){
  const now = Date.now();
  const alarm = t.alarm || {};
  const active = !!alarm.active;
  const list = by('active_alarms');
  const names = Array.isArray(alarm.active_names) ? alarm.active_names : [];
  list.hidden = !active || names.length === 0;
  list.textContent = names.length ? 'Active alerts: ' + names.join(', ') : '';

  if (active && alarm.event_id > lastAlarmId) {
    lastAlarmId = alarm.event_id;
    by('alarm_box').textContent = alarm.name || 'ALARM';
    alarmPopupUntilMs = now + 2500;
    if (alarm.sound) {
      const a = new Audio(`/sound?file=${encodeURIComponent(alarm.sound)}`);
      a.play().catch(()=>{});
    }
  }

  by('alarm_overlay').style.display = now < alarmPopupUntilMs ? 'flex' : 'none';
}

function setStaleView(isStale){
  // Each sensor has its own freshness; see drawTelemetry.
  document.querySelectorAll('.drone-only').forEach(el => {
    el.classList.toggle('telemetry-stale', isStale);
  });
  by('drone_bar_row').classList.toggle('telemetry-stale', isStale);
}

function sensorTitle(title, data){
  const age = data.seen && !data.fresh ? `<div class="sensor-age">Last update ${(Number(data.age_ms || 0)/1000).toFixed(1)} s ago</div>` : '';
  return `<h3>${title}</h3>${age}`;
}
function drawTelemetry(t){
  const tx = t.tx_stream || {};
  by('tx_stream_status').textContent = !tx.received ? 'Waiting for frames' : tx.active ? 'Frames received' : `No recent frames (${(Number(tx.age_ms || 0)/1000).toFixed(1)} s)`;
  by('drone_status').textContent = !t.last_drone_telem_ms ? 'No data yet' : t.drone_telem && t.drone_telem.active ? 'Data received' : `No recent data (${(Number(t.drone_telem?.age_ms || 0)/1000).toFixed(1)} s)`;
  const ageMs = Number((t.drone_telem && t.drone_telem.age_ms) || 0);
  const hasDroneTelem = Number(t.last_drone_telem_ms || 0) > 0;
  const isStale = hasDroneTelem && !(t.drone_telem && t.drone_telem.active);
  const hasFrames = Number(t.frames_total || 0) > 0;
  if (!hasFrames && !hasDroneTelem) {
    by('state').textContent = 'No telemetry yet (waiting TX stream)';
    by('state').className = 'warn';
  } else if (!hasDroneTelem) {
    by('state').textContent = 'No drone telemetry yet';
    by('state').className = 'warn';
  } else if (isStale) {
    by('state').textContent = `Drone telemetry stale (${(ageMs / 1000).toFixed(1)} s since last drone frame)`;
    by('state').className = 'warn';
  } else {
    by('state').textContent = 'Drone telemetry active';
    by('state').className = 'ok';
  }
  by('gps').innerHTML = `${sensorTitle('GPS', t.gps)}${!t.gps.seen ? '<div class="muted">No data</div>' : `<div class="row"><span class="k">Fix</span><span class="v">${t.gps.fix}</span></div><div class="row"><span class="k">Sats</span><span class="v">${t.gps.sats}</span></div><div class="row"><span class="k">Lat</span><span class="v">${f(t.gps.lat,7)}</span></div><div class="row"><span class="k">Lon</span><span class="v">${f(t.gps.lon,7)}</span></div><div class="row"><span class="k">Speed</span><span class="v">${f(t.gps.speed_kmh)} km/h</span></div>`}`;
  by('bat').innerHTML = `${sensorTitle('Battery', t.battery)}<div class="drone-only">${!t.battery.seen ? '<div class="muted">No data</div>' : `<div class="row"><span class="k">FC Voltage</span><span class="v">${f(t.battery.voltage,2)} V</span></div><div class="row"><span class="k">FC Current</span><span class="v">${f(t.battery.current,2)} A</span></div><div class="row"><span class="k">FC Remain</span><span class="v">${t.battery.remaining_pct} %</span></div>`}</div><hr><div class="row"><span class="k">Board Voltage</span><span class="v">${t.board_battery.valid ? f(t.board_battery.voltage,2)+' V' : 'No data'}</span></div><div class="row"><span class="k">Board Percent</span><span class="v">${t.board_battery.valid ? t.board_battery.percent+' %' : 'No data'}</span></div>`;
  by('link').innerHTML = `${sensorTitle('Link', t.link)}${!t.link.seen ? '<div class="muted">No data</div>' : `<div class="row"><span class="k">RSSI1</span><span class="v">${t.link.rssi1_dbm} dBm</span></div><div class="row"><span class="k">RSSI2</span><span class="v">${t.link.rssi2_dbm} dBm</span></div><div class="row"><span class="k">LQ</span><span class="v">${t.link.lq}</span></div><div class="row"><span class="k">SNR</span><span class="v">${t.link.snr_db} dB</span></div><div class="row"><span class="k">Active Ant</span><span class="v">${t.link.active_antenna}</span></div><div class="row"><span class="k">Down RSSI</span><span class="v">${t.link.down_rssi_dbm} dBm</span></div><div class="row"><span class="k">Down LQ</span><span class="v">${t.link.down_lq}</span></div><div class="row"><span class="k">Down SNR</span><span class="v">${t.link.down_snr_db} dB</span></div>`}`;
  by('att').innerHTML = `${sensorTitle('Attitude', t.attitude)}<div class="att-only">${!t.attitude.seen ? '<div class="muted">No attitude data</div>' : `<div class="row"><span class="k">Pitch</span><span class="v">${f(t.attitude.pitch_deg,2)}</span></div><div class="row"><span class="k">Roll</span><span class="v">${f(t.attitude.roll_deg,2)}</span></div><div class="row"><span class="k">Yaw</span><span class="v">${f(t.attitude.yaw_deg,2)}</span></div>`}</div><div class="row"><span class="k">Mode</span><span class="v">${!t.mode_seen ? 'No data' : h(t.flight_mode)+(t.mode_fresh ? '' : ` (${(Number(t.mode_age_ms || 0)/1000).toFixed(1)} s ago)`)}</span></div><div class="row"><span class="k">Armed</span><span class="v">${t.is_armed === null ? 'unknown' : t.is_armed}</span></div>`;
  by('sys').innerHTML = `<h3>System</h3><div class="row"><span class="k">Frames</span><span class="v">${t.frames_total}</span></div><div class="row"><span class="k">ESPNow</span><span class="v">${t.espnow.packets}</span></div><div class="row"><span class="k">Queue Drop</span><span class="v">${t.espnow.drop_queue}</span></div><div class="row"><span class="k">No CRSF</span><span class="v">${t.espnow.no_crsf}</span></div><div class="row"><span class="k">SD</span><span class="v">${t.sd.available}</span></div><div class="row"><span class="k">SD Status</span><span class="v">${h(t.sd.status)}</span></div><div class="row"><span class="k">Active log</span><span class="v">${h(t.sd.active_log)}</span></div>`;
  const logStates={no_sd:'No SD card',disabled:'Logging disabled',recording:'Recording',arm_unknown:'Not recording: ARM state unknown',waiting_arm:'Waiting for ARM',not_recording:'Not recording: check SD'};
  by('sd_status').textContent = logStates[t.sd.logging_state] || `SD: ${t.sd.status}`;
  by('recording_status').textContent = by('sd_status').textContent;
  setBar('bar_board','txt_board', t.board_battery.valid ? t.board_battery.percent : null);
  setBar('bar_drone','txt_drone', t.battery.seen ? t.battery.remaining_pct : null);
  setStaleView(!!t.battery.seen && !t.battery.fresh);
  by('gps').classList.toggle('telemetry-stale', !!t.gps.seen && !t.gps.fresh);
  by('link').classList.toggle('telemetry-stale', !!t.link.seen && !t.link.fresh);
  by('att').querySelector('.att-only').classList.toggle('telemetry-stale', !!t.attitude.seen && !t.attitude.fresh);
  drawUnknown(t.unknown_types);
  maybePlayAlarm(t);
}

async function loadConfig(){
  const r = await fetch('/api/config', {cache:'no-store'});
  cfg = await r.json();
  by('default_password_warning').hidden = !(cfg.wifi && cfg.wifi.default_password);

  by('bind_phrase').value = cfg.bind.bind_phrase || '';
  by('uid_text').value = cfg.bind.uid_text || '';
  by('effective_uid').textContent = cfg.bind.effective_uid || '-';

  const l = cfg.logging;
  by('log_enabled').checked = !!l.enabled;
  by('log_keep').value = l.keep_files;
  by('log_gps').checked = !!l.log_gps;
  by('log_link').checked = !!l.log_link;
  by('log_battery').checked = !!l.log_battery;
  by('log_attitude').checked = !!l.log_attitude;
  by('log_unknown').checked = !!l.log_unknown;

  const b = cfg.buzzer;
  by('bz_enabled').checked = !!b.enabled;
  by('bz_u_en').checked = !!b.uplink_en;
  by('bz_u_min').value = b.uplink_min;
  by('bz_d_en').checked = !!b.down_en;
  by('bz_d_min').value = b.down_min;
  by('bz_lq_en').checked = !!b.lq_en;
  by('bz_lq_min').value = b.lq_min;
  by('bz_t_en').checked = !!b.telem_en;
  by('bz_t_sec').value = b.telem_sec;
  by('bz_bb_en').checked = !!b.board_batt_en;
  by('bz_bb_min').value = b.board_batt_min;
  by('bz_db_en').checked = !!b.drone_batt_en;
  by('bz_db_min').value = b.drone_batt_min;

  fillSoundSelect('bz_u_snd', b.uplink_sound);
  fillSoundSelect('bz_d_snd', b.down_sound);
  fillSoundSelect('bz_lq_snd', b.lq_sound);
  fillSoundSelect('bz_t_snd', b.telem_sound);
  fillSoundSelect('bz_bb_snd', b.board_batt_sound);
  fillSoundSelect('bz_db_snd', b.drone_batt_sound);

  let list = '';
  for (const p of cfg.profiles){
    const isCurrent = p.bind_phrase === cfg.bind.bind_phrase && p.uid_text === cfg.bind.uid_text;
    const state = isCurrent ? '(current)' : '';
    list += `<div class="row"><span>#${p.index} ${h(p.note || '-')} ${state}</span><span><button onclick="applyProfile(${p.index})">Apply</button> <button onclick="deleteProfile(${p.index})">Delete</button></span></div>`;
  }
  by('profiles_list').innerHTML = list || 'No saved profiles';
}

async function loadSounds(){
  const r = await fetch('/api/sounds', {cache:'no-store'});
  const s = await r.json();
  sounds = Array.isArray(s.files) ? s.files : [];
}

async function loadLogs(){
  const r = await fetch('/api/logs', {cache:'no-store'});
  const j = await r.json();
  if (!Array.isArray(j.files) || j.files.length === 0) {
    by('logs_list').innerHTML = 'No logs';
    return;
  }
  by('logs_list').innerHTML = j.files.map(f=>`<div class="row"><span>${h(f.name)} (${f.size}B)</span><button onclick="viewLog('${encodeURIComponent(f.name)}')">View</button></div>`).join('');
}

async function viewLog(name){
  const r = await fetch(`/log?name=${name}`);
  by('log_preview').textContent = (await r.text()).slice(-6000);
}
window.viewLog = viewLog;

async function refreshTelemetry(){
  if (refreshTelemetry.busy) return;
  refreshTelemetry.busy = true;
  const controller = new AbortController();
  const timeout = setTimeout(()=>controller.abort(), 3000);
  try {
    const r = await fetch('/api/telemetry', {cache:'no-store', signal:controller.signal});
    if (!r.ok) throw Error('offline');
    drawTelemetry(await r.json());
  } catch(_) {
    by('state').textContent = 'Backpack disconnected — displayed values are outdated';
    by('state').className = 'warn';
    setStaleView(true);
    ['gps','link'].forEach(id=>by(id).classList.add('telemetry-stale'));
    const attitude = by('att').querySelector('.att-only');
    if (attitude) attitude.classList.add('telemetry-stale');
    by('tx_stream_status').textContent = 'Backpack disconnected';
    by('drone_status').textContent = 'Unknown';
  } finally { clearTimeout(timeout); refreshTelemetry.busy = false; }
}

by('wifi_form').onsubmit = async (event) => {
  event.preventDefault();
  const password = by('wifi_password').value;
  if (!/^[\x20-\x7e]{8,63}$/.test(password)) {
    by('wifi_msg').textContent = 'Use 8–63 printable ASCII characters.';
    return;
  }
  if (password !== by('wifi_confirm').value) {
    by('wifi_msg').textContent = 'Passwords do not match.';
    return;
  }
  by('wifi_save').disabled = true;
  try {
    const body = new URLSearchParams({password});
    const r = await fetch('/api/wifi_password', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
    const message = await r.text();
    if (!r.ok) throw Error(message || 'Save failed.');
    by('wifi_password').value = '';
    by('wifi_confirm').value = '';
    by('wifi_msg').textContent = message;
  } catch (error) { by('wifi_msg').textContent = error.message || 'Save failed.'; }
  finally { by('wifi_save').disabled = false; }
};

by('save_bind').onclick = async () => {
  const body = new URLSearchParams({bind_phrase: by('bind_phrase').value, uid_text: by('uid_text').value, note: by('bind_note').value});
  const r = await fetch('/api/bind', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
  by('bind_msg').textContent = await r.text();
  await loadConfig();
};
window.applyProfile = async (idx) => {
  const body = new URLSearchParams({index: String(idx)});
  const r = await fetch('/api/profile_apply', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
  by('bind_msg').textContent = await r.text();
  await loadConfig();
};
window.deleteProfile = async (idx) => {
  if (!confirm(`Delete profile #${idx}?`)) return;
  const body = new URLSearchParams({index: String(idx)});
  const r = await fetch('/api/profile_delete', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
  by('bind_msg').textContent = await r.text();
  await loadConfig();
};

by('save_log_cfg').onclick = async () => {
  const body = new URLSearchParams({
    enabled: by('log_enabled').checked ? '1' : '0',
    keep_files: by('log_keep').value,
    log_gps: by('log_gps').checked ? '1' : '0',
    log_link: by('log_link').checked ? '1' : '0',
    log_battery: by('log_battery').checked ? '1' : '0',
    log_attitude: by('log_attitude').checked ? '1' : '0',
    log_unknown: by('log_unknown').checked ? '1' : '0'
  });
  const r = await fetch('/api/log_cfg', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
  by('log_msg').textContent = await r.text();
  await loadConfig();
  await loadLogs();
};

by('save_bz_cfg').onclick = async () => {
  const body = new URLSearchParams({
    enabled: by('bz_enabled').checked ? '1' : '0',
    u_en: by('bz_u_en').checked ? '1' : '0',
    u_min: by('bz_u_min').value,
    u_snd: by('bz_u_snd').value,
    d_en: by('bz_d_en').checked ? '1' : '0',
    d_min: by('bz_d_min').value,
    d_snd: by('bz_d_snd').value,
    lq_en: by('bz_lq_en').checked ? '1' : '0',
    lq_min: by('bz_lq_min').value,
    lq_snd: by('bz_lq_snd').value,
    t_en: by('bz_t_en').checked ? '1' : '0',
    t_sec: by('bz_t_sec').value,
    t_snd: by('bz_t_snd').value,
    bb_en: by('bz_bb_en').checked ? '1' : '0',
    bb_min: by('bz_bb_min').value,
    bb_snd: by('bz_bb_snd').value,
    db_en: by('bz_db_en').checked ? '1' : '0',
    db_min: by('bz_db_min').value,
    db_snd: by('bz_db_snd').value
  });
  const r = await fetch('/api/buzzer_cfg', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'}, body});
  by('bz_msg').textContent = await r.text();
  await loadConfig();
};

by('upload_sound_btn').onclick = async () => {
  const fi = by('sound_file');
  if (!fi.files || !fi.files.length) {
    by('upload_msg').textContent = 'Choose file first';
    return;
  }
  const fd = new FormData();
  fd.append('file', fi.files[0], fi.files[0].name);
  const r = await fetch('/api/upload_sound', {method:'POST', headers:{'X-Backpack-Request':'1'}, body: fd});
  by('upload_msg').textContent = await r.text();
  await loadSounds();
  await loadConfig();
};

function renderGpsPoint() {
  const node = by('qr');
  node.innerHTML = '';
  if (gpsIdx < 0 || gpsIdx >= gpsPoints.length) {
    by('qr_meta').textContent = 'No points loaded';
    return;
  }
  const p = gpsPoints[gpsIdx];
  by('qr_meta').textContent = `${gpsIdx + 1}/${gpsPoints.length} | ${p.timestamp || p.uptime} | ${p.log}`;
  const txt = `https://maps.google.com/?q=${p.lat},${p.lon}`;
  if (typeof QRCode === 'function') {
    new QRCode(node, {text: txt, width: 180, height: 180});
  } else {
    const a = document.createElement('a');
    a.href = txt;
    a.textContent = txt;
    node.appendChild(a);
  }
}

by('btn_load_points').onclick = async () => {
  const r = await fetch('/api/gps_points?max=120', {cache:'no-store'});
  const j = await r.json();
  gpsPoints = Array.isArray(j.points) ? j.points : [];
  gpsIdx = gpsPoints.length ? 0 : -1;
  renderGpsPoint();
};
by('btn_prev_qr').onclick = () => {
  if (gpsIdx > 0) gpsIdx--;
  renderGpsPoint();
};
by('btn_next_qr').onclick = () => {
  if (gpsIdx >= 0 && gpsIdx < gpsPoints.length - 1) gpsIdx++;
  renderGpsPoint();
};

(async () => {
  try {
    await fetch('/api/time_sync', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Backpack-Request':'1'},
      body:new URLSearchParams({epoch: String(Math.floor(Date.now()/1000))})
    });
  } catch(_) {}
  try { await loadSounds(); await loadConfig(); await loadLogs(); }
  catch (_) { by('state').textContent = 'Backpack disconnected — retrying telemetry'; by('state').className = 'warn'; }
  await refreshTelemetry();
  setInterval(refreshTelemetry, 500);
})();
</script>
</main>
</body>
</html>
  )HTML";

  server.send(200, "text/html; charset=utf-8", HTML);
}

void handle_api_telemetry() {
  server.send(200, "application/json; charset=utf-8", telemetry_json());
}

void handle_api_config() {
  server.send(200, "application/json; charset=utf-8", config_json());
}

bool allow_write() {
  if (server.header("X-Backpack-Request") == "1") return true;
  server.send(403, "text/plain", "request denied");
  return false;
}

void handle_api_wifi_password() {
  if (!allow_write()) return;
  if (!server.hasArg("password")) { server.send(400, "text/plain", "Password is required."); return; }
  const String password = server.arg("password");
  if (password.length() < 8 || password.length() > 63) {
    server.send(400, "text/plain", "Use 8–63 printable ASCII characters."); return;
  }
  for (size_t i = 0; i < password.length(); ++i) {
    const unsigned char c = static_cast<unsigned char>(password[i]);
    if (c < 0x20 || c > 0x7e) { server.send(400, "text/plain", "Use printable ASCII characters."); return; }
  }
  if (!prefs.begin("backpack-ap", false)) { server.send(500, "text/plain", "Could not open password storage."); return; }
  const size_t written = prefs.putString("password", password);
  const bool saved = written > 0 && prefs.getString("password", "") == password;
  prefs.end();
  if (!saved) { server.send(500, "text/plain", "Could not save password."); return; }
  server.send(200, "text/plain", "Password saved. Restart Backpack to apply.");
}

void handle_api_time_sync() {
  if (!allow_write()) return;
  if (!server.hasArg("epoch")) {
    server.send(400, "text/plain", "missing epoch");
    return;
  }
  const long long epoch = atoll(server.arg("epoch").c_str());
  if (epoch < 946684800LL) {
    server.send(400, "text/plain", "bad epoch");
    return;
  }
  set_clock_base(static_cast<time_t>(epoch), TS_WEB, "web");
  server.send(200, "text/plain", "time synced");
}

void handle_api_bind() {
  if (!allow_write()) return;
  if (!server.hasArg("bind_phrase") || !server.hasArg("uid_text")) {
    server.send(400, "text/plain", "missing args");
    return;
  }

  config.bind_phrase = server.arg("bind_phrase");
  config.uid_text = server.arg("uid_text");
  const String note = server.hasArg("note") ? server.arg("note") : "";
  recalc_effective_uid();
  upsert_current_profile(note);
  save_config();
  schedule_espnow_restart();

  server.send(200, "text/plain", "Saved bind, profile upserted, UID applied.");
}

void handle_api_profile_save() {
  if (!allow_write()) return;
  if (!server.hasArg("index") || !server.hasArg("note") || !server.hasArg("bind_phrase") ||
      !server.hasArg("uid_text")) {
    server.send(400, "text/plain", "missing args");
    return;
  }

  const int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_BIND_PROFILES) {
    server.send(400, "text/plain", "bad index");
    return;
  }

  config.profiles[index].note = server.arg("note");
  config.profiles[index].bind_phrase = server.arg("bind_phrase");
  config.profiles[index].uid_text = server.arg("uid_text");
  save_config();

  server.send(200, "text/plain", "Profile saved.");
}

void handle_api_profile_delete() {
  if (!allow_write()) return;
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "missing index");
    return;
  }
  const int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_BIND_PROFILES) {
    server.send(400, "text/plain", "bad index");
    return;
  }

  config.profiles[index] = BindProfile{};
  save_config();
  server.send(200, "text/plain", "Profile deleted.");
}

void handle_api_profile_apply() {
  if (!allow_write()) return;
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "missing index");
    return;
  }

  const int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_BIND_PROFILES) {
    server.send(400, "text/plain", "bad index");
    return;
  }

  const BindProfile &p = config.profiles[index];
  if (p.bind_phrase.isEmpty() && p.uid_text.isEmpty()) {
    server.send(400, "text/plain", "profile is empty");
    return;
  }

  config.bind_phrase = p.bind_phrase;
  config.uid_text = p.uid_text;
  recalc_effective_uid();
  save_config();
  schedule_espnow_restart();

  server.send(200, "text/plain", "Profile applied.");
}

void handle_api_log_cfg() {
  if (!allow_write()) return;
  if (!server.hasArg("enabled") || !server.hasArg("keep_files") || !server.hasArg("log_gps") ||
      !server.hasArg("log_link") || !server.hasArg("log_battery") || !server.hasArg("log_attitude") ||
      !server.hasArg("log_unknown")) {
    server.send(400, "text/plain", "missing args");
    return;
  }

  config.log.enabled = (server.arg("enabled") == "1");
  const int keep = static_cast<int>(server.arg("keep_files").toInt());
  config.log.keep_files = static_cast<uint8_t>(keep < 1 ? 1 : (keep > 100 ? 100 : keep));
  config.log.log_gps = (server.arg("log_gps") == "1");
  config.log.log_link = (server.arg("log_link") == "1");
  config.log.log_battery = (server.arg("log_battery") == "1");
  config.log.log_attitude = (server.arg("log_attitude") == "1");
  config.log.log_unknown = (server.arg("log_unknown") == "1");

  save_config();
  rotate_logs_if_needed();
  server.send(200, "text/plain", "Log config saved.");
}

void handle_api_buzzer_cfg() {
  if (!allow_write()) return;
  if (!server.hasArg("enabled") || !server.hasArg("u_en") || !server.hasArg("u_min") ||
      !server.hasArg("u_snd") || !server.hasArg("d_en") || !server.hasArg("d_min") ||
      !server.hasArg("d_snd") || !server.hasArg("lq_en") || !server.hasArg("lq_min") ||
      !server.hasArg("lq_snd") || !server.hasArg("t_en") || !server.hasArg("t_sec") || !server.hasArg("t_snd") ||
      !server.hasArg("bb_en") || !server.hasArg("bb_min") || !server.hasArg("bb_snd") ||
      !server.hasArg("db_en") || !server.hasArg("db_min") || !server.hasArg("db_snd")) {
    server.send(400, "text/plain", "missing args");
    return;
  }

  config.buzzer.enabled = (server.arg("enabled") == "1");

  config.buzzer.alarm_uplink_rssi_enabled = (server.arg("u_en") == "1");
  config.buzzer.alarm_uplink_rssi_min_dbm = server.arg("u_min").toInt();
  config.buzzer.alarm_uplink_sound = server.arg("u_snd");

  config.buzzer.alarm_downlink_rssi_enabled = (server.arg("d_en") == "1");
  config.buzzer.alarm_downlink_rssi_min_dbm = server.arg("d_min").toInt();
  config.buzzer.alarm_downlink_sound = server.arg("d_snd");

  config.buzzer.alarm_lq_enabled = (server.arg("lq_en") == "1");
  config.buzzer.alarm_lq_min = server.arg("lq_min").toInt();
  config.buzzer.alarm_lq_sound = server.arg("lq_snd");

  config.buzzer.alarm_telem_loss_enabled = (server.arg("t_en") == "1");
  const int tsec = static_cast<int>(server.arg("t_sec").toInt());
  config.buzzer.alarm_telem_loss_sec = tsec < 1 ? 1 : tsec;
  config.buzzer.alarm_telem_loss_sound = server.arg("t_snd");
  config.buzzer.alarm_board_batt_enabled = (server.arg("bb_en") == "1");
  config.buzzer.alarm_board_batt_min_pct = server.arg("bb_min").toInt();
  config.buzzer.alarm_board_batt_sound = server.arg("bb_snd");
  config.buzzer.alarm_drone_batt_enabled = (server.arg("db_en") == "1");
  config.buzzer.alarm_drone_batt_min_pct = server.arg("db_min").toInt();
  config.buzzer.alarm_drone_batt_sound = server.arg("db_snd");

  save_config();
  server.send(200, "text/plain", "Buzzer config saved.");
}

void handle_api_logs() {
  server.send(200, "application/json; charset=utf-8", logs_json());
}

void handle_log_file() {
  if (!sd_available) {
    server.send(503, "text/plain", "sd not available");
    return;
  }
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "missing name");
    return;
  }

  String name = server.arg("name");
  if (!name.startsWith("/")) name = "/" + name;
  if (!is_safe_log_path(name)) {
    server.send(400, "text/plain", "bad path");
    return;
  }

  File f = SD.open(name.c_str(), FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "not found");
    return;
  }
  server.streamFile(f, "text/plain; charset=utf-8");
  f.close();
}

void handle_api_sounds() {
  server.send(200, "application/json; charset=utf-8", sounds_json());
}

bool upload_ok = false;
bool upload_permitted = false;
bool upload_exists = false;
String upload_path;
File upload_file;

void handle_api_upload_sound() {
  HTTPUpload &up = server.upload();
  if (!sd_available) return;

  if (up.status == UPLOAD_FILE_START) {
    upload_ok = false;
    upload_exists = false;
    upload_path = "";
    upload_permitted = server.header("X-Backpack-Request") == "1";
    if (!upload_permitted) return;
    String fn = up.filename;
    fn.replace("\\", "");
    fn.replace("/", "");
    if (!fn.endsWith(".wav") && !fn.endsWith(".WAV") && !fn.endsWith(".mp3") && !fn.endsWith(".MP3") &&
        !fn.endsWith(".ogg") && !fn.endsWith(".OGG")) {
      return;
    }
    String path = "/sound/" + fn;
    if (SD.exists(path.c_str())) { upload_exists = true; return; }
    upload_path = path;
    upload_file = SD.open(path.c_str(), FILE_WRITE);
    upload_ok = static_cast<bool>(upload_file);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (upload_file && upload_file.write(up.buf, up.currentSize) != up.currentSize) upload_ok = false;
  } else if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
    if (upload_file) upload_file.close();
    if (up.status == UPLOAD_FILE_ABORTED) upload_ok = false;
    if (!upload_ok && !upload_path.isEmpty()) SD.remove(upload_path.c_str());
  }
}

void handle_api_upload_sound_done() {
  if (!sd_available) {
    server.send(503, "text/plain", "sd not available");
    return;
  }
  if (!upload_permitted) { server.send(403, "text/plain", "request denied"); return; }
  if (upload_exists) { server.send(409, "text/plain", "sound already exists"); return; }
  if (!upload_ok) { server.send(400, "text/plain", "upload failed"); return; }
  server.send(200, "text/plain", "upload done");
}

void handle_api_gps_points() {
  server.send(200, "application/json; charset=utf-8", gps_points_json());
}

void handle_sound_file() {
  if (!sd_available) {
    server.send(503, "text/plain", "sd not available");
    return;
  }
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "missing file");
    return;
  }

  String name = server.arg("file");
  if (name.indexOf("..") >= 0 || name.indexOf('/') >= 0 || name.indexOf('\\') >= 0) {
    server.send(400, "text/plain", "bad file");
    return;
  }

  String path = "/sound/" + name;
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "sound not found");
    return;
  }

  String ct = "audio/wav";
  String lname = name;
  lname.toLowerCase();
  if (lname.endsWith(".mp3")) ct = "audio/mpeg";
  if (lname.endsWith(".ogg")) ct = "audio/ogg";

  server.streamFile(f, ct);
  f.close();
}

void handle_captive_redirect() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void handle_not_found() {
  handle_captive_redirect();
}

bool init_espnow_for_elrs() {
  if (!apply_uid_to_station_mac()) {
    Serial.println("failed to apply ELRS UID as STA MAC");
    return false;
  }
  if (ap_password.isEmpty() || !WiFi.softAP(AP_SSID, ap_password.c_str(), WIFI_CHANNEL)) {
    Serial.println("softAP start failed");
    return false;
  }

  dns_server.start(53, "*", WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    return false;
  }

  esp_now_register_recv_cb(on_espnow_recv);

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, config.uid, 6);
  peer_info.channel = WIFI_CHANNEL;
  peer_info.encrypt = false;
  const esp_err_t peer_err = esp_now_add_peer(&peer_info);
  if (peer_err != ESP_OK && peer_err != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("esp_now_add_peer failed: %d\n", static_cast<int>(peer_err));
    esp_now_deinit();
    return false;
  }

  espnow_ready = true;
  return true;
}

void setup_web_routes() {
  const char *headers[] = {"X-Backpack-Request"};
  server.collectHeaders(headers, 1);
  server.on("/qrcode.js", HTTP_GET, [](){ server.send_P(200, "application/javascript", QR_JS); });
  server.on("/", HTTP_GET, handle_root);
  server.on("/api/telemetry", HTTP_GET, handle_api_telemetry);
  server.on("/api/config", HTTP_GET, handle_api_config);
  server.on("/api/wifi_password", HTTP_POST, handle_api_wifi_password);
  server.on("/api/time_sync", HTTP_POST, handle_api_time_sync);
  server.on("/api/bind", HTTP_POST, handle_api_bind);
  server.on("/api/profile_save", HTTP_POST, handle_api_profile_save);
  server.on("/api/profile_apply", HTTP_POST, handle_api_profile_apply);
  server.on("/api/profile_delete", HTTP_POST, handle_api_profile_delete);
  server.on("/api/log_cfg", HTTP_POST, handle_api_log_cfg);
  server.on("/api/buzzer_cfg", HTTP_POST, handle_api_buzzer_cfg);
  server.on("/api/logs", HTTP_GET, handle_api_logs);
  server.on("/api/gps_points", HTTP_GET, handle_api_gps_points);
  server.on("/log", HTTP_GET, handle_log_file);
  server.on("/api/sounds", HTTP_GET, handle_api_sounds);
  server.on("/api/upload_sound", HTTP_POST, handle_api_upload_sound_done, handle_api_upload_sound);
  server.on("/sound", HTTP_GET, handle_sound_file);

  server.on("/generate_204", HTTP_GET, handle_captive_redirect);
  server.on("/gen_204", HTTP_GET, handle_captive_redirect);
  server.on("/hotspot-detect.html", HTTP_GET, handle_captive_redirect);
  server.on("/ncsi.txt", HTTP_GET, handle_captive_redirect);
  server.on("/connecttest.txt", HTTP_GET, handle_captive_redirect);
  server.on("/fwlink", HTTP_GET, handle_captive_redirect);

  server.onNotFound(handle_not_found);
  server.begin();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  reset_button_raw = reset_button_stable = digitalRead(RESET_BUTTON_PIN) == LOW;
  reset_button_changed_ms = millis();
  analogReadResolution(12);
  pinMode(BOARD_BATTERY_ADC_PIN, INPUT);

  load_config();
  ap_password = DEFAULT_AP_PASSWORD;
  if (prefs.begin("backpack-ap", false)) {
    const String saved_password = prefs.getString("password", "");
    if (saved_password.length() >= 8 && saved_password.length() <= 63) ap_password = saved_password;
    prefs.end();
  } else {
    ap_password = "";
    Serial.println("AP password storage failed; Wi-Fi disabled");
  }
  ensure_sd_card();

  Serial.println();
  Serial.println("ELRS Backpack panel: multimedia branch");

  if (!init_espnow_for_elrs()) {
    Serial.println("ESP-NOW init failed, running web only");
  }

  setup_web_routes();

  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP URL: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("ELRS UID: ");
  Serial.println(uid_to_text(config.uid));
  Serial.print("SD status: ");
  Serial.println(sd_status);
}

void loop() {
  process_espnow_queue();
  update_board_battery();
  update_logging_state();
  check_alarms();
  audio_poll();
  update_reset_button();
  update_status_led();

  dns_server.processNextRequest();
  server.handleClient();
  service_pending_espnow_restart();
}
