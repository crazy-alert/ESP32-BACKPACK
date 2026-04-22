#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/md5.h>

namespace {
constexpr char AP_SSID[] = "ELRS-Backpack";
constexpr char AP_PASSWORD[] = "12345678";
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t CRSF_MAX_FRAME_LEN = 64;
constexpr uint8_t ESPNOW_MAX_PAYLOAD = 250;
constexpr uint8_t MAX_BIND_PROFILES = 8;

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

struct TelemetryData {
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

  float board_voltage = 0.0f;
  int board_percent = 0;

  uint32_t frames_total = 0;
  uint32_t crc_errors = 0;
  uint32_t unknown_frames = 0;
  uint32_t last_frame_ms = 0;

  uint32_t espnow_packets = 0;
  uint32_t espnow_drop_short = 0;
  uint32_t espnow_drop_long = 0;
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

volatile bool espnow_frame_ready = false;
volatile uint8_t espnow_frame_len = 0;
uint8_t espnow_frame[ESPNOW_MAX_PAYLOAD] = {0};

bool sd_available = false;
String sd_status = "not initialized";
File active_log;
String active_log_name;
unsigned long last_log_flush_ms = 0;

bool last_alarm_uplink = false;
bool last_alarm_downlink = false;
bool last_alarm_lq = false;
bool last_alarm_telem = false;
uint32_t alarm_event_id = 0;
String last_alarm_name;
String last_alarm_sound;
unsigned long last_alarm_ms = 0;

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
  Serial.printf("[CRSF] type=0x%02X payload=%u\n", type, payload_len);
  Serial.printf("       data=%s\n", bytes_to_hex(payload, payload_len).c_str());
}

void update_armed_state() {
  String m = telemetry.flight_mode;
  m.toUpperCase();
  telemetry.is_armed = (m.indexOf("ARM") >= 0 && m.indexOf("DISARM") < 0);
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
  if (!config.log.enabled || !telemetry.is_armed || !sd_available || active_log) {
    return;
  }

  active_log_name = build_new_log_name();
  active_log = SD.open(active_log_name.c_str(), FILE_WRITE);
  if (!active_log) {
    sd_status = "log open failed";
    active_log_name = "";
    return;
  }

  active_log.println("ms,type,payload_hex,lat,lon,gps_fix,gps_sats,bat_v,bat_a,lq,rssi1,rssi2,down_rssi,down_lq,down_snr,mode");
  active_log.flush();
  rotate_logs_if_needed();
  sd_status = "logging";
}

void log_frame_csv(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
  if (!active_log || !should_log_type(type)) return;

  active_log.print(millis());
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
  active_log.println(telemetry.flight_mode);

  if (millis() - last_log_flush_ms > 1000) {
    active_log.flush();
    last_log_flush_ms = millis();
  }
}

void update_logging_state() {
  if (!telemetry.is_armed || !config.log.enabled || !sd_available) {
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
}

void check_alarms() {
  const bool uplink_bad = config.buzzer.alarm_uplink_rssi_enabled &&
                          telemetry.link_rssi_1_dbm < config.buzzer.alarm_uplink_rssi_min_dbm;
  const bool downlink_bad = config.buzzer.alarm_downlink_rssi_enabled &&
                            telemetry.link_down_rssi_dbm < config.buzzer.alarm_downlink_rssi_min_dbm;
  const bool lq_bad = config.buzzer.alarm_lq_enabled &&
                      telemetry.link_quality < config.buzzer.alarm_lq_min;
  const bool telem_bad = config.buzzer.alarm_telem_loss_enabled &&
                         (millis() - telemetry.last_frame_ms) >
                             static_cast<unsigned long>(config.buzzer.alarm_telem_loss_sec) * 1000UL;

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

  last_alarm_uplink = uplink_bad;
  last_alarm_downlink = downlink_bad;
  last_alarm_lq = lq_bad;
  last_alarm_telem = telem_bad;
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

  telemetry.frames_total++;
  telemetry.last_frame_ms = millis();
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
      break;
    }
    case 0x21: {
      if (payload_len == 0) break;
      char buf[20] = {0};
      const size_t copy_len = min(static_cast<size_t>(payload_len), sizeof(buf) - 1);
      memcpy(buf, payload, copy_len);
      telemetry.flight_mode = String(buf);
      update_armed_state();
      break;
    }
    case 0x07: {
      if (payload_len < 2) break;
      const int16_t vario_cms = static_cast<int16_t>(read_be16(payload));
      telemetry.vario_ms = static_cast<float>(vario_cms) / 100.0f;
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
  telemetry.espnow_packets++;

  if (len <= 0) {
    telemetry.espnow_drop_short++;
    return;
  }

  if (len > ESPNOW_MAX_PAYLOAD) {
    telemetry.espnow_drop_long++;
    return;
  }

  memcpy(espnow_frame, incoming_data, len);
  espnow_frame_len = static_cast<uint8_t>(len);
  espnow_frame_ready = true;
}

void process_espnow_queue() {
  if (!espnow_frame_ready) {
    return;
  }

  noInterrupts();
  const uint8_t len = espnow_frame_len;
  uint8_t local[ESPNOW_MAX_PAYLOAD] = {0};
  memcpy(local, espnow_frame, len);
  espnow_frame_ready = false;
  interrupts();

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

void update_board_battery() {
  static unsigned long last = 0;
  if (millis() - last < 500) {
    return;
  }
  last = millis();

  const int mv = analogReadMilliVolts(BOARD_BATTERY_ADC_PIN);
  const float v = (static_cast<float>(mv) / 1000.0f) * BOARD_BATTERY_DIVIDER_RATIO;
  telemetry.board_voltage = v;

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
  out += "\"frames_total\":" + String(telemetry.frames_total) + ",";
  out += "\"crc_errors\":" + String(telemetry.crc_errors) + ",";
  out += "\"unknown_frames\":" + String(telemetry.unknown_frames) + ",";

  out += "\"espnow\":{";
  out += "\"packets\":" + String(telemetry.espnow_packets) + ",";
  out += "\"drop_short\":" + String(telemetry.espnow_drop_short) + ",";
  out += "\"drop_long\":" + String(telemetry.espnow_drop_long) + ",";
  out += "\"no_crsf\":" + String(telemetry.espnow_no_crsf);
  out += "},";

  out += "\"wifi\":{";
  out += "\"sta_status\":\"" + wifi_status_text() + "\",";
  out += "\"sta_ip\":\"" + WiFi.localIP().toString() + "\",";
  out += "\"note\":\"" + json_escape(wifi_note) + "\"";
  out += "},";

  out += "\"sd\":{";
  out += "\"available\":" + String(sd_available ? "true" : "false") + ",";
  out += "\"status\":\"" + json_escape(sd_status) + "\",";
  out += "\"active_log\":\"" + json_escape(active_log_name) + "\"";
  out += "},";

  out += "\"board_battery\":{";
  out += "\"voltage\":" + String(telemetry.board_voltage, 2) + ",";
  out += "\"percent\":" + String(telemetry.board_percent);
  out += "},";

  out += "\"gps\":{";
  out += "\"fix\":" + String(telemetry.gps_fix ? "true" : "false") + ",";
  out += "\"sats\":" + String(telemetry.gps_sats) + ",";
  out += "\"lat\":" + String(telemetry.gps_lat, 7) + ",";
  out += "\"lon\":" + String(telemetry.gps_lon, 7) + ",";
  out += "\"speed_kmh\":" + String(telemetry.gps_speed_kmh, 1) + ",";
  out += "\"heading_deg\":" + String(telemetry.gps_heading_deg, 1) + ",";
  out += "\"altitude_m\":" + String(telemetry.gps_altitude_m);
  out += "},";

  out += "\"battery\":{";
  out += "\"voltage\":" + String(telemetry.battery_voltage, 2) + ",";
  out += "\"current\":" + String(telemetry.battery_current, 2) + ",";
  out += "\"capacity_mah\":" + String(telemetry.battery_capacity_mah) + ",";
  out += "\"remaining_pct\":" + String(telemetry.battery_remaining_pct);
  out += "},";

  out += "\"link\":{";
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
  out += "\"pitch_deg\":" + String(telemetry.attitude_pitch_deg, 2) + ",";
  out += "\"roll_deg\":" + String(telemetry.attitude_roll_deg, 2) + ",";
  out += "\"yaw_deg\":" + String(telemetry.attitude_yaw_deg, 2) + ",";
  out += "\"vario_ms\":" + String(telemetry.vario_ms, 2);
  out += "},";

  out += "\"flight_mode\":\"" + json_escape(telemetry.flight_mode) + "\",";
  out += "\"is_armed\":" + String(telemetry.is_armed ? "true" : "false") + ",";

  out += "\"alarm\":{";
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

  out += "\"bind\":{";
  out += "\"bind_phrase\":\"" + json_escape(config.bind_phrase) + "\",";
  out += "\"uid_text\":\"" + json_escape(config.uid_text) + "\",";
  out += "\"effective_uid\":\"" + json_escape(uid_to_text(config.uid)) + "\"";
  out += "},";

  out += "\"profiles\":[";
  for (int i = 0; i < MAX_BIND_PROFILES; i++) {
    if (i) out += ",";
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
  out += "\"telem_sound\":\"" + json_escape(config.buzzer.alarm_telem_loss_sound) + "\"";
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

void apply_uid_to_station_mac() {
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_mac(WIFI_IF_STA, config.uid);
}

void handle_root() {
  static const char HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ELRS Backpack Panel</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/qrcodejs/1.0.0/qrcode.min.js"></script>
  <style>
    :root{--bg:#0b1220;--panel:#111827;--line:#334155;--txt:#e5e7eb;--muted:#94a3b8;--ok:#34d399}
    *{box-sizing:border-box} body{margin:0;padding:16px;color:var(--txt);font-family:Segoe UI,sans-serif;background:radial-gradient(circle at top,#1e293b,var(--bg))}
    h1{margin:0 0 8px} .tabs{display:flex;gap:8px;margin:10px 0 14px;flex-wrap:wrap}.tab{padding:8px 12px;border:1px solid var(--line);border-radius:10px;cursor:pointer}
    .tab.active{background:#1f2937} .pane{display:none}.pane.active{display:block}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px;margin:8px 0}
    .row{display:flex;justify-content:space-between;margin:6px 0;gap:8px;align-items:center}.k{color:var(--muted)} .v{font-weight:700}
    input,select{width:100%;padding:10px;border-radius:8px;border:1px solid var(--line);background:#0f172a;color:var(--txt)}
    button{padding:10px 14px;border-radius:8px;border:1px solid var(--line);background:#1f2937;color:var(--txt);cursor:pointer}
    .ok{color:var(--ok)} .muted{color:var(--muted);font-size:12px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}
    code{white-space:pre-wrap}
  </style>
</head>
<body>
  <h1>ELRS Backpack Panel</h1>
  <div class="muted">API: /api/telemetry /api/config /api/logs /api/sounds</div>

  <div class="tabs">
    <div class="tab active" data-tab="telemetry">Telemetry</div>
    <div class="tab" data-tab="bind">Bind</div>
    <div class="tab" data-tab="logging">Logging</div>
    <div class="tab" data-tab="buzzer">Buzzer</div>
  </div>

  <div id="telemetry" class="pane active">
    <div id="state" class="ok">Waiting telemetry...</div>
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
      <div class="row"><span class="k">Bind Phrase</span></div>
      <input id="bind_phrase" placeholder="your bind phrase" />
      <div class="row"><span class="k">UID Digits (6 numbers)</span></div>
      <input id="uid_text" placeholder="106,19,19,206,193,30" />
      <div class="row"><button id="save_bind">Save Current Bind</button></div>
      <div class="row"><span class="k">Effective UID</span><span id="effective_uid" class="v">-</span></div>
      <div id="bind_msg" class="muted"></div>
    </div>

    <div class="card">
      <h3>Saved Profiles</h3>
      <div class="muted">Each profile stores note + bind phrase + uid digits.</div>
      <div class="row">
        <select id="profile_index"></select>
        <button id="load_profile">Load</button>
      </div>
      <div class="row"><span class="k">Note</span></div>
      <input id="profile_note" placeholder="TX #1" />
      <div class="row"><span class="k">Profile Bind Phrase</span></div>
      <input id="profile_phrase" placeholder="bind phrase" />
      <div class="row"><span class="k">Profile UID Digits</span></div>
      <input id="profile_uid" placeholder="106,19,19,206,193,30" />
      <div class="row">
        <button id="save_profile">Save Profile</button>
        <button id="apply_profile">Apply Profile</button>
      </div>
      <div id="profile_msg" class="muted"></div>
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
      <h3>Last GPS QR</h3>
      <div class="muted">Generated from last telemetry GPS point.</div>
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

      <div class="row"><button id="save_bz_cfg">Save Buzzer Config</button></div>
      <div id="bz_msg" class="muted"></div>
    </div>
  </div>

<script>
const by = id => document.getElementById(id);
const f = (n, d = 1) => Number.isFinite(n) ? Number(n).toFixed(d) : "-";
let cfg = null;
let sounds = [];
let lastAlarmId = 0;

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
      html += `<div class="muted">${u.last_hex}</div>`;
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
  new QRCode(node, {text: txt, width: 160, height: 160});
}

function maybePlayAlarm(t){
  if (!t.alarm) return;
  if (t.alarm.event_id <= lastAlarmId) return;
  lastAlarmId = t.alarm.event_id;
  if (!t.alarm.sound) return;
  const a = new Audio(`/sound?file=${encodeURIComponent(t.alarm.sound)}`);
  a.play().catch(()=>{});
}

function drawTelemetry(t){
  by('state').textContent = t.frames_total > 0 ? 'Telemetry active' : 'No telemetry yet';
  by('gps').innerHTML = `<h3>GPS</h3><div class="row"><span class="k">Fix</span><span class="v">${t.gps.fix}</span></div><div class="row"><span class="k">Sats</span><span class="v">${t.gps.sats}</span></div><div class="row"><span class="k">Lat</span><span class="v">${f(t.gps.lat,7)}</span></div><div class="row"><span class="k">Lon</span><span class="v">${f(t.gps.lon,7)}</span></div><div class="row"><span class="k">Speed</span><span class="v">${f(t.gps.speed_kmh)} km/h</span></div>`;
  by('bat').innerHTML = `<h3>Battery</h3><div class="row"><span class="k">FC Voltage</span><span class="v">${f(t.battery.voltage,2)} V</span></div><div class="row"><span class="k">FC Current</span><span class="v">${f(t.battery.current,2)} A</span></div><div class="row"><span class="k">FC Remain</span><span class="v">${t.battery.remaining_pct} %</span></div><hr><div class="row"><span class="k">Board Voltage</span><span class="v">${f(t.board_battery.voltage,2)} V</span></div><div class="row"><span class="k">Board Percent</span><span class="v">${t.board_battery.percent} %</span></div>`;
  by('link').innerHTML = `<h3>Link</h3><div class="row"><span class="k">RSSI1</span><span class="v">${t.link.rssi1_dbm} dBm</span></div><div class="row"><span class="k">RSSI2</span><span class="v">${t.link.rssi2_dbm} dBm</span></div><div class="row"><span class="k">LQ</span><span class="v">${t.link.lq}</span></div><div class="row"><span class="k">SNR</span><span class="v">${t.link.snr_db} dB</span></div><div class="row"><span class="k">Active Ant</span><span class="v">${t.link.active_antenna}</span></div><div class="row"><span class="k">Down RSSI</span><span class="v">${t.link.down_rssi_dbm} dBm</span></div><div class="row"><span class="k">Down LQ</span><span class="v">${t.link.down_lq}</span></div><div class="row"><span class="k">Down SNR</span><span class="v">${t.link.down_snr_db} dB</span></div>`;
  by('att').innerHTML = `<h3>Attitude</h3><div class="row"><span class="k">Pitch</span><span class="v">${f(t.attitude.pitch_deg,2)}</span></div><div class="row"><span class="k">Roll</span><span class="v">${f(t.attitude.roll_deg,2)}</span></div><div class="row"><span class="k">Yaw</span><span class="v">${f(t.attitude.yaw_deg,2)}</span></div><div class="row"><span class="k">Mode</span><span class="v">${t.flight_mode}</span></div><div class="row"><span class="k">Armed</span><span class="v">${t.is_armed}</span></div>`;
  by('sys').innerHTML = `<h3>System</h3><div class="row"><span class="k">Frames</span><span class="v">${t.frames_total}</span></div><div class="row"><span class="k">ESPNow</span><span class="v">${t.espnow.packets}</span></div><div class="row"><span class="k">No CRSF</span><span class="v">${t.espnow.no_crsf}</span></div><div class="row"><span class="k">SD</span><span class="v">${t.sd.available}</span></div><div class="row"><span class="k">SD Status</span><span class="v">${t.sd.status}</span></div><div class="row"><span class="k">Active log</span><span class="v">${t.sd.active_log}</span></div>`;
  by('sd_status').textContent = `SD: ${t.sd.status}`;
  drawUnknown(t.unknown_types);
  drawQR(t.gps);
  maybePlayAlarm(t);
}

async function loadConfig(){
  const r = await fetch('/api/config', {cache:'no-store'});
  cfg = await r.json();

  by('bind_phrase').value = cfg.bind.bind_phrase || '';
  by('uid_text').value = cfg.bind.uid_text || '';
  by('effective_uid').textContent = cfg.bind.effective_uid || '-';

  const pi = by('profile_index');
  pi.innerHTML = '';
  for (const p of cfg.profiles){
    const o = document.createElement('option');
    o.value = p.index;
    o.textContent = `#${p.index} ${p.note || '(empty)'}`;
    pi.appendChild(o);
  }
  loadProfileToForm();

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

  fillSoundSelect('bz_u_snd', b.uplink_sound);
  fillSoundSelect('bz_d_snd', b.down_sound);
  fillSoundSelect('bz_lq_snd', b.lq_sound);
  fillSoundSelect('bz_t_snd', b.telem_sound);

  let list = '';
  for (const p of cfg.profiles){
    if (p.note || p.bind_phrase || p.uid_text) {
      const diff = (p.bind_phrase !== cfg.bind.bind_phrase || p.uid_text !== cfg.bind.uid_text) ? ' (can apply)' : ' (current)';
      list += `#${p.index} ${p.note || '-'}${diff}<br>`;
    }
  }
  by('profiles_list').innerHTML = list || 'No saved profiles';
}

function loadProfileToForm(){
  if (!cfg) return;
  const idx = Number(by('profile_index').value || 0);
  const p = cfg.profiles.find(x => x.index === idx);
  if (!p) return;
  by('profile_note').value = p.note || '';
  by('profile_phrase').value = p.bind_phrase || '';
  by('profile_uid').value = p.uid_text || '';
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
  by('logs_list').innerHTML = j.files.map(f=>`<div class="row"><span>${f.name} (${f.size}B)</span><button onclick="viewLog('${f.name}')">View</button></div>`).join('');
}

async function viewLog(name){
  const r = await fetch(`/log?name=${encodeURIComponent(name)}`);
  by('log_preview').textContent = (await r.text()).slice(-6000);
}
window.viewLog = viewLog;

async function refreshTelemetry(){
  try {
    const r = await fetch('/api/telemetry', {cache:'no-store'});
    if (!r.ok) return;
    drawTelemetry(await r.json());
  } catch(_) {}
}

by('save_bind').onclick = async () => {
  const body = new URLSearchParams({bind_phrase: by('bind_phrase').value, uid_text: by('uid_text').value});
  const r = await fetch('/api/bind', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
  by('bind_msg').textContent = await r.text();
  await loadConfig();
};

by('load_profile').onclick = () => loadProfileToForm();

by('save_profile').onclick = async () => {
  const body = new URLSearchParams({
    index: by('profile_index').value,
    note: by('profile_note').value,
    bind_phrase: by('profile_phrase').value,
    uid_text: by('profile_uid').value
  });
  const r = await fetch('/api/profile_save', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
  by('profile_msg').textContent = await r.text();
  await loadConfig();
};

by('apply_profile').onclick = async () => {
  const body = new URLSearchParams({index: by('profile_index').value});
  const r = await fetch('/api/profile_apply', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
  by('profile_msg').textContent = await r.text();
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
  const r = await fetch('/api/log_cfg', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
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
    t_snd: by('bz_t_snd').value
  });
  const r = await fetch('/api/buzzer_cfg', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
  by('bz_msg').textContent = await r.text();
  await loadConfig();
};

(async () => {
  await loadSounds();
  await loadConfig();
  await loadLogs();
  await refreshTelemetry();
  setInterval(refreshTelemetry, 500);
})();
</script>
</body>
</html>
  )HTML";

  server.send(200, "text/html; charset=utf-8", HTML);
}

void handle_api_telemetry() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=utf-8", telemetry_json());
}

void handle_api_config() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=utf-8", config_json());
}

void handle_api_bind() {
  if (!server.hasArg("bind_phrase") || !server.hasArg("uid_text")) {
    server.send(400, "text/plain", "missing args");
    return;
  }

  config.bind_phrase = server.arg("bind_phrase");
  config.uid_text = server.arg("uid_text");
  recalc_effective_uid();
  save_config();
  apply_uid_to_station_mac();

  server.send(200, "text/plain", "Saved current bind and applied UID.");
}

void handle_api_profile_save() {
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

void handle_api_profile_apply() {
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
  apply_uid_to_station_mac();

  server.send(200, "text/plain", "Profile applied.");
}

void handle_api_log_cfg() {
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
  if (!server.hasArg("enabled") || !server.hasArg("u_en") || !server.hasArg("u_min") ||
      !server.hasArg("u_snd") || !server.hasArg("d_en") || !server.hasArg("d_min") ||
      !server.hasArg("d_snd") || !server.hasArg("lq_en") || !server.hasArg("lq_min") ||
      !server.hasArg("lq_snd") || !server.hasArg("t_en") || !server.hasArg("t_sec") ||
      !server.hasArg("t_snd")) {
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

  save_config();
  server.send(200, "text/plain", "Buzzer config saved.");
}

void handle_api_logs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
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
  if (!name.startsWith("/logs/")) {
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
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=utf-8", sounds_json());
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
  apply_uid_to_station_mac();
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);

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
  esp_now_add_peer(&peer_info);

  return true;
}

void setup_web_routes() {
  server.on("/", HTTP_GET, handle_root);
  server.on("/api/telemetry", HTTP_GET, handle_api_telemetry);
  server.on("/api/config", HTTP_GET, handle_api_config);
  server.on("/api/bind", HTTP_POST, handle_api_bind);
  server.on("/api/profile_save", HTTP_POST, handle_api_profile_save);
  server.on("/api/profile_apply", HTTP_POST, handle_api_profile_apply);
  server.on("/api/log_cfg", HTTP_POST, handle_api_log_cfg);
  server.on("/api/buzzer_cfg", HTTP_POST, handle_api_buzzer_cfg);
  server.on("/api/logs", HTTP_GET, handle_api_logs);
  server.on("/log", HTTP_GET, handle_log_file);
  server.on("/api/sounds", HTTP_GET, handle_api_sounds);
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
  analogReadResolution(12);
  pinMode(BOARD_BATTERY_ADC_PIN, INPUT);

  load_config();
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

  dns_server.processNextRequest();
  server.handleClient();
}
