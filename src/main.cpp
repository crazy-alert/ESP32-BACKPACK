#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
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
  int link_rf_mode = 0;
  int link_tx_power = 0;
  int link_active_antenna = 0;
  int link_down_rssi_dbm = 0;
  int link_down_quality = 0;
  int link_down_snr_db = 0;

  float attitude_pitch_deg = 0.0f;
  float attitude_roll_deg = 0.0f;
  float attitude_yaw_deg = 0.0f;
  float vario_ms = 0.0f;
  String flight_mode = "N/A";

  uint32_t frames_total = 0;
  uint32_t crc_errors = 0;
  uint32_t unknown_frames = 0;
  uint32_t last_frame_ms = 0;

  uint32_t espnow_packets = 0;
  uint32_t espnow_drop_short = 0;
  uint32_t espnow_drop_long = 0;
  uint32_t espnow_no_crsf = 0;
};

struct AppConfig {
  String bind_phrase;
  String uid_text;
  uint8_t uid[6] = {106, 19, 19, 206, 193, 30};
};

struct FrameStat {
  uint32_t count = 0;
  uint8_t last_payload_len = 0;
  String last_payload_hex = "";
};

TelemetryData telemetry;
AppConfig config;
FrameStat frame_stats[256];
WebServer server(80);
DNSServer dns_server;
Preferences prefs;

volatile bool espnow_frame_ready = false;
volatile uint8_t espnow_frame_len = 0;
uint8_t espnow_frame[ESPNOW_MAX_PAYLOAD] = {0};

String wifi_note = "home wifi disabled";

String bytes_to_hex(const uint8_t *data, uint8_t len) {
  String out;
  out.reserve(len * 2 + len);
  for (uint8_t i = 0; i < len; i++) {
    if (i) out += ' ';
    if (data[i] < 16) out += '0';
    out += String(data[i], HEX);
  }
  out.toUpperCase();
  return out;
}

void serial_log_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
  Serial.printf("[CRSF] type=0x%02X payload=%u\n", type, payload_len);
  Serial.printf("       data=%s\n", bytes_to_hex(payload, payload_len).c_str());
}

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
  out_uid[0] &= static_cast<uint8_t>(~0x01);  // force unicast MAC
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
  out_uid[0] &= static_cast<uint8_t>(~0x01);  // force unicast MAC
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
  prefs.end();
  recalc_effective_uid();
}

void save_config() {
  prefs.begin("cfg", false);
  prefs.putString("bind_phrase", config.bind_phrase);
  prefs.putString("uid_text", config.uid_text);
  prefs.end();
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
      if (payload_len < 15) return;
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
      if (payload_len < 8) return;
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
      if (payload_len < 10) return;
      // CRSF link stats RSSI fields are signed dBm values.
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
      if (payload_len < 6) return;
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
      if (payload_len == 0) return;
      char buf[20] = {0};
      const size_t copy_len = min(static_cast<size_t>(payload_len), sizeof(buf) - 1);
      memcpy(buf, payload, copy_len);
      telemetry.flight_mode = String(buf);
      break;
    }
    case 0x07: {
      if (payload_len < 2) return;
      const int16_t vario_cms = static_cast<int16_t>(read_be16(payload));
      telemetry.vario_ms = static_cast<float>(vario_cms) / 100.0f;
      break;
    }
    default:
      telemetry.unknown_frames++;
      break;
  }
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
  out.reserve(4096);
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
  out += "\"note\":\"" + wifi_note + "\"";
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
  out += "\"flight_mode\":\"" + telemetry.flight_mode + "\"";
  out += ",";
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
    out += "\"last_hex\":\"" + frame_stats[i].last_payload_hex + "\"";
    out += "}";
  }
  out += "]";
  out += "}";
  return out;
}

String config_json() {
  String out;
  out.reserve(512);
  out += "{";
  out += "\"bind_phrase\":\"" + config.bind_phrase + "\",";
  out += "\"uid_text\":\"" + config.uid_text + "\",";
  out += "\"effective_uid\":\"" + uid_to_text(config.uid) + "\",";
  out += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"";
  out += "}";
  return out;
}

void handle_root() {
  static const char HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ELRS Backpack Panel</title>
  <style>
    :root{--bg:#0b1220;--panel:#111827;--line:#334155;--txt:#e5e7eb;--muted:#94a3b8;--ok:#34d399}
    *{box-sizing:border-box} body{margin:0;padding:16px;color:var(--txt);font-family:Segoe UI,sans-serif;background:radial-gradient(circle at top,#1e293b,var(--bg))}
    h1{margin:0 0 8px} .tabs{display:flex;gap:8px;margin:10px 0 14px}.tab{padding:8px 12px;border:1px solid var(--line);border-radius:10px;cursor:pointer}
    .tab.active{background:#1f2937} .pane{display:none}.pane.active{display:block}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px;margin:8px 0}
    .row{display:flex;justify-content:space-between;margin:6px 0;gap:8px}.k{color:var(--muted)} .v{font-weight:700}
    input{width:100%;padding:10px;border-radius:8px;border:1px solid var(--line);background:#0f172a;color:var(--txt)}
    button{padding:10px 14px;border-radius:8px;border:1px solid var(--line);background:#1f2937;color:var(--txt);cursor:pointer}
    .ok{color:var(--ok)} .muted{color:var(--muted);font-size:12px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}
  </style>
</head>
<body>
  <h1>ELRS Backpack Panel</h1>
  <div class="muted">API: /api/telemetry, /api/config</div>

  <div class="tabs">
    <div class="tab active" data-tab="telemetry">Telemetry</div>
    <div class="tab" data-tab="bind">Bind</div>
  </div>

  <div id="telemetry" class="pane active">
    <div id="state" class="ok">Waiting telemetry...</div>
    <div class="grid">
      <div class="card" id="gps"></div>
      <div class="card" id="bat"></div>
      <div class="card" id="link"></div>
      <div class="card" id="att"></div>
      <div class="card" id="sys"></div>
      <div class="card" id="all"></div>
    </div>
  </div>

  <div id="bind" class="pane">
    <div class="card">
      <div class="row"><span class="k">Bind Phrase</span></div>
      <input id="bind_phrase" placeholder="your bind phrase" />
      <div class="muted">If UID digits are empty, UID will be calculated from phrase.</div>
      <div class="row"><span class="k">UID Digits (6 numbers)</span></div>
      <input id="uid_text" placeholder="106,19,19,206,193,30" />
      <div class="muted">Example: 106,19,19,206,193,30</div>
      <div class="row"><button id="save_bind">Save Bind</button></div>
      <div class="row"><span class="k">Effective UID</span><span id="effective_uid" class="v">-</span></div>
      <div id="bind_msg" class="muted"></div>
    </div>
  </div>

  <script>
    const by = id => document.getElementById(id);
    const f = (n, d = 1) => Number.isFinite(n) ? Number(n).toFixed(d) : "-";

    function setTab(name){
      document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active', t.dataset.tab===name));
      document.querySelectorAll('.pane').forEach(p=>p.classList.toggle('active', p.id===name));
    }
    document.querySelectorAll('.tab').forEach(t=>t.onclick=()=>setTab(t.dataset.tab));

    function drawTelemetry(t){
      by('state').textContent = t.frames_total > 0 ? 'Telemetry active' : 'No telemetry yet';
      by('gps').innerHTML = `<h3>GPS</h3><div class="row"><span class="k">Fix</span><span class="v">${t.gps.fix}</span></div><div class="row"><span class="k">Sats</span><span class="v">${t.gps.sats}</span></div><div class="row"><span class="k">Lat</span><span class="v">${f(t.gps.lat,7)}</span></div><div class="row"><span class="k">Lon</span><span class="v">${f(t.gps.lon,7)}</span></div><div class="row"><span class="k">Speed</span><span class="v">${f(t.gps.speed_kmh)} km/h</span></div>`;
      by('bat').innerHTML = `<h3>Battery</h3><div class="row"><span class="k">Voltage</span><span class="v">${f(t.battery.voltage,2)} V</span></div><div class="row"><span class="k">Current</span><span class="v">${f(t.battery.current,2)} A</span></div><div class="row"><span class="k">Remain</span><span class="v">${t.battery.remaining_pct} %</span></div>`;
      by('link').innerHTML = `<h3>Link</h3><div class="row"><span class="k">RSSI1</span><span class="v">${t.link.rssi1_dbm} dBm</span></div><div class="row"><span class="k">RSSI2</span><span class="v">${t.link.rssi2_dbm} dBm</span></div><div class="row"><span class="k">LQ</span><span class="v">${t.link.lq}</span></div><div class="row"><span class="k">SNR</span><span class="v">${t.link.snr_db} dB</span></div><div class="row"><span class="k">Active Ant</span><span class="v">${t.link.active_antenna}</span></div><div class="row"><span class="k">Down RSSI</span><span class="v">${t.link.down_rssi_dbm} dBm</span></div><div class="row"><span class="k">Down LQ</span><span class="v">${t.link.down_lq}</span></div><div class="row"><span class="k">Down SNR</span><span class="v">${t.link.down_snr_db} dB</span></div>`;
      by('att').innerHTML = `<h3>Attitude</h3><div class="row"><span class="k">Pitch</span><span class="v">${f(t.attitude.pitch_deg,2)}</span></div><div class="row"><span class="k">Roll</span><span class="v">${f(t.attitude.roll_deg,2)}</span></div><div class="row"><span class="k">Yaw</span><span class="v">${f(t.attitude.yaw_deg,2)}</span></div><div class="row"><span class="k">Mode</span><span class="v">${t.flight_mode}</span></div>`;
      by('sys').innerHTML = `<h3>System</h3><div class="row"><span class="k">Frames</span><span class="v">${t.frames_total}</span></div><div class="row"><span class="k">ESPNow</span><span class="v">${t.espnow.packets}</span></div><div class="row"><span class="k">No CRSF</span><span class="v">${t.espnow.no_crsf}</span></div><div class="row"><span class="k">STA</span><span class="v">${t.wifi.sta_status}</span></div><div class="row"><span class="k">STA IP</span><span class="v">${t.wifi.sta_ip}</span></div><div class="row"><span class="k">Note</span><span class="v">${t.wifi.note}</span></div>`;
      let html = '<h3>All CRSF Frame Types</h3>';
      const arr = Array.isArray(t.frame_types) ? t.frame_types : [];
      if (arr.length === 0) {
        html += '<div class="muted">No frames yet</div>';
      } else {
        arr.sort((a,b)=>a.type-b.type);
        for (const ft of arr) {
          html += `<div class="row"><span class="k">0x${Number(ft.type).toString(16).toUpperCase().padStart(2,'0')} cnt:${ft.count}</span><span class="v">${ft.payload_len}B</span></div>`;
          html += `<div class="muted">${ft.last_hex}</div>`;
        }
      }
      by('all').innerHTML = html;
    }

    async function loadConfig(){
      const r = await fetch('/api/config', {cache:'no-store'});
      const c = await r.json();
      by('bind_phrase').value = c.bind_phrase || '';
      by('uid_text').value = c.uid_text || '';
      by('effective_uid').textContent = c.effective_uid || '-';
    }

    async function refreshTelemetry(){
      try { const r = await fetch('/api/telemetry', {cache:'no-store'}); if (r.ok) drawTelemetry(await r.json()); } catch(_) {}
    }

    by('save_bind').onclick = async () => {
      const body = new URLSearchParams({bind_phrase: by('bind_phrase').value, uid_text: by('uid_text').value});
      const r = await fetch('/api/bind', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
      by('bind_msg').textContent = await r.text();
      await loadConfig();
    };

    loadConfig();
    refreshTelemetry();
    setInterval(refreshTelemetry, 500);
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

void apply_uid_to_station_mac() {
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_mac(WIFI_IF_STA, config.uid);
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
  server.send(200, "text/plain", "Saved bind settings.");
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

  // Captive portal URLs used by Android / iOS / Windows.
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
  load_config();

  Serial.println();
  Serial.println("ELRS Backpack panel with WiFi+Bind settings");

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
}

void loop() {
  process_espnow_queue();
  dns_server.processNextRequest();
  server.handleClient();
}
