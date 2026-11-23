/*
 * ======================================================================================
 * LASTLINK — Decentralized 433 MHz LoRa Mesh Handheld Transceiver Firmware (v3.0 SSOT)
 * Target Hardware: ESP32-WROOM-32 / ESP32-WROOM-DA + RFM96W/SX1278 + SSD1306 + NEO-6M
 * Core Framework:  Arduino-ESP32 Core 3.x (ledcAttach / esp_light_sleep_start)
 * ======================================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <math.h>

// ======================================================================================
// 1. NODE IDENTITY & RF CONFIGURATION
// ======================================================================================
#define MY_NODE_ID        1             // Unique Node ID (1..4) — set per handheld unit
#define MESH_SECRET       0xA55A9C39UL  // Shared 32-bit FNV-1a authentication secret
#define LORA_FREQ_HZ      433E6         // 433 MHz ISM Band
#define DEFAULT_TTL       3             // Default maximum hop count (1..5)
#define MAX_NEIGHBORS     12            // Maximum tracked peer nodes
#define DUP_CACHE_SIZE    32            // Circular ring-buffer size for deduplication
#define INBOX_SIZE        16            // Circular triage message log size
#define RSSI_HIST_LEN     28            // Rescue Finder oscilloscope sample buffer

// ======================================================================================
// 2. VERIFIED WI-FI SAFE GPIO PINOUT (SSOT v3.0)
// ======================================================================================
// LoRa Transceiver (Hardware VSPI)
#define LORA_SCK_PIN      18
#define LORA_MISO_PIN     19
#define LORA_MOSI_PIN     23
#define LORA_NSS_PIN      5
#define LORA_RST_PIN      14
#define LORA_DIO0_PIN     2             // Packet RX/TX IRQ + Light Sleep wakeup source

// OLED Display (Hardware I2C @ 400 kHz)
#define OLED_SDA_PIN      21
#define OLED_SCL_PIN      22
#define OLED_ADDR         0x3C
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64

// Tactile UI Buttons (Active LOW, Wi-Fi Safe ADC1 / Pure Digital)
#define BTN_SOS_PIN       32            // RED/SOS: 3x click = SOS; 1x = Backspace/Cancel
#define BTN_SELECT_PIN    4             // SELECT: Short = Confirm; Long (750ms) = Back/Send
#define BTN_NEXT_PIN      15            // NEXT: Short = Next; Hold (>450ms) = Turbo scroll

// Audio, LED & Battery Sense
#define BUZZER_PIN        13            // 3-pin passive buzzer (LEDC PWM)
#define STATUS_LED_PIN    33            // External status LED (via 220R)
#define BAT_ADC_PIN       34            // 100k + 100k voltage divider midpoint

// Optional NEO-6M GNSS Receiver (UART2 @ 9600 baud, auto-detected)
#define GPS_RX_PIN        26            // ESP32 RX2 <- NEO-6M TX (or GPIO 16 on 38-pin)
#define GPS_TX_PIN        27            // ESP32 TX2 -> NEO-6M RX (or GPIO 17 on 38-pin)

// ======================================================================================
// 3. OVER-THE-AIR BINARY PROTOCOL (44-BYTE PACKED FRAME)
// ======================================================================================
enum PacketType : uint8_t {
  PKT_SOS       = 0x01,
  PKT_PRESET    = 0x02,
  PKT_TEXT      = 0x03,
  PKT_ACK       = 0x04,
  PKT_RANGETEST = 0x06,
  PKT_HEARTBEAT = 0x55,
  PKT_ROLLCALL  = 0xA5,
  PKT_PRESENT   = 0x5A
};

#pragma pack(push, 1)
struct LoRaPacket {
  uint8_t  type;          // 1B: PacketType
  uint16_t srcId;         // 2B: Originating node ID
  char     srcName[7];    // 7B: 6-char null-terminated callsign
  uint16_t dstId;         // 2B: 0xFFFF = Broadcast, else Unicast target ID
  uint16_t msgId;         // 2B: Monotonic packet sequence ID
  uint8_t  origTtl;       // 1B: Initial TTL at source
  uint8_t  ttl;           // 1B: Remaining hops
  uint8_t  code;          // 1B: Preset index or 0xAA for Range-Test ACK
  uint8_t  hasGps;        // 1B: 1 if valid latE7/lonE7 attached, else 0
  int32_t  latE7;         // 4B: Latitude * 1e7
  int32_t  lonE7;         // 4B: Longitude * 1e7
  char     text[17];      // 17B: 16-char null-terminated payload
  uint16_t authTag;       // 2B: Keyed 16-bit FNV-1a MAC
};
#pragma pack(pop)

static_assert(sizeof(LoRaPacket) == 44, "LoRaPacket must be strictly 44 bytes");

// ======================================================================================
// 4. DATA STRUCTURES & UI STATE MACHINE
// ======================================================================================
enum ScreenState : uint8_t {
  SCR_HOME = 0,
  SCR_MESH,
  SCR_MSG_MENU,
  SCR_TARGET_PICK,
  SCR_KEYBOARD,
  SCR_PRESETS,
  SCR_INBOX,
  SCR_RESCUE,
  SCR_SETTINGS,
  SCR_RANGETEST,
  SCR_SOS_COUNTDOWN,
  SCR_TX_STATUS,
  SCR_INCOMING_ALERT
};

struct PeerNode {
  bool     active;
  uint16_t id;
  char     name[7];
  int16_t  lastRssi;
  float    lastSnr;
  bool     hasGps;
  int32_t  latE7;
  int32_t  lonE7;
  uint32_t lastSeenMs;
  int16_t  rssiHist[RSSI_HIST_LEN];
  uint8_t  rssiCount;
};

struct InboxEntry {
  bool     valid;
  uint8_t  type;
  uint16_t srcId;
  char     srcName[7];
  int16_t  rssi;
  uint8_t  hops;
  bool     hasGps;
  int32_t  latE7;
  int32_t  lonE7;
  char     text[17];
  uint32_t rxMs;
};

struct DupEntry {
  uint16_t srcId;
  uint16_t msgId;
  uint8_t  type;
};

// ======================================================================================
// 5. GLOBAL OBJECTS & RUNTIME STATE
// ======================================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
WebServer server(80);
Preferences prefs;

// Persistent Settings
char     myNodeName[7]   = "NODE01";
uint8_t  cfgTtl          = DEFAULT_TTL;
bool     cfgLongRange    = false;        // false = SF7 (NM), true = SF10 (LR)
bool     cfgBuzzer       = true;
uint8_t  cfgSleepIdx     = 1;            // 0=15s, 1=30s, 2=60s, 3=Never
const uint32_t SLEEP_OPTIONS_MS[4] = {15000UL, 30000UL, 60000UL, 0UL};

// Hardware & Radio Runtime
volatile bool loraIrqFlag = false;
bool     gpsDetected      = false;
bool     wifiApEnabled    = false;
uint16_t localMsgSeq      = 1;
uint32_t lastActivityMs   = 0;
uint32_t lastHeartbeatMs  = 0;
uint32_t relayBannerUntil = 0;
char     relayBannerName[7] = "";

// Mesh Tables
PeerNode   peers[MAX_NEIGHBORS];
InboxEntry inbox[INBOX_SIZE];
uint8_t    inboxHead      = 0;
bool       unreadFlag     = false;
DupEntry   dupCache[DUP_CACHE_SIZE];
uint8_t    dupHead        = 0;

// UI & Navigation State
ScreenState currentScreen = SCR_HOME;
uint8_t     homeCursor    = 0;
uint8_t     menuCursor    = 0;
uint8_t     settingsCursor= 0;
uint8_t     settingsTop   = 0;
uint8_t     inboxCursor   = 0;
uint8_t     rescuePeerIdx = 0;
uint16_t    selectedDstId = 0xFFFF;
uint8_t     targetCursor  = 0;

// 4x7 Keyboard State
bool        kbdEditNodeId = false;
uint8_t     kbdRow        = 0;
uint8_t     kbdCol        = 0;
bool        kbdRowLocked  = false;
char        kbdBuffer[17] = "";
const char* KBD_ROWS[4]   = {"ABCDEFG", "HIJKLMN", "OPQRSTU", "VWXYZ_<"}; // '_'=Space, '<'=OK

// Presets
const char* PRESET_LIST[6] = {
  "NEED HELP!", "MEDICAL!", "TRAPPED!",
  "NEED WATER", "NEED FOOD", "I AM SAFE"
};

// SOS & TX Pipeline State
bool     sosActive        = false;
uint32_t sosCountdownStart= 0;
uint32_t sosLastPulseMs   = 0;
uint16_t sosPulseCount    = 0;
uint16_t sosBaseMsgId     = 0;

uint8_t  txStage          = 0;            // 1=SENT, 2=RELAY, 3=ACK
uint16_t trackedMsgId     = 0;
uint8_t  trackedRetries   = 0;
uint32_t trackedLastTxMs  = 0;
LoRaPacket trackedPkt;
bool     txTrackingActive = false;

// Incoming Alert Banner State
InboxEntry activeAlert;

// Range Test State
uint32_t rtLastPingMs     = 0;
uint16_t rtSent           = 0;
uint16_t rtAcked          = 0;
int16_t  rtLastRssi       = 0;
float    rtLastSnr        = 0.0f;

// Button Debounce & Multi-Click State
struct BtnState {
  uint8_t  pin;
  bool     lastRaw;
  bool     stableState;
  uint32_t lastDebounceMs;
  uint32_t pressStartMs;
  bool     longHandled;
};
BtnState btnSos    = {BTN_SOS_PIN,    HIGH, HIGH, 0, 0, false};
BtnState btnSelect = {BTN_SELECT_PIN, HIGH, HIGH, 0, 0, false};
BtnState btnNext   = {BTN_NEXT_PIN,   HIGH, HIGH, 0, 0, false};

uint8_t  sosClickCount    = 0;
uint32_t sosFirstClickMs  = 0;
uint32_t lastTurboMs      = 0;

// Non-Blocking Audio Sequencer
struct ToneStep { uint16_t freq; uint16_t durMs; };
ToneStep audioQueue[8];
uint8_t  audioLen         = 0;
uint8_t  audioIdx         = 0;
uint32_t audioStepEndMs   = 0;

// ======================================================================================
// 6. CRYPTOGRAPHIC FNV-1a MAC & PACKET HELPERS
// ======================================================================================
uint16_t computeAuthTag(const LoRaPacket& pkt) {
  uint32_t hash = 2166136261UL ^ MESH_SECRET;
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
  const size_t payloadLen = sizeof(LoRaPacket) - sizeof(uint16_t);
  for (size_t i = 0; i < payloadLen; i++) {
    hash ^= raw[i];
    hash *= 16777619UL;
  }
  return static_cast<uint16_t>((hash >> 16) ^ (hash & 0xFFFF));
}

bool isDuplicatePacket(uint16_t srcId, uint16_t msgId, uint8_t type) {
  for (uint8_t i = 0; i < DUP_CACHE_SIZE; i++) {
    if (dupCache[i].srcId == srcId && dupCache[i].msgId == msgId && dupCache[i].type == type) {
      return true;
    }
  }
  return false;
}

void recordPacketSeen(uint16_t srcId, uint16_t msgId, uint8_t type) {
  dupCache[dupHead] = {srcId, msgId, type};
  dupHead = (dupHead + 1) % DUP_CACHE_SIZE;
}

// ======================================================================================
// 7. HARDWARE SERVICES (AUDIO, BATTERY, GPS, RADIO)
// ======================================================================================
void queueTone(uint16_t f1, uint16_t d1, uint16_t f2 = 0, uint16_t d2 = 0,
               uint16_t f3 = 0, uint16_t d3 = 0, uint16_t f4 = 0, uint16_t d4 = 0) {
  if (!cfgBuzzer) return;
  audioLen = 0;
  if (d1) audioQueue[audioLen++] = {f1, d1};
  if (d2) audioQueue[audioLen++] = {f2, d2};
  if (d3) audioQueue[audioLen++] = {f3, d3};
  if (d4) audioQueue[audioLen++] = {f4, d4};
  audioIdx = 0;
  if (audioLen > 0) {
    ledcWriteTone(BUZZER_PIN, audioQueue[0].freq);
    audioStepEndMs = millis() + audioQueue[0].durMs;
  }
}

void serviceAudio() {
  if (audioLen == 0) return;
  if (millis() >= audioStepEndMs) {
    audioIdx++;
    if (audioIdx < audioLen) {
      ledcWriteTone(BUZZER_PIN, audioQueue[audioIdx].freq);
      audioStepEndMs = millis() + audioQueue[audioIdx].durMs;
    } else {
      ledcWriteTone(BUZZER_PIN, 0);
      audioLen = 0;
    }
  }
}

uint8_t readBatteryPercent() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(BAT_ADC_PIN);
  float vPin = (sum / 8.0f) * (3.3f / 4095.0f);
  float vBat = vPin * 2.0f * 1.06f; // 100k/100k divider + ADC calibration factor
  if (vBat >= 4.15f) return 100;
  if (vBat <= 3.30f) return 0;
  return static_cast<uint8_t>(((vBat - 3.30f) / (4.15f - 3.30f)) * 100.0f);
}

void serviceGps() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (!gpsDetected && (c == '$' || c == '\n')) {
      gpsDetected = true;
    }
    gps.encode(c);
  }
}

void applyRadioProfile() {
  LoRa.idle();
  if (cfgLongRange) {
    LoRa.setSpreadingFactor(10);      // Long-Range SF10
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);
  } else {
    LoRa.setSpreadingFactor(7);       // Normal SF7
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();
  LoRa.receive();
}

void IRAM_ATTR onLoraDio0() {
  loraIrqFlag = true;
}

void transmitPacket(LoRaPacket& pkt) {
  pkt.authTag = computeAuthTag(pkt);
  recordPacketSeen(pkt.srcId, pkt.msgId, pkt.type);
  digitalWrite(STATUS_LED_PIN, HIGH);
  LoRa.beginPacket();
  LoRa.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LoRaPacket));
  LoRa.endPacket(false);
  LoRa.receive();
  digitalWrite(STATUS_LED_PIN, LOW);
}

// ======================================================================================
// 8. VECTOR MASCOT ANIMATIONS (6 EVENT-DRIVEN STATES)
// ======================================================================================
void drawMascotEyes(int cx, int cy, int w, int h, bool happy) {
  display.fillRoundRect(cx - 14 - w / 2, cy - h / 2, w, h, 2, SSD1306_WHITE);
  display.fillRoundRect(cx + 14 - w / 2, cy - h / 2, w, h, 2, SSD1306_WHITE);
  if (happy) {
    display.fillRect(cx - 20, cy + 1, 40, 8, SSD1306_BLACK);
  }
}

void animGlitch() {
  for (int f = 0; f < 4; f++) {
    display.clearDisplay();
    int shift = (f % 2 == 0) ? -4 : 4;
    drawMascotEyes(64 + shift, 32, 12, (f == 1) ? 2 : 12, false);
    display.drawFastHLine(0, 18 + f * 7, 128, SSD1306_WHITE);
    display.display();
    delay(45);
  }
}

void animSlide() {
  for (int x = 128; x >= 64; x -= 16) {
    display.clearDisplay();
    drawMascotEyes(x, 32, 14, 10, false);
    display.display();
    delay(20);
  }
}

void animToss() {
  for (int step = 0; step <= 4; step++) {
    display.clearDisplay();
    int y = 44 - (step * 7);
    int r = 10 - abs(2 - step) * 3;
    display.drawRoundRect(64 - r, y - 6, r * 2 + 2, 12, 2, SSD1306_WHITE);
    display.display();
    delay(28);
  }
}

void animSpinZoom() {
  for (int r = 2; r <= 18; r += 4) {
    display.clearDisplay();
    display.drawCircle(64, 32, r, SSD1306_WHITE);
    drawMascotEyes(64, 32, 12, 12, true);
    display.display();
    delay(30);
  }
}

void animDither() {
  display.clearDisplay();
  for (int y = 0; y < 64; y += 4) {
    for (int x = 0; x < 128; x += 4) {
      display.fillRect(x, y, 2, 2, SSD1306_WHITE);
    }
  }
  display.display();
  delay(80);
}

void animSleep() {
  display.clearDisplay();
  drawMascotEyes(64, 32, 12, 3, false);
  display.display();
  delay(120);
  display.clearDisplay();
  display.drawFastHLine(20, 32, 88, SSD1306_WHITE);
  display.display();
  delay(90);
  display.clearDisplay();
  display.display();
}

// ======================================================================================
// 9. MESH ROUTING, PEER TABLE & GEODESIC MATH
// ======================================================================================
int findOrAllocPeer(uint16_t id) {
  int freeIdx = -1;
  int oldestIdx = 0;
  uint32_t oldestTs = UINT32_MAX;
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (peers[i].active && peers[i].id == id) return i;
    if (!peers[i].active && freeIdx < 0) freeIdx = i;
    if (peers[i].active && peers[i].lastSeenMs < oldestTs) {
      oldestTs = peers[i].lastSeenMs;
      oldestIdx = i;
    }
  }
  int idx = (freeIdx >= 0) ? freeIdx : oldestIdx;
  memset(&peers[idx], 0, sizeof(PeerNode));
  peers[idx].active = true;
  peers[idx].id = id;
  return idx;
}

uint8_t countActivePeers() {
  uint8_t c = 0;
  uint32_t now = millis();
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (peers[i].active && (now - peers[i].lastSeenMs) < 600000UL) c++;
  }
  return c;
}

void pushRssiSample(PeerNode& p, int16_t rssi) {
  if (p.rssiCount < RSSI_HIST_LEN) {
    p.rssiHist[p.rssiCount++] = rssi;
  } else {
    memmove(p.rssiHist, p.rssiHist + 1, (RSSI_HIST_LEN - 1) * sizeof(int16_t));
    p.rssiHist[RSSI_HIST_LEN - 1] = rssi;
  }
}

void computeHaversine(double lat1, double lon1, double lat2, double lon2,
                       double& distMeters, double& bearingDeg) {
  const double R = 6371000.0;
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double dPhi = radians(lat2 - lat1);
  double dLam = radians(lon2 - lon1);
  double a = sin(dPhi / 2.0) * sin(dPhi / 2.0) +
             cos(phi1) * cos(phi2) * sin(dLam / 2.0) * sin(dLam / 2.0);
  distMeters = R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double y = sin(dLam) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLam);
  bearingDeg = fmod(degrees(atan2(y, x)) + 360.0, 360.0);
}

const char* cardinalFromBearing(double deg) {
  const char* dirs[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[static_cast<int>((deg + 22.5) / 45.0) % 8];
}

void sendControlPacket(uint8_t type, uint16_t dstId = 0xFFFF, uint16_t refMsgId = 0, uint8_t code = 0) {
  LoRaPacket pkt{};
  pkt.type    = type;
  pkt.srcId   = MY_NODE_ID;
  strncpy(pkt.srcName, myNodeName, 6);
  pkt.dstId   = dstId;
  pkt.msgId   = (refMsgId != 0) ? refMsgId : localMsgSeq++;
  pkt.origTtl = cfgTtl;
  pkt.ttl     = cfgTtl;
  pkt.code    = code;
  if (gpsDetected && gps.location.isValid()) {
    pkt.hasGps = 1;
    pkt.latE7  = static_cast<int32_t>(gps.location.lat() * 1e7);
    pkt.lonE7  = static_cast<int32_t>(gps.location.lng() * 1e7);
  }
  transmitPacket(pkt);
}

void sendUserMessage(uint8_t type, uint16_t dstId, const char* msgText, uint8_t code = 0) {
  LoRaPacket pkt{};
  pkt.type    = type;
  pkt.srcId   = MY_NODE_ID;
  strncpy(pkt.srcName, myNodeName, 6);
  pkt.dstId   = dstId;
  pkt.msgId   = localMsgSeq++;
  pkt.origTtl = cfgTtl;
  pkt.ttl     = cfgTtl;
  pkt.code    = code;
  strncpy(pkt.text, msgText, 16);
  if (gpsDetected && gps.location.isValid()) {
    pkt.hasGps = 1;
    pkt.latE7  = static_cast<int32_t>(gps.location.lat() * 1e7);
    pkt.lonE7  = static_cast<int32_t>(gps.location.lng() * 1e7);
  }
  transmitPacket(pkt);

  trackedPkt       = pkt;
  trackedMsgId     = pkt.msgId;
  trackedRetries   = 0;
  trackedLastTxMs  = millis();
  txStage          = 1; // [SENT]
  txTrackingActive = true;
}

void handleIncomingLoRa(bool wokeFromSleep) {
  int packetSize = LoRa.parsePacket();
  if (packetSize != sizeof(LoRaPacket)) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  LoRaPacket pkt{};
  LoRa.readBytes(reinterpret_cast<uint8_t*>(&pkt), sizeof(LoRaPacket));
  int16_t rssi = LoRa.packetRssi();
  float   snr  = LoRa.packetSnr();

  // Verify FNV-1a MAC integrity
  if (pkt.authTag != computeAuthTag(pkt)) return;
  if (pkt.srcId == MY_NODE_ID) return;

  // Update Peer Table
  int pIdx = findOrAllocPeer(pkt.srcId);
  peers[pIdx].active     = true;
  peers[pIdx].id         = pkt.srcId;
  strncpy(peers[pIdx].name, pkt.srcName, 6);
  peers[pIdx].lastRssi   = rssi;
  peers[pIdx].lastSnr    = snr;
  peers[pIdx].lastSeenMs = millis();
  pushRssiSample(peers[pIdx], rssi);
  if (pkt.hasGps) {
    peers[pIdx].hasGps = true;
    peers[pIdx].latE7  = pkt.latE7;
    peers[pIdx].lonE7  = pkt.lonE7;
  }

  // Check if neighbor overhear confirms [RELAY] stage for our active TX
  if (txTrackingActive && pkt.srcId != MY_NODE_ID && pkt.msgId == trackedMsgId && txStage < 2) {
    txStage = 2; // [RELAY]
  }

  bool seenBefore = isDuplicatePacket(pkt.srcId, pkt.msgId, pkt.type);
  if (seenBefore) return;
  recordPacketSeen(pkt.srcId, pkt.msgId, pkt.type);

  uint8_t hops = (pkt.origTtl >= pkt.ttl) ? (pkt.origTtl - pkt.ttl + 1) : 1;

  // Controlled Flooding Relay (with randomized 60-240 ms anti-collision jitter)
  if (pkt.ttl > 1 && (pkt.dstId == 0xFFFF || pkt.dstId != MY_NODE_ID)) {
    LoRaPacket fwd = pkt;
    fwd.ttl--;
    delay(random(60, 241));
    transmitPacket(fwd);
    strncpy(relayBannerName, pkt.srcName, 6);
    relayBannerUntil = millis() + 2500UL;
  }

  // Handle Packet Types
  if (pkt.type == PKT_ROLLCALL) {
    delay(random(40, 221));
    sendControlPacket(PKT_PRESENT, pkt.srcId);
    return; // Silent machine-to-machine reply (OLED stays off if sleeping)
  }

  if (pkt.type == PKT_HEARTBEAT || pkt.type == PKT_PRESENT) {
    return; // Silent telemetry update
  }

  if (pkt.type == PKT_RANGETEST) {
    delay(random(30, 120));
    sendControlPacket(PKT_ACK, pkt.srcId, pkt.msgId, 0xAA);
    return;
  }

  if (pkt.type == PKT_ACK) {
    if (pkt.dstId == MY_NODE_ID) {
      if (pkt.code == 0xAA) {
        rtAcked++;
        rtLastRssi = rssi;
        rtLastSnr  = snr;
      } else if (txTrackingActive &&
                (pkt.msgId == trackedMsgId || (sosActive && pkt.msgId >= sosBaseMsgId))) {
        txStage = 3; // [ACK]
        txTrackingActive = false;
        queueTone(1500, 70, 2400, 110);
        if (!wokeFromSleep) animSpinZoom();
      }
    }
    return;
  }

  // User-facing Messages (SOS, Preset, Text)
  if (pkt.dstId == 0xFFFF || pkt.dstId == MY_NODE_ID) {
    InboxEntry entry{};
    entry.valid   = true;
    entry.type    = pkt.type;
    entry.srcId   = pkt.srcId;
    strncpy(entry.srcName, pkt.srcName, 6);
    entry.rssi    = rssi;
    entry.hops    = hops;
    entry.hasGps  = pkt.hasGps;
    entry.latE7   = pkt.latE7;
    entry.lonE7   = pkt.lonE7;
    strncpy(entry.text, pkt.text, 16);
    entry.rxMs    = millis();

    inbox[inboxHead] = entry;
    inboxHead = (inboxHead + 1) % INBOX_SIZE;
    unreadFlag = true;
    activeAlert = entry;

    // Send ACK back to originator
    delay(random(50, 180));
    sendControlPacket(PKT_ACK, pkt.srcId, pkt.msgId, 0x00);

    // Wake OLED & trigger Alert Screen + Morse Buzzer
    if (wokeFromSleep) {
      display.ssd1306_command(SSD1306_DISPLAYON);
      animGlitch();
    }
    lastActivityMs = millis();
    currentScreen  = SCR_INCOMING_ALERT;
    if (pkt.type == PKT_SOS) {
      queueTone(2800, 120, 1900, 120, 2800, 120, 1900, 260);
    } else {
      queueTone(1800, 90, 2200, 120);
    }
  }
}

// ======================================================================================
// 10. OFFLINE WI-FI COMMAND PORTAL (192.168.4.1)
// ======================================================================================
void handleWebRoot() {
  String html = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<meta http-equiv='refresh' content='5'>"
                  "<title>LASTLINK Command Portal</title>"
                  "<style>body{background:#0b0d10;color:#e5e7eb;font-family:monospace;padding:12px;margin:0}"
                  "h2{color:#38bdf8;margin:0 0 8px}table{width:100%;border-collapse:collapse;margin:10px 0}"
                  "th,td{border:1px solid #232833;padding:6px;text-align:left;font-size:13px}"
                  ".sos{background:#3f1215;border-left:4px solid #ef4444;padding:8px;margin:6px 0}"
                  ".msg{background:#12151b;border-left:4px solid #10b981;padding:8px;margin:6px 0}"
                  "input,button{background:#181c24;color:#fff;border:1px solid #38bdf8;padding:8px;font-family:monospace}"
                  "a{color:#38bdf8}</style></head><body>");
  html += "<h2>LASTLINK // " + String(myNodeName) + " (192.168.4.1)</h2>";
  html += "<div>BAT: " + String(readBatteryPercent()) + "% | RF: " +
          String(cfgLongRange ? "LR SF10" : "NM SF7") + " | GPS: " +
          String(gpsDetected ? (gps.location.isValid() ? "LOCK" : "WAIT") : "NONE") + "</div>";

  html += F("<form action='/send' method='POST' style='margin:10px 0'>"
            "<input name='txt' maxlength='16' placeholder='BROADCAST (16C)' required> "
            "<button type='submit'>TX MESH</button></form>");

  html += F("<h3>DISCOVERED MESH NODES</h3><table><tr><th>NODE</th><th>RSSI</th><th>GPS / MAP</th><th>AGE</th></tr>");
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (!peers[i].active) continue;
    html += "<tr><td>" + String(peers[i].name) + " (#" + String(peers[i].id) + ")</td><td>" +
            String(peers[i].lastRssi) + " dBm</td><td>";
    if (peers[i].hasGps) {
      double lat = peers[i].latE7 / 1e7;
      double lon = peers[i].lonE7 / 1e7;
      html += "<a target='_blank' href='https://maps.google.com/?q=" +
              String(lat, 6) + "," + String(lon, 6) + "'>" +
              String(lat, 5) + "," + String(lon, 5) + "</a>";
    } else {
      html += "RF-ONLY";
    }
    html += "</td><td>" + String((millis() - peers[i].lastSeenMs) / 1000) + "s</td></tr>";
  }
  html += F("</table><h3>EMERGENCY TRIAGE & INBOX</h3>");

  for (int k = 0; k < INBOX_SIZE; k++) {
    int idx = (inboxHead - 1 - k + INBOX_SIZE) % INBOX_SIZE;
    if (!inbox[idx].valid) continue;
    html += (inbox[idx].type == PKT_SOS) ? "<div class='sos'>" : "<div class='msg'>";
    html += "<b>" + String(inbox[idx].srcName) + "</b> (" + String(inbox[idx].hops) + "H, " +
            String(inbox[idx].rssi) + " dBm): " + String(inbox[idx].text);
    if (inbox[idx].hasGps) {
      double lat = inbox[idx].latE7 / 1e7;
      double lon = inbox[idx].lonE7 / 1e7;
      html += " [<a target='_blank' href='https://maps.google.com/?q=" +
              String(lat, 6) + "," + String(lon, 6) + "'>MAP</a>]";
    }
    html += "</div>";
  }
  html += F("</body></html>");
  server.send(200, "text/html", html);
}

void handleWebSend() {
  if (server.hasArg("txt")) {
    String msg = server.arg("txt");
    msg.toUpperCase();
    char buf[17] = "";
    msg.toCharArray(buf, 17);
    if (strlen(buf) > 0) {
      sendUserMessage(PKT_TEXT, 0xFFFF, buf);
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void setWifiPortal(bool enable) {
  if (enable && !wifiApEnabled) {
    WiFi.mode(WIFI_AP);
    String ssid = "LASTLINK_" + String(myNodeName);
    WiFi.softAP(ssid.c_str());
    esp_wifi_set_max_tx_power(44); // ~11 dBm (WIFI_POWER_11dBm) to prevent 3.3V brownout
    server.on("/", HTTP_GET, handleWebRoot);
    server.on("/send", HTTP_POST, handleWebSend);
    server.begin();
    wifiApEnabled = true;
  } else if (!enable && wifiApEnabled) {
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiApEnabled = false;
  }
}

// ======================================================================================
// 11. OLED DISPLAY RENDERER (128x64 STATUS BAR + 12 SCREENS)
// ======================================================================================
void drawStatusBar() {
  display.fillRect(0, 0, 128, 11, SSD1306_BLACK);
  display.drawFastHLine(0, 11, 128, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 1);

  if (millis() < relayBannerUntil) {
    display.print(">>RELAY:");
    display.print(relayBannerName);
  } else {
    display.print(myNodeName);
  }

  // Radio / Mesh Badge: [NM|2NG]
  char badge[12];
  snprintf(badge, sizeof(badge), "[%s|%d%c%c]",
           cfgLongRange ? "LR" : "NM",
           countActivePeers(),
           (gpsDetected && gps.location.isValid()) ? 'G' : '-',
           wifiApEnabled ? 'W' : ' ');
  display.setCursor(46, 1);
  display.print(badge);

  // Battery Percentage + Icon
  uint8_t bat = readBatteryPercent();
  display.setCursor(96, 1);
  display.print(bat);
  display.print("%");
  display.drawRect(118, 2, 9, 6, SSD1306_WHITE);
  display.fillRect(119, 3, (bat * 7) / 100, 4, SSD1306_WHITE);
}

void renderScreen() {
  display.clearDisplay();
  drawStatusBar();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (currentScreen) {
    case SCR_HOME: {
      const char* labels[4] = {"MESH NET", "MESSAGES", "RESCUE", "SETTINGS"};
      for (uint8_t i = 0; i < 4; i++) {
        int x = (i % 2) * 64 + 1;
        int y = (i / 2) * 25 + 14;
        if (i == homeCursor) {
          display.fillRoundRect(x, y, 61, 23, 3, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.drawRoundRect(x, y, 61, 23, 3, SSD1306_WHITE);
          display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(x + 6, y + 8);
        display.print(labels[i]);
        if (i == 1 && unreadFlag) {
          display.fillCircle(x + 55, y + 5, 2, (i == homeCursor) ? SSD1306_BLACK : SSD1306_WHITE);
        }
      }
      break;
    }

    case SCR_MESH: {
      display.setCursor(0, 14);
      display.print("PEER   RSSI  GPS  AGE");
      int row = 0;
      for (int i = 0; i < MAX_NEIGHBORS && row < 4; i++) {
        if (!peers[i].active) continue;
        int y = 24 + row * 10;
        display.setCursor(0, y);
        display.printf("%-6s %4d %s %3lus",
                       peers[i].name,
                       peers[i].lastRssi,
                       peers[i].hasGps ? "GPS" : "---",
                       (millis() - peers[i].lastSeenMs) / 1000UL);
        row++;
      }
      if (row == 0) {
        display.setCursor(10, 34);
        display.print("SCANNING MESH...");
      }
      break;
    }

    case SCR_MSG_MENU: {
      const char* items[4] = {
        "1. Write Msg (Keybd)",
        "2. Quick Presets",
        "3. Inbox & Triage",
        "4. < Back Home"
      };
      for (uint8_t i = 0; i < 4; i++) {
        int y = 15 + i * 12;
        if (i == menuCursor) {
          display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(2, y);
        display.print(items[i]);
      }
      break;
    }

    case SCR_TARGET_PICK: {
      display.setCursor(0, 14);
      display.print("SELECT TARGET:");
      display.setCursor(2, 26);
      if (targetCursor == 0) {
        display.print("> ALL NODES (BCAST)");
      } else {
        int found = 0;
        for (int i = 0; i < MAX_NEIGHBORS; i++) {
          if (!peers[i].active) continue;
          found++;
          if (found == targetCursor) {
            display.printf("> PRIV -> %s (#%d)", peers[i].name, peers[i].id);
            selectedDstId = peers[i].id;
            break;
          }
        }
      }
      if (targetCursor == 0) selectedDstId = 0xFFFF;
      display.setCursor(0, 54);
      display.print("[NEXT]Cyc [SEL]Ok");
      break;
    }

    case SCR_KEYBOARD: {
      display.setCursor(0, 13);
      display.print(kbdEditNodeId ? "ID: " : "MSG:");
      display.print(kbdBuffer);
      display.print("_");
      for (uint8_t r = 0; r < 4; r++) {
        int y = 24 + r * 10;
        if (r == kbdRow && !kbdRowLocked) {
          display.drawRect(0, y - 1, 128, 10, SSD1306_WHITE);
        }
        for (uint8_t c = 0; c < 7; c++) {
          int x = 4 + c * 18;
          char ch = KBD_ROWS[r][c];
          if (r == kbdRow && c == kbdCol && kbdRowLocked) {
            display.fillRect(x - 2, y - 1, 12, 10, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
          } else {
            display.setTextColor(SSD1306_WHITE);
          }
          display.setCursor(x, y);
          display.print((ch == '_') ? ' ' : ch);
        }
      }
      break;
    }

    case SCR_PRESETS: {
      for (uint8_t i = 0; i < 5; i++) {
        uint8_t idx = (menuCursor + i) % 6;
        int y = 14 + i * 10;
        if (i == 0) {
          display.fillRect(0, y - 1, 128, 10, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(4, y);
        display.print(PRESET_LIST[idx]);
      }
      break;
    }

    case SCR_INBOX: {
      int idx = (inboxHead - 1 - inboxCursor + INBOX_SIZE) % INBOX_SIZE;
      if (!inbox[idx].valid) {
        display.setCursor(18, 32);
        display.print("INBOX EMPTY");
      } else {
        display.setCursor(0, 14);
        display.printf("%s %dH %ddBm", inbox[idx].srcName, inbox[idx].hops, inbox[idx].rssi);
        display.setCursor(0, 26);
        display.print(inbox[idx].text);
        display.setCursor(0, 40);
        if (inbox[idx].hasGps) {
          display.printf("%.4f,%.4f", inbox[idx].latE7 / 1e7, inbox[idx].lonE7 / 1e7);
        } else {
          display.print("GPS: NO FIX / RF-ONLY");
        }
        display.setCursor(0, 54);
        display.print("[NEXT]Scroll [HOLD]Bk");
      }
      break;
    }

    case SCR_RESCUE: {
      int pIdx = -1;
      int seen = 0;
      for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!peers[i].active) continue;
        if (seen == rescuePeerIdx) { pIdx = i; break; }
        seen++;
      }
      if (pIdx < 0) {
        display.setCursor(10, 32);
        display.print("NO ACTIVE PEERS");
        break;
      }
      PeerNode& p = peers[pIdx];
      display.setCursor(0, 13);
      display.printf("TGT:%s (%ddBm)", p.name, p.lastRssi);

      if (gpsDetected && gps.location.isValid() && p.hasGps) {
        // Outdoor GPS Haversine Vector Mode
        double distM = 0, brg = 0;
        computeHaversine(gps.location.lat(), gps.location.lng(),
                         p.latE7 / 1e7, p.lonE7 / 1e7, distM, brg);
        display.setTextSize(2);
        display.setCursor(0, 26);
        if (distM >= 1000.0) display.printf("%.2fkm", distM / 1000.0);
        else                 display.printf("%.0fm", distM);
        display.setCursor(76, 26);
        display.print(cardinalFromBearing(brg));
        display.setTextSize(1);
        display.setCursor(0, 48);
        display.printf("BRG: %.0f deg  SNR:%.1f", brg, p.lastSnr);
      } else {
        // Indoor RSSI Oscilloscope Sparkline Mode
        float approxM = powf(10.0f, (-45.0f - p.lastRssi) / 27.0f);
        const char* trend = "STABLE";
        if (p.rssiCount >= 2) {
          int16_t diff = p.rssiHist[p.rssiCount - 1] - p.rssiHist[p.rssiCount - 2];
          if (diff >= 2) trend = "^HOT";
          else if (diff <= -2) trend = "vCLD";
        }
        display.setCursor(0, 23);
        display.printf("~%.0fm  %s  SNR:%.1f", approxM, trend, p.lastSnr);
        display.drawRect(0, 34, 128, 28, SSD1306_WHITE);
        for (uint8_t s = 1; s < p.rssiCount; s++) {
          int y1 = map(constrain(p.rssiHist[s - 1], -125, -35), -125, -35, 60, 36);
          int y2 = map(constrain(p.rssiHist[s],     -125, -35), -125, -35, 60, 36);
          display.drawLine((s - 1) * 4 + 2, y1, s * 4 + 2, y2, SSD1306_WHITE);
        }
      }
      break;
    }

    case SCR_SETTINGS: {
      const char* sleepNames[4] = {"15s", "30s", "60s", "Never"};
      char rows[8][24];
      snprintf(rows[0], 24, "1. Sleep: %s", sleepNames[cfgSleepIdx]);
      snprintf(rows[1], 24, "2. TTL: %d Hops", cfgTtl);
      snprintf(rows[2], 24, "3. Range: %s", cfgLongRange ? "Long SF10" : "Norm SF7");
      snprintf(rows[3], 24, "4. Buzzer: %s", cfgBuzzer ? "ON" : "OFF");
      snprintf(rows[4], 24, "5. NodeID: %s", myNodeName);
      snprintf(rows[5], 24, "6. WiFiAP: %s", wifiApEnabled ? "192.168.4.1" : "OFF");
      snprintf(rows[6], 24, "7. Run Range Test >");
      snprintf(rows[7], 24, "8. GPS: %s",
               !gpsDetected ? "Not Wired" : (gps.location.isValid() ? "FIX" : "Wait"));

      for (uint8_t i = 0; i < 4; i++) {
        uint8_t idx = settingsTop + i;
        if (idx >= 8) break;
        int y = 15 + i * 12;
        if (idx == settingsCursor) {
          display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(2, y);
        display.print(rows[idx]);
      }
      break;
    }

    case SCR_RANGETEST: {
      float pdr = (rtSent > 0) ? (100.0f * rtAcked / rtSent) : 0.0f;
      display.setCursor(0, 14);
      display.print("JUDGE RANGE LOGGER");
      display.setCursor(0, 26);
      display.printf("SENT:%u  ACK:%u", rtSent, rtAcked);
      display.setCursor(0, 38);
      display.printf("PDR: %.1f%%", pdr);
      display.setCursor(0, 50);
      display.printf("RSSI:%ddBm SNR:%.1f", rtLastRssi, rtLastSnr);
      break;
    }

    case SCR_SOS_COUNTDOWN: {
      uint32_t elapsed = millis() - sosCountdownStart;
      int rem = 5 - (elapsed / 1000);
      if (rem < 0) rem = 0;
      display.fillRect(0, 12, 128, 52, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(18, 18);
      display.print("!! SOS ALERT !!");
      display.setTextSize(2);
      display.setCursor(56, 32);
      display.print(rem);
      display.setTextSize(1);
      display.setCursor(16, 52);
      display.print("[RED] TO CANCEL");
      break;
    }

    case SCR_TX_STATUS: {
      display.setCursor(0, 16);
      if (sosActive) {
        display.printf("!! SOS PULSE #%u !!", sosPulseCount);
      } else {
        display.print("DISPATCH PIPELINE:");
      }
      // 3-Stage Graphical Delivery Bar: [SENT] - [RELAY] - [ACK]
      const char* st[3] = {"SENT", "RELAY", "ACK"};
      for (uint8_t s = 0; s < 3; s++) {
        int x = 2 + s * 43;
        if (txStage >= (s + 1)) {
          display.fillRoundRect(x, 30, 38, 16, 2, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.drawRoundRect(x, 30, 38, 16, 2, SSD1306_WHITE);
          display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(x + 4, 34);
        display.print(st[s]);
      }
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 53);
      display.print(sosActive ? "[RED] STOP SOS" : "[HOLD SEL] HOME");
      break;
    }

    case SCR_INCOMING_ALERT: {
      display.fillRect(0, 12, 128, 52, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(2, 15);
      display.printf("%s FROM %s (%dH)",
                     (activeAlert.type == PKT_SOS) ? "SOS!" : "MSG",
                     activeAlert.srcName, activeAlert.hops);
      display.setCursor(2, 28);
      display.print(activeAlert.text);
      display.setCursor(2, 41);
      if (activeAlert.hasGps) {
        display.printf("%.4f,%.4f", activeAlert.latE7 / 1e7, activeAlert.lonE7 / 1e7);
      } else {
        display.printf("RSSI: %d dBm", activeAlert.rssi);
      }
      display.setCursor(2, 53);
      display.print("[RED/SEL] DISMISS");
      break;
    }
  }

  display.display();
}

// ======================================================================================
// 12. INPUT HANDLING & BUTTON GRAMMAR (GPIO 32 / 4 / 15)
// ======================================================================================
void onSosTripleClick() {
  sosActive         = true;
  sosCountdownStart = millis();
  sosPulseCount     = 0;
  currentScreen     = SCR_SOS_COUNTDOWN;
  queueTone(2400, 100, 0, 80, 2400, 100);
}

void onRedSingleClick() {
  if (currentScreen == SCR_SOS_COUNTDOWN) {
    sosActive = false;
    currentScreen = SCR_HOME;
    queueTone(900, 120);
    return;
  }
  if (sosActive) {
    sosActive = false;
    txTrackingActive = false;
    currentScreen = SCR_HOME;
    queueTone(900, 120);
    return;
  }
  if (currentScreen == SCR_KEYBOARD) {
    size_t len = strlen(kbdBuffer);
    if (len > 0) kbdBuffer[len - 1] = '\0';
    kbdRowLocked = false;
    queueTone(1000, 40);
    return;
  }
  if (currentScreen == SCR_INCOMING_ALERT) {
    unreadFlag = false;
    currentScreen = SCR_HOME;
    return;
  }
  currentScreen = SCR_HOME;
}

void onSelectShort() {
  queueTone(1400, 45);
  switch (currentScreen) {
    case SCR_HOME:
      animSlide();
      if (homeCursor == 0) {
        sendControlPacket(PKT_ROLLCALL);
        currentScreen = SCR_MESH;
      } else if (homeCursor == 1) {
        menuCursor = 0;
        currentScreen = SCR_MSG_MENU;
      } else if (homeCursor == 2) {
        sendControlPacket(PKT_ROLLCALL);
        rescuePeerIdx = 0;
        currentScreen = SCR_RESCUE;
      } else {
        settingsCursor = 0;
        settingsTop = 0;
        currentScreen = SCR_SETTINGS;
      }
      break;

    case SCR_MSG_MENU:
      if (menuCursor == 0) {
        kbdEditNodeId = false;
        targetCursor = 0;
        currentScreen = SCR_TARGET_PICK;
      } else if (menuCursor == 1) {
        menuCursor = 0;
        currentScreen = SCR_PRESETS;
      } else if (menuCursor == 2) {
        inboxCursor = 0;
        unreadFlag = false;
        currentScreen = SCR_INBOX;
      } else {
        currentScreen = SCR_HOME;
      }
      break;

    case SCR_TARGET_PICK:
      kbdRow = 0; kbdCol = 0; kbdRowLocked = false;
      kbdBuffer[0] = '\0';
      currentScreen = SCR_KEYBOARD;
      break;

    case SCR_KEYBOARD: {
      if (!kbdRowLocked) {
        kbdRowLocked = true;
        kbdCol = 0;
      } else {
        char ch = KBD_ROWS[kbdRow][kbdCol];
        if (ch == '<') {
          // Commit text or save NodeID
          if (kbdEditNodeId) {
            if (strlen(kbdBuffer) > 0) {
              memset(myNodeName, 0, sizeof(myNodeName));
              strncpy(myNodeName, kbdBuffer, 6);
              prefs.putString("nodeName", myNodeName);
              animDither();
            }
            currentScreen = SCR_SETTINGS;
          } else if (strlen(kbdBuffer) > 0) {
            animToss();
            sendUserMessage(PKT_TEXT, selectedDstId, kbdBuffer);
            currentScreen = SCR_TX_STATUS;
          }
        } else {
          size_t maxLen = kbdEditNodeId ? 6 : 16;
          size_t len = strlen(kbdBuffer);
          if (len < maxLen) {
            kbdBuffer[len] = (ch == '_') ? ' ' : ch;
            kbdBuffer[len + 1] = '\0';
          }
          kbdRowLocked = false;
        }
      }
      break;
    }

    case SCR_PRESETS:
      animToss();
      sendUserMessage(PKT_PRESET, 0xFFFF, PRESET_LIST[menuCursor], menuCursor);
      currentScreen = SCR_TX_STATUS;
      break;

    case SCR_SETTINGS:
      if (settingsCursor == 0) {
        cfgSleepIdx = (cfgSleepIdx + 1) % 4;
      } else if (settingsCursor == 1) {
        cfgTtl = (cfgTtl % 5) + 1;
      } else if (settingsCursor == 2) {
        cfgLongRange = !cfgLongRange;
        applyRadioProfile();
        animDither();
      } else if (settingsCursor == 3) {
        cfgBuzzer = !cfgBuzzer;
      } else if (settingsCursor == 4) {
        kbdEditNodeId = true;
        kbdRow = 0; kbdCol = 0; kbdRowLocked = false;
        strncpy(kbdBuffer, myNodeName, 6);
        kbdBuffer[6] = '\0';
        currentScreen = SCR_KEYBOARD;
      } else if (settingsCursor == 5) {
        setWifiPortal(!wifiApEnabled);
      } else if (settingsCursor == 6) {
        rtSent = 0; rtAcked = 0; rtLastPingMs = 0;
        currentScreen = SCR_RANGETEST;
      }
      break;

    case SCR_INCOMING_ALERT:
      unreadFlag = false;
      currentScreen = SCR_HOME;
      break;

    default:
      break;
  }
}

void onSelectLong() {
  queueTone(1100, 70);
  if (currentScreen == SCR_KEYBOARD) {
    if (kbdEditNodeId) {
      if (strlen(kbdBuffer) > 0) {
        memset(myNodeName, 0, sizeof(myNodeName));
        strncpy(myNodeName, kbdBuffer, 6);
        prefs.putString("nodeName", myNodeName);
        animDither();
      }
      currentScreen = SCR_SETTINGS;
    } else if (strlen(kbdBuffer) > 0) {
      animToss();
      sendUserMessage(PKT_TEXT, selectedDstId, kbdBuffer);
      currentScreen = SCR_TX_STATUS;
    } else {
      currentScreen = SCR_MSG_MENU;
    }
    return;
  }
  if (!sosActive) {
    txTrackingActive = false;
    currentScreen = SCR_HOME;
  }
}

void onNextShort() {
  queueTone(1200, 30);
  switch (currentScreen) {
    case SCR_HOME:
      homeCursor = (homeCursor + 1) % 4;
      break;
    case SCR_MESH:
      sendControlPacket(PKT_ROLLCALL);
      break;
    case SCR_MSG_MENU:
      menuCursor = (menuCursor + 1) % 4;
      break;
    case SCR_TARGET_PICK: {
      uint8_t total = countActivePeers() + 1;
      targetCursor = (targetCursor + 1) % total;
      break;
    }
    case SCR_KEYBOARD:
      if (!kbdRowLocked) kbdRow = (kbdRow + 1) % 4;
      else               kbdCol = (kbdCol + 1) % 7;
      break;
    case SCR_PRESETS:
      menuCursor = (menuCursor + 1) % 6;
      break;
    case SCR_INBOX:
      inboxCursor = (inboxCursor + 1) % INBOX_SIZE;
      break;
    case SCR_RESCUE: {
      uint8_t n = countActivePeers();
      if (n > 0) rescuePeerIdx = (rescuePeerIdx + 1) % n;
      break;
    }
    case SCR_SETTINGS:
      settingsCursor = (settingsCursor + 1) % 8;
      if (settingsCursor < settingsTop) settingsTop = settingsCursor;
      if (settingsCursor >= settingsTop + 4) settingsTop = settingsCursor - 3;
      break;
    default:
      break;
  }
}

void pollButton(BtnState& b, bool isSelect, bool isRed, bool isNext) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.lastRaw) {
    b.lastDebounceMs = now;
    b.lastRaw = raw;
  }
  if ((now - b.lastDebounceMs) > 35) { // 35 ms Wi-Fi RF-ripple debounce
    if (raw != b.stableState) {
      b.stableState = raw;
      if (b.stableState == LOW) {
        lastActivityMs = now;
        b.pressStartMs = now;
        b.longHandled  = false;
      } else {
        // Released
        if (!b.longHandled) {
          if (isRed) {
            if (sosClickCount == 0 || (now - sosFirstClickMs) > 1200UL) {
              sosClickCount = 1;
              sosFirstClickMs = now;
            } else {
              sosClickCount++;
            }
            if (sosClickCount >= 3) {
              sosClickCount = 0;
              onSosTripleClick();
            } else {
              onRedSingleClick();
            }
          } else if (isSelect) {
            onSelectShort();
          } else if (isNext) {
            onNextShort();
          }
        }
      }
    } else if (b.stableState == LOW) {
      if (isSelect && !b.longHandled && (now - b.pressStartMs) >= 750UL) {
        b.longHandled = true;
        lastActivityMs = now;
        onSelectLong();
      }
      if (isNext && currentScreen == SCR_KEYBOARD &&
          (now - b.pressStartMs) >= 450UL && (now - lastTurboMs) >= 140UL) {
        lastTurboMs = now;
        onNextShort();
      }
    }
  }
}

// ======================================================================================
// 13. SMART LIGHT SLEEP ENGINE (DIO0 + BUTTON + 5-MIN RTC HEARTBEAT)
// ======================================================================================
void serviceLightSleep() {
  uint32_t timeout = SLEEP_OPTIONS_MS[cfgSleepIdx];
  if (timeout == 0 || wifiApEnabled || sosActive || currentScreen == SCR_RANGETEST) return;
  if ((millis() - lastActivityMs) < timeout) return;

  animSleep();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  LoRa.receive();

  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(LORA_DIO0_PIN), 1);
  uint64_t btnMask = (1ULL << BTN_SOS_PIN) | (1ULL << BTN_SELECT_PIN) | (1ULL << BTN_NEXT_PIN);
  esp_sleep_enable_ext1_wakeup(btnMask, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_sleep_enable_timer_wakeup(300000000ULL); // 5-minute silent heartbeat wake

  esp_light_sleep_start();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    sendControlPacket(PKT_HEARTBEAT);
    lastHeartbeatMs = millis();
  } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    handleIncomingLoRa(true);
  } else {
    display.ssd1306_command(SSD1306_DISPLAYON);
    animGlitch();
    lastActivityMs = millis();
  }
}

// ======================================================================================
// 14. SETUP & NON-BLOCKING MAIN LOOP
// ======================================================================================
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

  pinMode(BTN_SOS_PIN, INPUT_PULLUP);
  pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
  pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  ledcAttach(BUZZER_PIN, 2000, 8);
  analogReadResolution(12);

  // Load persisted Node Callsign from NVS Flash
  prefs.begin("lastlink", false);
  snprintf(myNodeName, sizeof(myNodeName), "NODE%02d", MY_NODE_ID);
  String savedName = prefs.getString("nodeName", myNodeName);
  savedName.toCharArray(myNodeName, 7);

  // Initialize I2C OLED
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    animGlitch();
  }

  // Initialize Optional NEO-6M GNSS UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize VSPI LoRa Transceiver (433 MHz)
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (LoRa.begin(LORA_FREQ_HZ)) {
    applyRadioProfile();
    LoRa.onReceive([](int) { loraIrqFlag = true; });
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0_PIN), onLoraDio0, RISING);
  }

  lastActivityMs  = millis();
  lastHeartbeatMs = millis();
  queueTone(1600, 60, 2200, 80);
  sendControlPacket(PKT_ROLLCALL);
}

void loop() {
  uint32_t now = millis();

  serviceGps();
  serviceAudio();
  if (wifiApEnabled) server.handleClient();

  // Poll 3 tactile buttons
  pollButton(btnSos,    false, true,  false);
  pollButton(btnSelect, true,  false, false);
  pollButton(btnNext,   false, false, true);

  // Process incoming LoRa IRQ
  if (loraIrqFlag) {
    loraIrqFlag = false;
    handleIncomingLoRa(false);
  }

  // SOS Countdown -> Infinite Beacon Pulse Engine (every 3000 ms with fresh msgId)
  if (sosActive) {
    if (currentScreen == SCR_SOS_COUNTDOWN && (now - sosCountdownStart) >= 5000UL) {
      sosBaseMsgId   = localMsgSeq;
      sosLastPulseMs = 0;
      currentScreen  = SCR_TX_STATUS;
    }
    if (currentScreen == SCR_TX_STATUS && (now - sosLastPulseMs) >= 3000UL) {
      sosLastPulseMs = now;
      sosPulseCount++;
      sendUserMessage(PKT_SOS, 0xFFFF, "EMERGENCY SOS!");
      queueTone(2800, 120, 1900, 120);
    }
  }

  // Automatic Retransmission for Tracked Unicast/Broadcast Messages
  if (txTrackingActive && !sosActive && txStage < 3 && trackedRetries < 3) {
    if ((now - trackedLastTxMs) >= 2500UL) {
      trackedRetries++;
      trackedLastTxMs = now;
      transmitPacket(trackedPkt);
    }
  }

  // Automated Judge Range Test Ping Loop (every 2500 ms)
  if (currentScreen == SCR_RANGETEST && (now - rtLastPingMs) >= 2500UL) {
    rtLastPingMs = now;
    rtSent++;
    sendControlPacket(PKT_RANGETEST);
  }

  // Periodic 5-minute Silent Mesh Heartbeat
  if ((now - lastHeartbeatMs) >= 300000UL) {
    lastHeartbeatMs = now;
    sendControlPacket(PKT_HEARTBEAT);
  }

  // Refresh OLED UI at ~20 Hz
  static uint32_t lastUiMs = 0;
  if ((now - lastUiMs) >= 50UL) {
    lastUiMs = now;
    renderScreen();
  }

  serviceLightSleep();
}
