# LASTLINK — Decentralized 433 MHz LoRa Mesh Handheld Transceiver

![Platform](https://img.shields.io/badge/Platform-ESP32--WROOM--32-blue?style=flat-square&logo=espressif)
![Radio](https://img.shields.io/badge/Radio-433%20MHz%20LoRa%20(SX1278%20%2F%20RFM96W)-orange?style=flat-square)
![GNSS](https://img.shields.io/badge/GNSS-NEO--6M%20UART2%20(Auto--Detect)-green?style=flat-square)
![Security](https://img.shields.io/badge/Auth-Keyed%20FNV--1a%20MAC-red?style=flat-square)
![License](https://img.shields.io/badge/Status-Field%20Verified%20SSOT%20v3.0-brightgreen?style=flat-square)

**LASTLINK** is an infrastructure-independent, peer-to-peer emergency mesh communication transceiver engineered for disaster response, search-and-rescue (SAR), and off-grid expeditions where cellular networks and internet backhaul are unavailable. 

Built around the **ESP32-WROOM-32** and **433 MHz Chirp Spread Spectrum (LoRa)** radios, every node operates as a self-contained, battery-powered handheld transceiver capable of originating alerts, autonomously relaying multi-hop mesh traffic (even while in Light Sleep), performing dual-mode GPS/RF victim localization, and hosting an offline Wi-Fi Command & Triage web portal (`192.168.4.1`).

---

## Table of Contents
1. [System Architecture & Node Symmetry](#1-system-architecture--node-symmetry)
2. [Hardware Bill of Materials & Verified Pinout](#2-hardware-bill-of-materials--verified-pinout)
3. [RF Link Budget & Antenna Engineering](#3-rf-link-budget--antenna-engineering)
4. [Over-the-Air Binary Protocol (`LoRaPacket`)](#4-over-the-air-binary-protocol-lorapacket)
5. [Mesh Routing, Reliability & Power Management](#5-mesh-routing-reliability--power-management)
6. [User Interface & Control Grammar](#6-user-interface--control-grammar)
7. [Offline Wi-Fi Command & Triage Portal (`192.168.4.1`)](#7-offline-wi-fi-command--triage-portal-19216841)
8. [Root-Cause Engineering & Hardening Log](#8-root-cause-engineering--hardening-log)
9. [Build, Flash & Field Operation Guide](#9-build-flash--field-operation-guide)

---

## 1. System Architecture & Node Symmetry

```text
+-----------------------------------------------------------------------------------+
|                        LASTLINK SYMMETRIC PEER-TO-PEER MESH                       |
|                                                                                   |
|   [NODE 01: Handheld + GPS] <=== 433 MHz LoRa ===> [NODE 02: Handheld + GPS]      |
|              ^                                                ^                   |
|              | (Multi-Hop Controlled Flooding, TTL 1..5)      |                   |
|              v                                                v                   |
|   [NODE 03: Handheld Relay] <=== 433 MHz LoRa ===> [NODE 04: Handheld + Wi-Fi AP] |
|                                                               |                   |
|                                                      (SoftAP 192.168.4.1)         |
|                                                               v                   |
|                                                    [Coordinator Smartphone/PC]    |
+-----------------------------------------------------------------------------------+
```

* **100% Symmetric Peer-to-Peer Topology:** Unlike star-topology LoRaWAN systems that fail if a central gateway goes down, every **LASTLINK** device is a portable, battery-powered **Mobile Field / Handheld Unit**. Any node can originate alerts, forward neighbor packets in the background, or activate its local Wi-Fi Command Portal on demand.
* **Unified Firmware & Runtime Sensor Auto-Detection:** All devices run the identical firmware codebase (differentiated only by `#define MY_NODE_ID 1..4`). During runtime, `serviceGps()` continuously monitors `UART2`:
  * **GPS-Equipped Units:** Automatically lock NMEA streams via `TinyGPSPlus`, embed high-precision fixed-point coordinates into outgoing packets, and enable **GPS Vector Tracking** (Haversine distance + compass bearing).
  * **Non-GPS Units:** Operate seamlessly using RF-only localization (**28-sample RSSI oscilloscope radar**) while still receiving, displaying, and relaying incoming GPS coordinates from GNSS-equipped peers.

---

## 2. Hardware Bill of Materials & Verified Pinout

### Bill of Materials (Per Node)
* **Microcontroller:** ESP32-WROOM-32 Development Board (30-pin or 38-pin)
* **LoRa Transceiver:** HopeRF RFM96W or Semtech SX1278 (433 MHz SPI module)
* **Display:** 0.96" SSD1306 Monochrome OLED (`128 × 64`, I2C)
* **GNSS Module (Optional/Modular):** u-blox NEO-6M GPS with ceramic patch antenna
* **User Inputs:** 3× Tactile Push-Buttons (`RED/SOS`, `SELECT`, `NEXT`)
* **Audio/Visual Feedback:** 3-pin Passive Buzzer module + 5mm Status LED (`220 Ω` current-limiting resistor)
* **Power & Telemetry:** 18650 Li-Ion Cell + TP4056/Boost + `100 kΩ / 100 kΩ` voltage divider + `100 µF–470 µF` electrolytic decoupling capacitor

### Verified Wi-Fi-Safe Pinout Table

| Subsystem | Module Pin | ESP32 GPIO | Bus / Electrical Specification |
| :--- | :--- | :--- | :--- |
| **LoRa Transceiver** *(RFM96W / SX1278)* | `VCC` / `GND` | `3.3V` / `GND` | Place **100 µF – 470 µF capacitor** across `3.3V` & `GND` |
| | `SCK` | `GPIO 18` | Hardware VSPI Clock |
| | `MISO` | `GPIO 19` | Hardware VSPI MISO |
| | `MOSI` | `GPIO 23` | Hardware VSPI MOSI |
| | `NSS` (`CS`) | `GPIO 5` | SPI Chip Select |
| | `RST` | `GPIO 14` | Hardware Reset |
| | `DIO0` (`G0`) | `GPIO 2` | Packet RX/TX IRQ + Light Sleep external wakeup trigger |
| | `ANT` | *Antenna* | **17.3 cm** $\lambda/4$ straight monopole *(or ~23 cm pencil helix)* |
| **OLED Display** *(0.96" SSD1306)* | `VCC` / `GND` | `3.3V` / `GND` | I2C Address `0x3C`, Fast-Mode Clock `400 kHz` |
| | `SDA` / `SCL` | `GPIO 21` / `GPIO 22` | Hardware I2C Bus |
| **Tactile Buttons** *(Active LOW)* | `BTN_SOS` (Red) | `GPIO 32` | Wi-Fi safe (`INPUT_PULLUP`, ADC1 domain) |
| | `BTN_SELECT` | `GPIO 4` | Wi-Fi safe (`INPUT_PULLUP`, pure digital) |
| | `BTN_NEXT` | `GPIO 15` | Wi-Fi safe (`INPUT_PULLUP`; do not hold LOW during reset) |
| **Audio & Visuals** | `BUZZER` (`I/O`) | `GPIO 13` | 3-pin passive buzzer (`VCC` $\rightarrow$ `3.3V`/`5V`), `ledcAttach()` PWM |
| | `STATUS_LED` | `GPIO 33` | Anode via `220 Ω` resistor to `GPIO 33`, cathode to `GND` |
| **Battery Sense** | `BAT_ADC` | `GPIO 34` | Midpoint of `100 kΩ + 100 kΩ` divider (`0–4.2V` $\rightarrow$ `0–2.1V`) |
| **GNSS Receiver** *(NEO-6M, Auto-Detect)* | `TX` $\rightarrow$ `RX2` | `GPIO 26` *(or `16`)* | `9600 baud` NMEA stream (`UART2` RX) |
| | `RX` $\leftarrow$ `TX2` | `GPIO 27` *(or `17`)* | `9600 baud` UBX command line (`UART2` TX) |

---

## 3. RF Link Budget & Antenna Engineering

* **Carrier Frequency:** $f = 433\text{ MHz}$ ($\lambda = \frac{c}{f} = \frac{2.998 \times 10^8}{433 \times 10^6} \approx 69.24\text{ cm}$).
* **Quarter-Wave Monopole ($\lambda/4$):** $\frac{69.24\text{ cm}}{4} = 17.31\text{ cm}$ solid-core copper wire soldered directly to the `ANT` pad (eliminating SMA connector insertion loss and matching ~36–50 $\Omega$ impedance against the PCB ground plane).
* **Compact Helical Option:** ~23 cm enameled/insulated wire wound 14–16 turns around a 7 mm form for pocketable handheld enclosures (~3–5 dB gain trade-off vs. full $\lambda/4$ whip).
* **Dual Spreading Factor Profiles:**
  * **Normal Mode (`NM` / `SF7`, `125 kHz` BW):** Low airtime (~60 ms/packet), fast mesh convergence, ideal for dense urban/indoor relays.
  * **Long-Range Mode (`LR` / `SF10`, `125 kHz` BW):** +7.5 dB additional link budget (~132 dB total link budget), penetrating dense foliage and multi-wall structures.

---

## 4. Over-the-Air Binary Protocol (`LoRaPacket`)

All frames use a tightly packed 44-byte binary structure (`#pragma pack(push, 1)`) protected against spoofing and noise corruption by a **16-bit keyed FNV-1a Message Authentication Code** (`MESH_SECRET = 0xA55A9C39`).

```text
Offset:  0      1       3           10      12      14        15    16     17       18        22        26         42
       +------+-------+------------+-------+-------+---------+-----+------+--------+---------+---------+----------+---------+
Field: | type | srcId | srcName[7] | dstId | msgId | origTtl | ttl | code | hasGps |  latE7  |  lonE7  | text[17] | authTag |
Size:  |  1B  |  2B   |     7B     |  2B   |  2B   |   1B    | 1B  |  1B  |   1B   |   4B    |   4B    |   17B    |   2B    |
       +------+-------+------------+-------+-------+---------+-----+------+--------+---------+---------+----------+---------+
Total Frame Size: 44 Bytes
```

### Field Specifications
* **`type` (`uint8_t`):** Packet command opcode:
  * `0x01` (`PKT_SOS`): Continuous emergency beacon (re-transmits every `3000 ms` with an incremented `msgId` so multi-hop relays forward every wave and Rescue Radar updates RSSI/GPS continuously until manually aborted).
  * `0x02` (`PKT_PRESET`): One-tap standardized emergency status code (`0..5`, or `99` for automated `LAST GASP 5%!` low-battery alert).
  * `0x03` (`PKT_TEXT`): Free-form 16-character alphanumeric message.
  * `0x04` (`PKT_ACK`): End-to-end delivery receipt (`code = 0xAA` identifies Range-Test ACKs).
  * `0x06` (`PKT_RANGETEST`): Automated numbered field-test ping (`PING #N`) sent every `2.5 s`.
  * `0x55` (`PKT_HEARTBEAT`): Silent machine-to-machine telemetry beacon every `300,000 ms` (5 minutes).
  * `0xA5` (`PKT_ROLLCALL`) / `0x5A` (`PKT_PRESENT`): On-demand silent mesh discovery ping and response.
* **`dstId` (`uint16_t`):** `0xFFFF` for network-wide broadcast, or specific target `srcId` (`1..4`) for private unicast delivery.
* **`origTtl` & `ttl` (`uint8_t`):** Time-To-Live hop counter (`1..5`). Receiving nodes compute exact hop distance via $\text{Hops} = \text{origTtl} - \text{ttl} + 1$.
* **`latE7` & `lonE7` (`int32_t`):** Latitude and longitude scaled by $10^7$, achieving **~1.1 cm spatial resolution** using 32-bit integers without floating-point serialization issues.
* **`authTag` (`uint16_t`):** Folded 32-bit FNV-1a hash computed across bytes `0..41` XORed with `MESH_SECRET`.

---

## 5. Mesh Routing, Reliability & Power Management

### Controlled Flooding & Collision Avoidance
1. **32-Entry Ring-Buffer Deduplication (`dupCache`):** Every node caches `(srcId, msgId)` pairs. Duplicate frames are immediately dropped to prevent broadcast storms.
2. **Randomized CSMA-Style Relay Jitter:** When a node receives a packet with `ttl > 0` not exclusively addressed to itself, it schedules retransmission after a randomized `60–220 ms` backoff (`relayTime`).
3. **Overhear Suppression:** If the node overhears another neighbor already relaying the same `(srcId, msgId)` during its backoff window, it cancels its pending relay (`relayPending = false`), conserving airtime and battery.
4. **3-Stage Delivery Pipeline (`SCR_TX_STATUS`):** Outgoing messages track real-time progression across `[SENT]` $\rightarrow$ `[RELAY]` (triggered when the sender overhears a neighbor rebroadcast its packet) $\rightarrow$ `[ACK]` (triggered upon receiving `PKT_ACK` from the destination), with up to 3 automatic retries at `3000 ms` intervals.

### Smart Light Sleep & Silent Wakeup (`enterSmartLightSleep()`)
* After user-configurable inactivity (`15s`, `30s`, `60s`, or `Never`), the node plays its CRT power-down animation, turns off the OLED (`SSD1306_DISPLAYOFF`), leaves the LoRa transceiver in continuous `LoRa.receive()` mode (~11 mA), and puts the ESP32 into `esp_light_sleep_start()` (~0.8 mA CPU draw).
* **Hardware Wake Sources:**
  * Pressing any button (`GPIO 32`, `4`, `15` LOW) wakes the UI immediately.
  * Incoming LoRa packets assert `DIO0` (`GPIO 2` HIGH), waking the CPU to authenticate and process the frame:
    * If the packet is `PKT_ROLLCALL` (`0xA5`), `PKT_HEARTBEAT` (`0x55`), or a packet requiring relay, the node replies/forwards silently **without turning on the OLED** and returns directly to Light Sleep.
    * If the packet is an incoming message or SOS addressed to the node, the OLED powers on immediately and triggers the audio/visual alert.

---

## 6. User Interface & Control Grammar

### Tactile 3-Button Input Grammar
| Button | GPIO | Short Press | Long Press (`>750 ms`) | Multi-Press |
| :--- | :--- | :--- | :--- | :--- |
| **`RED / SOS`** | `GPIO 32` | Backspace (Keyboard) / Cancel / Dismiss | — | **3× Click (`<1.2 s`)**: Global SOS interrupt |
| **`SELECT`** | `GPIO 4` | Confirm / Toggle / Lock Row or Char | Universal **Back** / **Send & Save** | — |
| **`NEXT`** | `GPIO 15` | Advance cursor / Scroll / Cycle target | Turbo-scroll inside Keyboard (`>450 ms`) | — |

### OLED Display Architecture (`128 × 64`)
* **Persistent Top Bar:** Displays 6-character custom `myNodeName` (flashes `>>RELAY:<Name>` for `2.5 s` when forwarding traffic), mesh status badge (`[NM|2NG]` indicating `NM/LR` mode, active neighbor count, and `G` for GPS lock or `W` for Wi-Fi AP active), and battery percentage + icon.
* **6 Event-Driven Vector Mascot Animations:** `animGlitch` (Boot/Wake), `animSlide` (Menu navigation), `animToss` (Packet TX), `animSpinZoom` (ACK confirmed), `animDither` (SF toggle / Node ID save), and `animSleep` (Wink + eye droop + CRT collapse).
* **12 Screen States:**
  1. `SCR_HOME`: 2×2 Quadrant Grid (`MESH NET`, `MESSAGES`, `RESCUE`, `SETTINGS`).
  2. `SCR_MESH`: Broadcasts `PKT_ROLLCALL` on entry; lists active neighbors (`Name | RSSI | GPS/--- | Age`).
  3. `SCR_MSG_MENU`: `1. Write Msg (Keybd)`, `2. Quick Presets`, `3. Inbox & Triage`, `4. < Back Home`.
  4. `SCR_TARGET_PICK`: Select `ALL NODES (Broadcast)` (`0xFFFF`) or `PRIV -> <Name> (#ID)`.
  5. `SCR_KEYBOARD`: Two-axis 4×7 character matrix (`A–Z`, `SP`, `OK`) for 16-char messages or 6-char NVS-persisted Node IDs.
  6. `SCR_PRESETS`: `NEED HELP!`, `MEDICAL!`, `TRAPPED!`, `NEED WATER`, `NEED FOOD`, `I AM SAFE`.
  7. `SCR_INBOX`: 16-entry circular triage log displaying sender, RSSI, hop count, decimal GPS coordinates, and payload.
  8. `SCR_RESCUE`: Dual-mode victim locator:
     * **GPS Vector Mode** (both nodes locked): Displays exact Haversine distance (`m`/`km`), cardinal direction (`N..NW`), and compass bearing (`0–359°`).
     * **Indoor RSSI Radar Mode** (no GPS lock): Displays path-loss distance estimate (`~Xm`), `^HOT`/`vCLD` signal trend, SNR, and a live 28-sample scrolling oscilloscope sparkline.
  9. `SCR_SETTINGS`: 8-item scrolling menu (`Sleep`, `TTL`, `Range SF7/SF10`, `Buzzer`, `NodeID`, `WiFiAP`, `Run Range Test`, `GPS Status`).
  10. `SCR_RANGETEST`: Live automated `PING` logger computing `SENT`, `ACK`, `DELIVERY RATE (%)`, `RSSI`, and `SNR`.
  11. `SCR_SOS_COUNTDOWN` & `SCR_TX_STATUS`: 5-second abortable SOS countdown + 3-stage graphical delivery bar (`[SENT] — [RELAY] — [ACK]`).
  12. `SCR_INCOMING_ALERT`: High-contrast inverted OLED banner with dual-tone Morse siren (`... -`).

---

## 7. Offline Wi-Fi Command & Triage Portal (`192.168.4.1`)

Toggling `Settings -> 6. WiFiAP` to `ON` turns any handheld node into an offline field command server without requiring internet access:
* **Access Point:** Open SSID `LASTLINK_<NodeName>`, throttled to `WIFI_POWER_11dBm` for low power draw and RF coexistence.
* **Web Dashboard (`http://192.168.4.1`):** Auto-refreshes every 5 seconds and provides:
  * Live node battery, spreading factor, and GNSS lock status.
  * **Web-to-LoRa Dispatch Form:** Broadcast 16-character emergency instructions from a smartphone/laptop keyboard across the 433 MHz mesh.
  * **Discovered Mesh Nodes Table:** Live RSSI, battery %, last-seen age, and clickable coordinate links (`https://maps.google.com/?q=lat,lon`).
  * **Emergency Triage Log:** Chronological feed highlighting `PKT_SOS` alerts in red with hop counts and map links.

---

## 8. Root-Cause Engineering & Hardening Log

During hardware integration and field testing on the ESP32-WROOM-32, three critical silicon and protocol edge cases were diagnosed and resolved:

1. **ESP32 ADC2 vs. Wi-Fi Contention & 3.3V Brownout Suppression:**
   * *Symptom:* Activating `WiFi.softAP()` caused buttons wired to `GPIO 25/26/27` (ADC2) to lock up or phantom-scroll through Settings due to Wi-Fi driver ADC2 ownership and 200 mA RF startup ripples.
   * *Resolution:* Migrated all tactile buttons to Wi-Fi-safe pins (`GPIO 32`, `GPIO 4`, `GPIO 15`), throttled SoftAP transmit power to `WIFI_POWER_11dBm`, implemented a 35 ms noise-immune debounce guard with post-AP state flushing, and added bulk decoupling capacitance (`100–470 µF`) across `3.3V` and `GND`.
2. **ESP32 Core 3.x Hardware PWM Audio Stability:**
   * *Symptom:* Dynamic `tone()`/`noTone()` calls repeatedly allocated and detached LEDC timers during rapid UI clicks, introducing interrupt jitter alongside VSPI LoRa transactions.
   * *Resolution:* Pre-bound a dedicated hardware PWM channel once in `setup()` via `ledcAttach(BUZZER_PIN, 2000, 8)` and drove non-blocking frequency transitions via `ledcWriteTone()`.
3. **Continuous SOS Beacon vs. Duplicate Cache Filtering:**
   * *Symptom:* Repeating an SOS alert with a static `msgId` caused intermediate relay nodes to drop subsequent pulses as duplicates.
   * *Resolution:* Configured `PKT_SOS` to increment `msgId` on every 3-second pulse while preserving ACK recognition across sequential pulse IDs so every beacon wave propagates through multi-hop relays and updates Rescue Radar telemetry in real time.

---

## 9. Build, Flash & Field Operation Guide

### Required Arduino Libraries
* `LoRa` by Sandeep Mistry
* `Adafruit GFX Library` & `Adafruit SSD1306`
* `TinyGPSPlus` by Mikal Hart
* Built-in ESP32 Core libraries (`Wire`, `SPI`, `WiFi`, `WebServer`, `Preferences`, `esp_sleep`)

### Flashing Procedure
1. Open the unified firmware sketch in Arduino IDE (`ESP32 Dev Module`, `115200` baud).
2. Set `#define MY_NODE_ID` to a unique integer (`1`, `2`, `3`, or `4`) for each unit before uploading.
3. Ensure the **17.3 cm antenna** is soldered to `ANT` **before** powering on the board to prevent RF power amplifier damage.
4. *(Optional)* Customize each unit's 6-character callsign directly on the device via `Settings -> 5. NodeID` (saved permanently to NVS Flash).

### Quick Field Verification Checklist
* **Mesh Discovery:** Open `MESH NET` on Node 1 $\rightarrow$ sends `PKT_ROLLCALL` $\rightarrow$ Node 2 replies silently and appears with live RSSI (`dBm`).
* **Tracked Messaging:** Send a preset or custom message $\rightarrow$ verify `[SENT] -> [RELAY] -> [ACK]` confirmation and spin-zoom animation.
* **Global SOS:** Triple-click `RED` (`GPIO 32`) from any screen $\rightarrow$ verify 5-second countdown, siren/inverted alert on peers, and single-click dismissal.
* **GNSS & Rescue:** Take a GPS-equipped node outdoors until the `G` badge appears in the top bar $\rightarrow$ open `RESCUE` on a peer node to verify live Haversine distance (`m`/`km`) and compass bearing (`deg`).
