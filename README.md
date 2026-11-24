# LASTLINK — Decentralized 433 MHz LoRa Mesh Handheld Transceiver

**Symmetric, zero-infrastructure 433 MHz LoRa emergency mesh communication & triage system built on the ESP32.**

> 🚀 **[Launch Interactive Browser Digital Twin (Google Sites)](PASTE_YOUR_GOOGLE_SITES_URL_HERE)**  
> *Test 433 MHz obstacle attenuation, multi-hop `[SENT] → [RELAY] → [ACK]` routing, the 12-state OLED handset, and the `192.168.4.1` triage portal directly in your browser.*

---

## 📂 Repository Map

| File | Description |
| :--- | :--- |
| **[Interactive Digital Twin]([PASTE_YOUR_GOOGLE_SITES_URL_HERE](https://sites.google.com/view/lastlinkdt/full-page-embed))** | Live web-based 433 MHz RF terrain & 12-state OLED firmware simulator |
| `LASTLINK.ino` / `LASTLINK.cpp` | Unified C++ firmware (Arduino-ESP32 Core 3.x) for all symmetric handheld nodes |
| `LASTLINK.md` | Single Source of Truth (SSOT v3.0) hardware pinout, protocol & UI state specification |
| `LASTLINK_Digital_Twin.html` | Standalone single-file source code for the browser Digital Twin workbench |

---

## ⚙️ Core System Highlights

* **100% Symmetric Peer-to-Peer Topology:** Every handheld unit runs identical firmware and acts simultaneously as a field communicator, an autonomous multi-hop relay, and an on-demand `192.168.4.1` Wi-Fi triage gateway.
* **44-Byte Authenticated Binary Protocol:** Packs fixed-point `latE7`/`lonE7` GPS coordinates (`1.1 cm` resolution), hop tracking, and a keyed 16-bit FNV-1a MAC (`0xA55A9C39`) into a strict 44-byte frame with zero floating-point overhead.
* **Controlled Flooding & Light Sleep:** Uses a 32-entry duplicate ring buffer, randomized `60–240 ms` anti-collision relay backoff, and `~0.8 mA` Light Sleep with hardware `DIO0` interrupt wakeups.
* **Dual-Mode Rescue Radar:** Automatically switches between outdoor GPS Haversine distance/bearing and an indoor 28-sample RSSI oscilloscope sparkline for RF-only units.

---

## 🛠️ Hardware Stack

* **MCU:** ESP32-WROOM-32 / ESP32-WROOM-DA
* **RF Transceiver:** RFM96W / SX1278 (433 MHz ISM, `SF7` / `SF10`, `+20 dBm`, $\lambda/4$ helical antenna)
* **Display & UI:** SSD1306 `128×64` I2C OLED + 3-button Wi-Fi-safe tactile interface (`GPIO 32 / 4 / 15`) + LEDC passive buzzer (`GPIO 13`)
* **GNSS (Optional):** NEO-6M UART2 module (`GPIO 26 / 27`, auto-detected at runtime)
* **Enclosure:** Custom Onshape 3D-printed chassis with 3 mm laser-cut acrylic lid
