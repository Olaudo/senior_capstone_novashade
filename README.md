IGNORE NNF FILE
<div align="center">

<img src="https://img.shields.io/badge/Platform-ESP32-blue?style=for-the-badge&logo=espressif" />
<img src="https://img.shields.io/badge/Language-C%2B%2B%20%7C%20HTML%2FJS-purple?style=for-the-badge" />
<img src="https://img.shields.io/badge/Power-12V%20Automotive-green?style=for-the-badge" />
<img src="https://img.shields.io/badge/License-MIT-orange?style=for-the-badge" />
<img src="https://img.shields.io/badge/Status-Validated-brightgreen?style=for-the-badge" />

# ❄️ NovaShade
### Embedded Thermal Regulation System for Smart Defroster Applications
#### *PWM-Based Windshield Pre-Heating with RTC Scheduling and Mobile Web App*

**Team 10 · Georgia Southern University · Department of Electrical and Computer Engineering**  
Olaudo Victor-Ofoegbu · Jalen Mordica · Iniubong Unah  
*Faculty Mentors: Dr. Rocio Alba-Flores & Dr. Reza Jalilzadeh Hamidi*

---

</div>

## 📌 Overview

**NovaShade** is a fully autonomous, plug-and-play windshield pre-heating system designed to eliminate the need for cold-weather engine idling. The system integrates direct-contact surface temperature sensing, voltage-monitored power delivery, and microcontroller-based PWM control into a single embedded platform — delivering intelligent, scheduled defrosting from a standard 12V automotive outlet, with zero vehicle modification required.

Unlike all existing aftermarket defrosters, which operate at fixed power with no thermal feedback and no scheduling capability, NovaShade applies the right amount of heat at the right time, only when conditions are electrically safe, and turns itself off automatically when the target temperature is reached.

> **2026 Student Research Symposium — Georgia Southern University, Statesboro, GA**

---

## ⚡ Key Features

| Feature | Description |
|---|---|
| **RTC-Scheduled Pre-Heating** | User sets departure time via mobile app; DS3231 RTC wakes system 45 min before |
| **Closed-Loop PWM Control** | Piecewise thermal zone mapping dynamically adjusts heater duty cycle |
| **Ramp-Rate Management** | Duty cycle steps incrementally to eliminate inrush current transients on 12V bus |
| **Voltage-Gated Safety** | INA226 reads supply voltage before every cycle; heater never activates under unsafe conditions |
| **Dual Thermal Protection** | KSD301 mechanical cutoff + firmware 67°F ceiling = redundant failsafe |
| **Deep Sleep Architecture** | ESP32 sleeps between cycles; negligible parasitic battery drain |
| **Mobile Web App (PWA)** | Served directly from ESP32; no app store, no installation, works on any browser |
| **AP + STA Dual WiFi** | Hotspot always available; optionally joins home network simultaneously |
| **100W Heater Control** | Full-range PWM over silicone heating pad from standard cigarette lighter outlet |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    12V AUTOMOTIVE SUPPLY                        │
│         Cigarette Lighter → Inline Fuse → TVS Diode            │
│                         → Buck Converter (5V)                  │
└────────────────────────────┬────────────────────────────────────┘
                             │
                    ┌────────▼────────┐
                    │   ESP32-WROOM   │  ← Brain of the system
                    │   (MCU + WiFi)  │
                    └──┬──────────┬───┘
                       │          │
          ┌────────────▼──┐   ┌───▼──────────────┐
          │  I2C Bus 0    │   │   I2C Bus 1       │
          │  GPIO 21/22   │   │   GPIO 18/19      │
          │               │   │                   │
          │ ┌───────────┐ │   │ ┌───────────────┐ │
          │ │  DS3231   │ │   │ │   MCP9808     │ │
          │ │    RTC    │ │   │ │  Temp Sensor  │ │
          │ └───────────┘ │   │ └───────────────┘ │
          │ ┌───────────┐ │   └───────────────────┘
          │ │  INA226   │ │
          │ │  V/I/P    │ │
          │ └───────────┘ │
          └───────────────┘
                  │
          ┌───────▼──────────────────────────────┐
          │         GPIO 27 — PWM Output          │
          │    MOSFET Trigger Module (5V–36V)     │
          │         ↓ PWM-controlled gate         │
          │      100W Silicone Heating Pad        │
          │     (mounted on windshield glass)     │
          └───────────────────────────────────────┘
```

### Firmware State Machine

```
COLD BOOT / DEEP SLEEP WAKE
         │
         ▼
  [Read INA226 Voltage]
         │
    ┌────┴────┐
    │ Safe?   │
    └────┬────┘
   No ◄──┘  └──► Yes
    │              │
[Sleep]      [Read MCP9808]
                   │
         [Piecewise Zone Mapping]
           Zone 1: Full power  (< 35°F)
           Zone 2: Ramp down   (35–50°F)
           Zone 3: Minimum     (50–67°F)
                   │
           [Ramp Controller]
         Step duty cycle gradually
         → no inrush transient
                   │
           [Heating Loop]
         Poll sensors every 200ms
         Adjust duty cycle live
                   │
              ┌────┴────┐
              │ ≥ 67°F? │
              └────┬────┘
         Yes ◄─────┘
              │
       [Heater OFF]
       [Schedule next RTC alarm]
       [Deep Sleep]
```

---

## 🔧 Hardware

### Components

| # | Component | Purpose | Cost |
|---|---|---|---|
| 1 | ESP32-WROOM-32 Development Board | MCU, WiFi, PWM, deep sleep | $19.98 |
| 2 | MOSFET Trigger Module (5V–36V, 15A) | High-current PWM switching | $7.99 |
| 3 | MCP9808 I²C Temperature Sensor | Direct windshield surface temp | $7.64 |
| 4 | KSD301 Thermal Switch (75°C, 10A) | Mechanical backup cutoff | $11.59 |
| 5 | INA226 Voltage/Current/Power Monitor | Voltage gating + power telemetry | $12.99 |
| 6 | 100W Silicone Heating Pad | Heating element | $27.59 |
| 7 | DC-DC 5V Buck Converter | MCU power supply from 12V rail | $9.99 |
| 8 | DS3231 RTC Module | Scheduled wake alarms | $7.99 |
| 9 | 12V Fused Cigarette Lighter Plug | Vehicle power interface | $6.39 |
| 10 | 14 AWG Stranded Copper Wire (75 ft) | Heater power wiring | $31.99 |
| 11 | Silicone Rubber Sheet (1/8") | Thermal insulation | $14.99 |
| 12 | Practice Windshield Glass | Testing substrate | $20.95 |
| | | **Total** | **$180.08** |

### Pin Mapping

```
ESP32 GPIO    Function
─────────────────────────────
GPIO 21       I2C Bus 0 — SDA  (RTC + INA226)
GPIO 22       I2C Bus 0 — SCL  (RTC + INA226)
GPIO 18       I2C Bus 1 — SDA  (MCP9808)
GPIO 19       I2C Bus 1 — SCL  (MCP9808)
GPIO 27       MOSFET PWM Output
GPIO 33       RTC Interrupt (EXT0 wake)
GPIO 2        Status LED
```

### Input Protection Stage

```
CIG LIGHTER (+12V)
      │
   [3A FUSE]           ← inline, series on + rail
      │
      ├── [SMBJ24A TVS] ── GND   ← clamps load-dump spikes
      │
      ├── [470µF 25V]  ── GND    ← bulk smoothing cap
      │
      ├── [100nF ceramic] ── GND ← HF decoupling
      │
   LM2596 VIN → 5V out → ESP32
```

---

## 💻 Software

### Repository Structure

```
NovaShade/
├── firmware/
│   └── NovaShade_v3.ino       # Main firmware (ESP32 Arduino)
├── web/
│   └── app.html               # PWA source (embedded in firmware as PROGMEM)
├── hardware/
│   ├── schematic.pdf          # Circuit schematic
│   └── pcb/                   # KiCad PCB files
├── docs/
│   ├── final_report.pdf
│   ├── sustainability.pdf
│   └── poster.pdf
└── README.md
```

### Dependencies

Install via Arduino Library Manager:

```
RTClib                  (Adafruit)
Adafruit_INA219         (Adafruit)  ← Note: code uses INA226 on board
Adafruit_MCP9808        (Adafruit)
ArduinoJson             (Benoit Blanchon)
Preferences             (built-in ESP32 core)
```

### API Endpoints

The ESP32 WebServer exposes a lightweight REST API:

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/` | Serves the full PWA HTML |
| `GET` | `/api/status` | Live sensor data + system state (JSON) |
| `POST` | `/api/departure` | Set departure epoch for RTC scheduling |
| `POST` | `/api/settime` | Sync RTC to phone's current time |
| `POST` | `/api/wifi` | Save home WiFi credentials to flash |
| `POST` | `/api/cancel` | Emergency heater shutdown |

**Example `/api/status` response:**
```json
{
  "tempC": 3.25,
  "tempF": 37.85,
  "targetC": 19.4,
  "voltage": 12.6,
  "current": 7.8,
  "pwmPct": 86.3,
  "currentDuty": 220,
  "targetDuty": 220,
  "rampStep": 1,
  "status": "heating",
  "phase": "heating",
  "nextWake": "07:15",
  "departureTime": "08:00",
  "rtcTime": "27 Apr 2026 06:15:00",
  "rtcEpoch": 1745719200,
  "staConnected": true,
  "staSSID": "HomeNetwork",
  "staIP": "192.168.1.42"
}
```

---

## 📱 Mobile Web App

The NovaShade PWA is compiled directly into the firmware as a `PROGMEM` string — no SD card, no SPIFFS, no external server required.

**Connect in two ways:**

| Mode | How | URL |
|---|---|---|
| **Hotspot** | Connect phone to `NovaShade` WiFi (pass: `novashade`) | `http://192.168.4.1` |
| **Home WiFi** | Enter home network credentials in app → device joins both networks | `http://<home-ip>` |

**App features:**
- Live surface temperature with color-coded thermal state
- Real-time heater PWM output bar + duty cycle detail
- Voltage, current, and power metrics with fault coloring
- Departure time picker with countdown timer
- One-tap RTC clock sync from phone
- Next scheduled wake display
- Home WiFi credential management
- Cancel & Shutdown emergency control

---

## 📊 Results

| Test | Target | Result |
|---|---|---|
| PWM ramp transitions | Spike-free under 100W load | ✅ Confirmed via oscilloscope |
| Thermal zone progression | 3 zones traversed in sequence | ✅ Confirmed on real glass |
| Automatic shutoff | Consistent trigger above 67°F | ✅ Confirmed all test runs |
| Voltage-gated protection | Skip heat cycle if V out of range | ✅ Confirmed all test runs |
| RTC scheduling | Wake at correct alarm time | ✅ Confirmed all test runs |
| End-to-end operation | Full cycle app → heat → sleep | ✅ Confirmed |

---

## 🌿 Sustainability

Vehicle cold-weather idling wastes **6 billion gallons of fuel** and generates **58 million metric tons of CO₂** annually in the US. A single NovaShade unit eliminates approximately **1,311 g CO₂ per defrost cycle** vs. idling. At 1 million units, that's **78,660 metric tons of CO₂ saved per winter season**.

- Operates from existing 12V battery — no new energy infrastructure
- PWM control minimizes switching losses at full 100W load
- Deep sleep draws negligible current between cycles
- Plug-and-play design extends usable life of vehicles without OEM defrosters

---

## 🔒 Standards Compliance

| Standard | Relevance |
|---|---|
| ISO 26262:2018 | Functional safety — dual thermal cutoff strategy |
| ISO 16750-3:2012 | Automotive environmental durability |
| IEC 60364 | Low-voltage wiring and overcurrent protection |
| CISPR 25:2016 | EMI from PWM/MOSFET switching |
| IEEE 802.11 | Wi-Fi (ESP32 is FCC Part 15 certified) |
| IEEE 802.15.1 | Bluetooth (optional interface) |
| RoHS Directive | All components confirmed hazardous-substance free |

---

## 🚀 Getting Started

### 1. Flash the Firmware

```bash
# Clone the repository
git clone https://github.com/your-team/novashade.git
cd novashade/firmware

# Open NovaShade_v3.ino in Arduino IDE
# Board: ESP32 Dev Module
# Upload Speed: 921600
# Flash Size: 4MB
```

### 2. Wire the Hardware

Follow the pin mapping table above. Key notes:
- **Do NOT** `pinMode()` or `digitalWrite()` on GPIO 27 — it's managed entirely by `ledcWrite()`
- The MOSFET uses **inverted PWM**: `255 = OFF`, `0 = full power`
- Always install the inline fuse before connecting to the vehicle

### 3. Connect and Configure

1. Power the device from a 12V source
2. Connect your phone to the **NovaShade** WiFi network (`novashade`)
3. Open `http://192.168.4.1` in any browser
4. Tap **Sync Phone Time to Device** first
5. Set your departure time
6. Mount the heating pad on the windshield glass and you're done

---

## ⚠️ Known Limitations

- Tested in controlled lab conditions; extreme cold (< -20°C) not yet validated at scale
- Heating pad covers a fixed area; different windshield geometries not accounted for
- WiFi radio on ESP32 is sensitive to automotive voltage spikes — input protection stage is mandatory
- Current hardware recyclability estimated at 20–30% due to bonded silicone/microchip materials

---

## 🔭 Future Work

- [ ] Weather API integration for automatic activation based on forecast
- [ ] PCB enclosure with automotive-grade connectors and potting compound
- [ ] Multi-zone heating with independently controlled pad segments
- [ ] OBD-II integration to correlate battery state-of-charge before activation
- [ ] Solar trickle charging for engine-off overnight pre-heat scenarios

---

## 👥 Team

| Name | Role |
|---|---|
| **Olaudo Victor-Ofoegbu** | Firmware architecture, PWM control, hypothesis & validation |
| **Jalen Mordica** | Hardware design, circuit protection, PCB layout |
| **Iniubong Unah** | Mobile web app, WiFi stack, system integration |

---

## 📚 References

1. S. Wang et al., "Experimental study of icing and deicing situations of different wettability surfaces composite electric heating," *Appl. Therm. Eng.*, vol. 238, 2023. [DOI](https://doi.org/10.1016/j.applthermaleng.2023.122045)
2. C. Sun et al., "An intelligent heating system based on the Internet of Things and STM32 microcontroller," *Energy Inform.*, vol. 7, Apr. 2024. [DOI](https://doi.org/10.1186/s42162-024-00326-2)
3. U.S. Dept. of Energy, "Which Is Greener: Idle, or Stop and Restart?" Argonne National Laboratory. [Link](https://afdc.energy.gov/files/u/publication/which_is_greener.pdf)

---

## 📬 Contact

| Name | Email |
|---|---|
| Olaudo Victor-Ofoegbu | ov00422@georgiasouthern.edu |
| Jalen Mordica | jm50671@georgiasouthern.edu |
| Iniubong Unah | iu00226@georgiasouthern.edu |

---

<div align="center">

*Built at Georgia Southern University · 2025–2026*  
*Department of Electrical and Computer Engineering*

</div>
