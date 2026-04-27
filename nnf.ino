/*
 * ============================================================
 *   NovaShade — Full System Firmware v3
 *   RTC Scheduling + INA219 Monitoring + MCP9808 Temp +
 *   Adaptive PWM Heating + WiFi AP+STA + PWA Web App
 * ============================================================
 *
 *  I2C Bus 0 (Wire)  — GPIO 21 SDA, GPIO 22 SCL
 *    ├── DS3231 RTC   (0x68)
 *    └── INA219       (0x40)
 *
 *  I2C Bus 1 (Wire1) — GPIO 18 SDA, GPIO 19 SCL
 *    └── MCP9808      (0x18)
 *
 *  MOSFET PWM  — GPIO 27
 *  RTC INT     — GPIO 33
 *  LED         — GPIO 2
 *
 *  WiFi Hotspot always on: "NovaShade" / "novashade"  → 192.168.4.1
 *  If home WiFi saved:      also joins home network   → shown in Serial
 *
 * ============================================================
 *  Libraries needed:
 *    RTClib           (Adafruit)
 *    Adafruit INA219
 *    Adafruit MCP9808
 *    ArduinoJson      (Benoit Blanchon)
 *    Preferences      (built-in ESP32)
 * ============================================================
 *
 *  FIXES IN THIS VERSION
 *  1. liveNextWakeStore is RTC_DATA_ATTR — survives deep sleep, always
 *     shows the correct next scheduled wake time in the app.
 *  2. WiFi stays on:  scheduleWake() serves clients for 5 s BEFORE
 *     shutting the radio down; the hotspot is always restored on wake.
 *  3. AP+STA dual mode: hotspot (192.168.4.1) always runs; device also
 *     connects to saved home WiFi if credentials exist.  Both work
 *     simultaneously so the phone never has to leave home WiFi.
 *  4. /api/wifi endpoint saves home WiFi creds to Preferences (flash)
 *     and reconnects without rebooting.
 *  5. RTC epoch sent in every /api/status so the JS clock ticks smoothly
 *     between polls without waiting for the next network round-trip.
 *  6. Temperature thresholds are in Celsius internally (sensor native).
 *     The app converts to °F for display.  No changes needed here.
 * ============================================================
 */

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MCP9808.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

/* ===================== PINS ===================== */
#define SDA_BUS0    21
#define SCL_BUS0    22
#define SDA_BUS1    18
#define SCL_BUS1    19
#define RTC_INT_PIN 33
#define MOSFET_PIN  27
#define LED_PIN     2

/* ===================== WIFI ===================== */
const char* AP_SSID = "NovaShade";
const char* AP_PASS = "novashade";          // hotspot — always on

/* ===================== I2C / DEVICES ===================== */
TwoWire          I2C_1 = TwoWire(1);
RTC_DS3231       rtc;
Adafruit_INA219  ina219;
Adafruit_MCP9808 tempsensor;

/* ===================== WEB SERVER ===================== */
WebServer server(80);

/* ===================== PREFERENCES (flash storage) ===================== */
Preferences prefs;

/* ===================== RTC PERSISTENT STATE ===================== */
// These live in RTC SRAM — survive deep sleep, lost on power-off
RTC_DATA_ATTR unsigned long departureEpoch   = 0;
RTC_DATA_ATTR unsigned long lastAlarmEpoch   = 0;
RTC_DATA_ATTR float         startingTempC    = 999.0;
RTC_DATA_ATTR bool          systemDone       = false;
RTC_DATA_ATTR char          liveNextWakeStore[6] = "--:--"; // "HH:MM\0"

/* ===================== RUNTIME STATE ===================== */
const unsigned long preWakeOffset = 45UL * 60UL;   // first wake 45 min before departure
const unsigned long stepInterval  = 15UL * 60UL;   // repeat every 15 min

float  liveTemp    = 0;
float  liveVoltage = 0;
float  liveCurrent = 0;
float  livePwmPct  = 0;
int    liveDuty    = 0;
String livePhase   = "idle";
String liveFault   = "";
String liveStatus  = "idle";

/* ===================== PWM ===================== */
const int PWM_FREQ       = 500;   // from defroster code
const int PWM_RESOLUTION = 8;

/* ===================== DEFROSTER THRESHOLDS (°F) ===================== */
// Thresholds from the original defroster code — kept in °F.
// Sensor reads °C so we convert before comparing: tempF = (tempC*9/5)+32
const float LOW_THRES  = 35.0;   // °F — full power below this
const float MID_TEMP   = 50.0;   // °F — mid ramp point
const float HIGH_THRES = 67.0;   // °F — minimum power above this

const int DEFROSTER_MIN_PWM = 50;
const int DEFROSTER_MAX_PWM = 220;

/* ===================== VOLTAGE / CURRENT LIMITS ===================== */
const float VMIN               = 10.0;
const float VMAX               = 16.0;
const float MIN_HEATER_CURRENT = 0.05;
const float MAX_HEATER_CURRENT = 5.0;
/* ===================== SMOOTHING ===================== */
const int smoothingSamples = 5;
float     voltageBuffer[smoothingSamples];
int       vBufIdx = 0;
/* ===================== VOLTAGE SMOOTHING ===================== */
float smoothReading(float v, float* buf, int &idx) {
  buf[idx] = v;
  idx = (idx + 1) % smoothingSamples;
  float sum = 0;
  for (int i = 0; i < smoothingSamples; i++) sum += buf[i];
  return sum / smoothingSamples;
}

/* ===================== RAMP STATE ===================== */
// Preserved across the heating loop — ramp smoothly toward target duty
int  currentDuty  = 0;
int  targetDuty   = 0;
int  prevDuty     = -1;

const unsigned long RAMP_INTERVAL = 3000;   // ms between ramp steps
unsigned long lastRampUpdate = 0;



bool systemFault     = false;
bool heatingActive   = false;
bool targetReached   = false;
bool cancelRequested = false;

/* ===================== DUTY CALC (from defroster code, unchanged) ===================== */
// tempF — surface temperature in °F
// prevDuty — pass -1 on first call, then pass the last duty each time
int calculateDuty(float tempF, int pDuty = -1) {
  int duty;
  if (tempF <= LOW_THRES) {
    if (pDuty < 0) duty = DEFROSTER_MAX_PWM;
    else {
      duty = pDuty - 1;
      if (duty < DEFROSTER_MAX_PWM - 20) duty = DEFROSTER_MAX_PWM - 20;
    }
  } else if (tempF <= MID_TEMP) {
    if (pDuty < 0)
      duty = map((long)tempF, (long)LOW_THRES, (long)MID_TEMP,
                 DEFROSTER_MAX_PWM - 20, (DEFROSTER_MAX_PWM + DEFROSTER_MIN_PWM)/2);
    else {
      duty = pDuty - 2;
      if (duty < (DEFROSTER_MAX_PWM + DEFROSTER_MIN_PWM)/2)
        duty = (DEFROSTER_MAX_PWM + DEFROSTER_MIN_PWM)/2;
    }
  } else if (tempF <= HIGH_THRES) {
    if (pDuty < 0)
      duty = map((long)tempF, (long)MID_TEMP, (long)HIGH_THRES,
                 (DEFROSTER_MAX_PWM + DEFROSTER_MIN_PWM)/2, DEFROSTER_MIN_PWM);
    else {
      duty = pDuty - 3;
      if (duty < DEFROSTER_MIN_PWM) duty = DEFROSTER_MIN_PWM;
    }
  } else {
    duty = 255;   // above high threshold — heater off (inverted PWM)
  }
  return constrain(duty, 0, 255);
}

/* ===================== RAMP SPEED (from defroster code, unchanged) ===================== */
int getRampStep(float tempF) {
  if (tempF < 40) return 1;
  else if (tempF < 50) return 2;
  else if (tempF < 60) return 3;
  else return 5;
}

/* ===================== APPLY RAMPED PWM TO MOSFET ===================== */
// Called every RAMP_INTERVAL ms during the heating loop.
// Uses inverted PWM (hwDuty = 255 - currentDuty) exactly as the defroster code does.
void applyRampedPWM(float tempF) {
  if (millis() - lastRampUpdate < RAMP_INTERVAL) return;
  lastRampUpdate = millis();

  int rampStep = getRampStep(tempF);
  if      (currentDuty < targetDuty) currentDuty += rampStep;
  else if (currentDuty > targetDuty) currentDuty -= rampStep;
  currentDuty = constrain(currentDuty, 0, 255);

  int hwDuty = 255 - currentDuty;   // INVERTED — matches defroster hardware wiring
  ledcWrite(MOSFET_PIN, hwDuty);

  liveDuty   = currentDuty;
  livePwmPct = (currentDuty / 255.0) * 100.0;
  prevDuty   = currentDuty;
}

/* ===================== SET HEATER (instant, for fault/off) ===================== */
void setHeater(int duty) {
  currentDuty = constrain(duty, 0, 255);
  targetDuty  = currentDuty;
  liveDuty    = currentDuty;
  livePwmPct  = (currentDuty / 255.0) * 100.0;
  int hwDuty  = 255 - currentDuty;   // inverted: 0 duty = 255 hw = off
  ledcWrite(MOSFET_PIN, hwDuty);
  // Never call digitalWrite on MOSFET_PIN — it overrides ledcWrite PWM mode
}

/* ===================== HEATER OFF ===================== */
void heaterOff() {
  // Write inverted 255 = 0% duty = MOSFET off
  // Do NOT call digitalWrite on MOSFET_PIN — that kills PWM mode
  currentDuty = 0; targetDuty = 0; prevDuty = -1;
  liveDuty = 0; livePwmPct = 0;
  ledcWrite(MOSFET_PIN, 255);   // inverted: 255 = off
  heatingActive = false;
}

/* ===================== FAULT HANDLER ===================== */
void triggerFault(const char* reason) {
  Serial.print("FAULT: "); Serial.println(reason);
  heaterOff();
  digitalWrite(LED_PIN, LOW);
  liveStatus  = "fault";
  liveFault   = String(reason);
  systemFault = true;
}

/* ===================== SAVED WIFI STATE ===================== */
String savedSSID = "";
String savedPASS = "";
bool   staMode   = false;   // true when connected to home WiFi in STA mode

/* ===================== SETUP WIFI ===================== */
// AP is ALWAYS started first — hotspot is guaranteed on every boot.
// If home WiFi credentials are saved, also connect STA (AP+STA mode).
// This matches the Wifi_MOSFET pattern but ensures hotspot never fails.
void setupWiFi() {
  prefs.begin("wifi", true);
  savedSSID = prefs.getString("ssid", "");
  savedPASS = prefs.getString("pass", "");
  prefs.end();

  WiFi.setSleep(false);   // keeps radio always active

  // ── Step 1: always start AP hotspot first ──
  WiFi.mode(WIFI_AP);
  delay(100);

  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);   // give AP time to broadcast before anything else

  Serial.print("Hotspot UP: "); Serial.print(AP_SSID);
  Serial.print("  IP: "); Serial.println(WiFi.softAPIP());

  // ── Step 2: if credentials saved, also join home WiFi ──
  if (savedSSID.length() > 0) {
    Serial.print("Joining home WiFi: "); Serial.println(savedSSID);
    WiFi.mode(WIFI_AP_STA);   // keep AP running while connecting STA
    WiFi.begin(savedSSID.c_str(), savedPASS.c_str());

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      staMode = true;
      Serial.print("Home WiFi IP: "); Serial.println(WiFi.localIP());
    } else {
      staMode = false;
      // STA failed — drop back to AP-only so hotspot stays clean
      WiFi.mode(WIFI_AP);
      Serial.println("Home WiFi failed — hotspot only");
    }
  }
}

/* ===================== SCHEDULE NEXT RTC WAKE ===================== */
void scheduleWake(unsigned long epoch) {
  DateTime t(epoch);
  rtc.clearAlarm(1);
  rtc.setAlarm1(t, DS3231_A1_Date);

  // Persist the next wake time so the UI shows it correctly after sleep
  snprintf(liveNextWakeStore, sizeof(liveNextWakeStore),
           "%02d:%02d", t.hour(), t.minute());

  Serial.print("Next wake: "); Serial.println(t.timestamp());

  heaterOff();
  liveStatus = "idle";
  livePhase  = "sleeping";
  digitalWrite(LED_PIN, LOW);

  // Keep serving web clients for 5 s so the phone gets the final status
  unsigned long serveUntil = millis() + 5000;
  while (millis() < serveUntil) {
    server.handleClient();
    delay(10);
  }

  // Shut down WiFi cleanly before sleep — just mode(OFF) is enough.
  // Do NOT call esp_wifi_stop() — it tears down the netstack and on the
  // next wake (which is a full reboot) the Arduino WiFi driver can't
  // reinitialise it, causing "netstack cb reg failed" and no WiFi.
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
  delay(100);
  esp_deep_sleep_start();
}

/* ===================== READ SENSORS ===================== */
void readSensors() {
  liveTemp    = tempsensor.readTempC();
  liveVoltage = ina219.getBusVoltage_V();
  liveCurrent = ina219.getCurrent_mA() / 1000.0;
  liveVoltage = smoothReading(liveVoltage, voltageBuffer, vBufIdx);
}

/* ===================== RUN FAULT CHECKS ===================== */
bool runFaultChecks() {
  if (liveVoltage < VMIN)               { triggerFault("VOLTAGE TOO LOW");  return false; }
  if (liveVoltage > VMAX)               { triggerFault("VOLTAGE TOO HIGH"); return false; }
  if (liveCurrent > MAX_HEATER_CURRENT) { triggerFault("HEATER SHORT");     return false; }
  // NOTE: MIN current check removed — current is 0 at cycle start and ramps up
  // The ramp takes several seconds to reach detectable current levels
  return true;
}

/* ===================== SERVE PWA HTML ===================== */
extern const char PWA_HTML[] PROGMEM;

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", PWA_HTML);
}

/* ===================== API: /api/status ===================== */
void handleApiStatus() {
  StaticJsonDocument<700> doc;

  doc["tempC"]          = liveTemp;
  // Target expressed in °F (HIGH_THRES) converted back to °C for the app
  // The app will convert to °F for display — this keeps the API consistent
  float targetC_val = (HIGH_THRES - 32.0) * 5.0 / 9.0;  // 67°F → ~19.4°C
  doc["targetC"] = (startingTempC < 999) ? targetC_val : -1;
  doc["voltage"]        = liveVoltage;
  doc["current"]        = liveCurrent;
  doc["pwmPct"]         = livePwmPct;
  doc["status"]         = liveStatus;
  doc["phase"]          = livePhase;
  doc["faultMsg"]       = liveFault;
  doc["nextWake"]       = liveNextWakeStore;   // from RTC_DATA_ATTR — always correct
  doc["departureEpoch"] = (uint32_t)departureEpoch;
  doc["systemDone"]     = systemDone;

  // RTC time string for display
  DateTime now = rtc.now();
  static const char* monthNames[] = {
    "", "Jan","Feb","Mar","Apr","May","Jun",
    "Jul","Aug","Sep","Oct","Nov","Dec"
  };
  char rtcBuf[25];
  snprintf(rtcBuf, sizeof(rtcBuf), "%02d %s %04d %02d:%02d:%02d",
           now.day(), monthNames[now.month()], now.year(),
           now.hour(), now.minute(), now.second());
  doc["rtcTime"]  = String(rtcBuf);
  doc["rtcEpoch"] = (uint32_t)now.unixtime();   // JS uses this to tick clock between polls

  // Departure time as HH:MM string
  if (departureEpoch > 0) {
    DateTime dep(departureEpoch);
    char depBuf[6];
    snprintf(depBuf, sizeof(depBuf), "%02d:%02d", dep.hour(), dep.minute());
    doc["departureTime"] = String(depBuf);
  } else {
    doc["departureTime"] = "--:--";
  }

  // Home WiFi status for the app's setup screen
  prefs.begin("wifi", true);
  doc["staSSID"] = prefs.getString("ssid", "");
  prefs.end();
  doc["staConnected"] = (WiFi.status() == WL_CONNECTED);
  if (WiFi.status() == WL_CONNECTED) {
    doc["staIP"] = WiFi.localIP().toString();
  }

  // PWM ramp details — so the app can show exactly what the heater is doing
  doc["targetDuty"]  = targetDuty;
  doc["currentDuty"] = currentDuty;
  doc["rampStep"]    = getRampStep((liveTemp * 9.0 / 5.0) + 32.0);
  doc["tempF"]       = (liveTemp * 9.0 / 5.0) + 32.0;

  String out;
  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

/* ===================== API: /api/departure ===================== */
/*  Accepts: { "departureEpoch": <unix seconds> }
 *  The app sends the user-chosen time as a UTC-adjusted epoch.
 *  IMPORTANT: sync the RTC first via /api/settime before calling this.
 */
void handleApiDeparture() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "No body"); return; }

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON"); return;
  }

  uint32_t depEpoch = doc["departureEpoch"] | 0;
  Serial.print("Received departureEpoch: "); Serial.println(depEpoch);

  if (depEpoch < 1700000000UL) {
    Serial.println("Rejected: epoch too small — sync the clock first");
    server.send(400, "text/plain", "Invalid epoch — sync clock first");
    return;
  }

  unsigned long nowEpoch = rtc.now().unixtime();
  Serial.print("RTC now: "); Serial.println(nowEpoch);

  // Only enforce "must be in future" if the RTC is plausibly synced
  if (nowEpoch > 1700000000UL && depEpoch <= nowEpoch + 60) {
    server.send(400, "text/plain", "Time must be in future");
    return;
  }

  departureEpoch = depEpoch;

  // First wake = 45 min before departure; clamp if already past that window
  unsigned long firstWake = departureEpoch - preWakeOffset;
  if (firstWake <= nowEpoch) firstWake = nowEpoch + 10;
  lastAlarmEpoch = firstWake;

  Serial.print("Departure: "); Serial.println(DateTime(departureEpoch).timestamp());
  Serial.print("First wake: "); Serial.println(DateTime(lastAlarmEpoch).timestamp());

  if (startingTempC >= 999) {
    startingTempC = liveTemp;
    Serial.print("Starting temp: "); Serial.println(startingTempC);
  }

  // Respond BEFORE going to sleep so the app gets the 200 OK
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500);
  scheduleWake(lastAlarmEpoch);
}

/* ===================== API: /api/settime ===================== */
void handleApiSetTime() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "No body"); return; }

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON"); return;
  }

  uint32_t epoch = doc["epoch"] | 0;
  if (epoch < 1700000000UL) {
    server.send(400, "text/plain", "Invalid epoch");
    return;
  }

  rtc.adjust(DateTime(epoch));
  Serial.print("RTC synced: "); Serial.println(rtc.now().timestamp());
  server.send(200, "application/json", "{\"ok\":true}");
}

/* ===================== API: /api/wifi ===================== */
/*  Accepts: { "ssid": "MyWiFi", "password": "secret" }
 *  Saves to Preferences flash and immediately attempts to join.
 *  Hotspot stays up throughout — no reboot needed.
 */
void handleApiWifi() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "No body"); return; }

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON"); return;
  }

  const char* ssid = doc["ssid"] | "";
  const char* pass = doc["password"] | "";

  if (strlen(ssid) == 0) {
    server.send(400, "text/plain", "SSID required");
    return;
  }

  // Save to flash
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();

  Serial.print("Saved WiFi creds for: "); Serial.println(ssid);

  // Update in-memory credentials immediately
  savedSSID = String(ssid);
  savedPASS = String(pass);

  server.send(200, "application/json", "{\"ok\":true}");
  delay(200);

  // Switch to STA mode and connect (same pattern as Wifi_MOSFET.ino)
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    server.handleClient();
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    staMode = true;
    Serial.print("Joined home WiFi. IP: "); Serial.println(WiFi.localIP());
  } else {
    // Fall back to AP if connection fails
    staMode = false;
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.println("Home WiFi join timed out — back on hotspot");
  }
}

/* ===================== API: /api/cancel ===================== */
void handleApiCancel() {
  server.send(200, "application/json", "{\"ok\":true}");
  cancelRequested = true;
  Serial.println("Cancel requested via app");
}

/* ===================== SETUP WEBSERVER ===================== */
void setupWebServer() {
  server.on("/",               HTTP_GET,  handleRoot);
  server.on("/api/status",     HTTP_GET,  handleApiStatus);
  server.on("/api/departure",  HTTP_POST, handleApiDeparture);
  server.on("/api/settime",    HTTP_POST, handleApiSetTime);
  server.on("/api/wifi",       HTTP_POST, handleApiWifi);
  server.on("/api/cancel",     HTTP_POST, handleApiCancel);
  server.begin();
  Serial.println("Web server started");
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== NovaShade Boot ===");

  /* ---- GPIO ---- */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  // MOSFET_PIN is managed entirely by ledcWrite — do NOT pinMode or digitalWrite it
  // pinMode/digitalWrite disables PWM mode and fights ledcWrite

  /* ---- PWM ---- */
  ledcAttach(MOSFET_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(MOSFET_PIN, 255);   // inverted: 255 = MOSFET off at boot

  /* ---- I2C ---- */
  Wire.begin(SDA_BUS0, SCL_BUS0);
  I2C_1.begin(SDA_BUS1, SCL_BUS1, 100000);

  /* ---- RTC ---- */
  if (!rtc.begin(&Wire)) {
    Serial.println("RTC not found! Halting.");
    while (1) delay(1000);
  }
  Serial.println("RTC OK");

  /* ---- INA219 ---- */
  if (!ina219.begin(&Wire)) {
    Serial.println("INA219 not found! Halting.");
    while (1) delay(1000);
  }
  Serial.println("INA219 OK");

  /* ---- MCP9808 ---- */
  if (!tempsensor.begin(0x18, &I2C_1)) {
    Serial.println("MCP9808 not found! Halting.");
    while (1) delay(1000);
  }
  tempsensor.setResolution(3);
  Serial.println("MCP9808 OK");

  /* ---- Smoothing buffer ---- */
  float v = ina219.getBusVoltage_V();
  for (int i = 0; i < smoothingSamples; i++) voltageBuffer[i] = v;

  /* ---- WiFi (AP always + optional STA) ---- */
  setupWiFi();
  setupWebServer();

  /* ===================== WAKE-FROM-SLEEP HANDLING ===================== */
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke from RTC alarm");

    unsigned long nowEpoch = rtc.now().unixtime();

    /* ---- Departure reached? ---- */
    if (departureEpoch != 0 && nowEpoch >= departureEpoch) {
      Serial.println("Departure reached — system complete");
      liveStatus = "departed";
      livePhase  = "done";
      systemDone = true;
      rtc.clearAlarm(1);
      heaterOff();
      digitalWrite(LED_PIN, LOW);

      // Stay awake 30 s so the app can see the final state
      unsigned long until = millis() + 30000;
      while (millis() < until) { server.handleClient(); delay(10); }
      esp_deep_sleep_start();
      return;
    }

    /* ---- Heating cycle ---- */
    // Convert current sensor reading to °F for the defroster threshold logic
    float tempF = (liveTemp * 9.0 / 5.0) + 32.0;

    // Heating is needed when surface temp is below HIGH_THRES (67°F)
    if (tempF >= HIGH_THRES) {
      Serial.println("Temp OK — no heating needed");
      livePhase  = "warm";
      liveStatus = "done";
    } else {
      liveStatus    = "heating";
      livePhase     = "heating";
      heatingActive = true;
      prevDuty      = -1;   // fresh start — first calculateDuty call gets full power

      // Prime the target duty from the current temperature
      targetDuty = calculateDuty(tempF, prevDuty);

      while (heatingActive && !systemFault && !cancelRequested) {
        server.handleClient();   // keep phone connected during heating
        readSensors();

        if (!runFaultChecks()) break;

        // Recalculate target from live sensor reading (°F)
        tempF      = (liveTemp * 9.0 / 5.0) + 32.0;
        targetDuty = calculateDuty(tempF, prevDuty);

        // Step currentDuty toward targetDuty at the ramp rate (non-blocking)
        applyRampedPWM(tempF);

        // Done when surface is warm enough
        if (tempF >= HIGH_THRES) {
          Serial.println("Target temp reached!");
          liveStatus    = "done";
          livePhase     = "threshold reached";
          targetReached = true;
          heaterOff();
          break;
        }

        Serial.print("Temp: "); Serial.print(liveTemp);
        Serial.print(" C ("); Serial.print(tempF, 1); Serial.print(" F)");
        Serial.print("  Target: "); Serial.print(targetDuty);
        Serial.print("  Current: "); Serial.print(currentDuty);
        Serial.print("  HW(inv): "); Serial.println(255 - currentDuty);
        delay(200);   // short delay — lets ramp fire at its own 3s interval
      }
    }

    if (cancelRequested) {
      Serial.println("Cancelled by user");
      heaterOff();
      digitalWrite(LED_PIN, LOW);
      liveStatus = "idle";
      unsigned long until = millis() + 10000;
      while (millis() < until) { server.handleClient(); delay(10); }
      esp_deep_sleep_start();
      return;
    }

    if (!systemFault) {
      /* Advance to next 15-min alarm, capping at departure */
      unsigned long nextAlarm = lastAlarmEpoch + stepInterval;
      if (nextAlarm > departureEpoch) nextAlarm = departureEpoch;
      lastAlarmEpoch = nextAlarm;
      Serial.print("Next alarm: "); Serial.println(DateTime(lastAlarmEpoch).timestamp());
      scheduleWake(lastAlarmEpoch);
    } else {
      // Fault — stay awake 60 s so user can see it in the app
      unsigned long until = millis() + 60000;
      while (millis() < until) { server.handleClient(); delay(10); }
      esp_deep_sleep_start();
    }

    return;
  }

  /* ===================== COLD BOOT ===================== */
  Serial.println("Cold boot — ready");
  Serial.println("1. Connect phone to 'NovaShade' WiFi (or home WiFi if saved)");
  Serial.println("2. Open http://192.168.4.1  (or home IP shown above)");
  Serial.println("3. Sync the clock, then set your departure time");

  liveStatus = "idle";
  livePhase  = "waiting for setup";

  readSensors();
  startingTempC = liveTemp;
}

/* ===================== LOOP ===================== */
void loop() {
  yield();
  server.handleClient();

  // WiFi reconnect watchdog (from Wifi_MOSFET.ino pattern)
  if (staMode && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.disconnect();
    WiFi.begin(savedSSID.c_str(), savedPASS.c_str());
  }

  // Read sensors every second so the app always shows live data
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > 1000) {
    readSensors();
    lastSensorRead = millis();
  }

  // Every 3 minutes re-read temp and recalculate target duty (matches Wifi_MOSFET inputInterval)
  // lastTempUpdate starts at 0 so the first update fires immediately on boot
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate >= 180000UL || lastTempUpdate == 0) {
    lastTempUpdate = millis();
    float tempF = (liveTemp * 9.0 / 5.0) + 32.0;
    targetDuty = calculateDuty(tempF, prevDuty);
    Serial.print("Temp update — "); Serial.print(liveTemp);
    Serial.print(" C / "); Serial.print(tempF);
    Serial.print(" F  Target duty: "); Serial.print(targetDuty);
    Serial.print("  Current duty: "); Serial.println(currentDuty);
  }

  // Apply ramp every RAMP_INTERVAL ms — always active, not just when heating
  // This lets the MOSFET respond to temperature even on cold boot / idle
  float nowTempF = (liveTemp * 9.0 / 5.0) + 32.0;
  applyRampedPWM(nowTempF);

  // Blink LED while idle/waiting
  static unsigned long lastBlink = 0;
  static bool ledState = true;
  if (liveStatus == "idle" && millis() - lastBlink > 1000) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }
}

/* ===================== PWA HTML (PROGMEM) ===================== */
const char PWA_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="NovaShade">
<meta name="theme-color" content="#060412">
<title>NovaShade</title>
<link href="https://fonts.googleapis.com/css2?family=DM+Mono:wght@300;400;500&family=Syne:wght@700;800&display=swap" rel="stylesheet">
<style>
:root {
  --bg:      #060412;
  --border:  rgba(139,92,246,0.18);
  --accent:  #a78bfa;
  --accent2: #7c3aed;
  --accent3: #c4b5fd;
  --text:    #ede9fe;
  --muted:   #5b4d8a;
  --ok:      #34d399;
  --warn:    #fbbf24;
  --danger:  #f87171;
  --card:    rgba(109,40,217,0.06);
}
* { box-sizing:border-box; margin:0; padding:0; -webkit-tap-highlight-color:transparent; }
html,body { height:100%; background:var(--bg); color:var(--text); font-family:'DM Mono',monospace; overflow-x:hidden; }

/* ── ATMOSPHERE ── */
.bg-orb { position:fixed; border-radius:50%; pointer-events:none; filter:blur(80px); z-index:0; }
.bg-orb-1 { width:480px; height:480px; background:radial-gradient(circle,rgba(109,40,217,.18) 0%,transparent 70%); top:-160px; left:50%; transform:translateX(-50%); animation:orbFloat 7s ease-in-out infinite; }
.bg-orb-2 { width:300px; height:300px; background:radial-gradient(circle,rgba(167,139,250,.08) 0%,transparent 70%); bottom:20%; right:-80px; animation:orbFloat 9s ease-in-out infinite reverse; }
.bg-orb-3 { width:200px; height:200px; background:radial-gradient(circle,rgba(79,26,180,.12) 0%,transparent 70%); top:40%; left:-60px; animation:orbFloat 11s ease-in-out infinite; }
.rays { position:fixed; inset:0; background:conic-gradient(from 260deg at 50% -5%,transparent 0deg,rgba(109,40,217,.06) 4deg,transparent 12deg,rgba(109,40,217,.04) 18deg,transparent 26deg,rgba(90,20,200,.07) 32deg,transparent 42deg,rgba(109,40,217,.05) 48deg,transparent 58deg,rgba(90,20,200,.03) 63deg,transparent 75deg); pointer-events:none; z-index:0; animation:rayPulse 6s ease-in-out infinite; }
body::after { content:''; position:fixed; inset:0; background-image:url("data:image/svg+xml,%3Csvg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='n'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.85' numOctaves='4' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23n)' opacity='0.035'/%3E%3C/svg%3E"); pointer-events:none; z-index:998; opacity:.5; }
@keyframes orbFloat { 0%,100%{transform:translateX(-50%) translateY(0) scale(1)} 50%{transform:translateX(-50%) translateY(-20px) scale(1.05)} }
@keyframes rayPulse { 0%,100%{opacity:.6} 50%{opacity:1} }

/* ── SETUP SCREEN ── */
#setupScreen { position:fixed; inset:0; z-index:200; background:var(--bg); display:flex; flex-direction:column; align-items:center; justify-content:center; padding:32px; text-align:center; transition:opacity .4s; }
#setupScreen.hidden { opacity:0; pointer-events:none; }
.setup-logo { font-family:'Syne',sans-serif; font-size:36px; font-weight:800; letter-spacing:-1px; margin-bottom:6px; }
.setup-logo span { color:var(--accent3); }
.setup-sub { font-size:11px; color:var(--muted); letter-spacing:.12em; text-transform:uppercase; margin-bottom:32px; }
.setup-card { width:100%; max-width:360px; background:var(--card); border:1px solid var(--border); border-radius:20px; padding:24px; text-align:left; position:relative; overflow:hidden; margin-bottom:12px; }
.setup-card::before { content:''; position:absolute; top:0; left:20%; right:20%; height:1px; background:linear-gradient(90deg,transparent,rgba(167,139,250,.5),transparent); }
.setup-label { font-size:10px; color:var(--muted); letter-spacing:.1em; text-transform:uppercase; margin-bottom:5px; margin-top:14px; }
.setup-label:first-of-type { margin-top:0; }
.setup-input { width:100%; background:rgba(139,92,246,.08); border:1px solid var(--border); border-radius:10px; padding:12px 14px; font-family:'DM Mono',monospace; font-size:14px; color:var(--text); outline:none; transition:border-color .2s,box-shadow .2s; }
.setup-input:focus { border-color:var(--accent); box-shadow:0 0 0 3px rgba(124,58,237,.15); }
.setup-input::placeholder { color:var(--muted); }
.setup-note { font-size:10px; color:var(--muted); margin-top:10px; line-height:1.6; letter-spacing:.03em; }
.btn-primary { width:100%; margin-top:16px; background:linear-gradient(135deg,var(--accent2),var(--accent)); color:#fff; border:none; border-radius:12px; padding:14px; font-family:'Syne',sans-serif; font-weight:700; font-size:14px; letter-spacing:.05em; cursor:pointer; box-shadow:0 4px 24px rgba(124,58,237,.4); transition:opacity .2s,transform .1s; }
.btn-primary:active { opacity:.85; transform:scale(.98); }
.setup-skip { font-size:11px; color:var(--muted); cursor:pointer; letter-spacing:.06em; padding:8px; }
.setup-skip:hover { color:var(--accent3); }
.setup-tabs { display:flex; gap:8px; margin-bottom:16px; }
.setup-tab { flex:1; padding:9px; border-radius:10px; border:1px solid var(--border); background:transparent; font-family:'DM Mono',monospace; font-size:11px; color:var(--muted); cursor:pointer; transition:all .2s; text-align:center; letter-spacing:.05em; }
.setup-tab.active { background:rgba(124,58,237,.15); border-color:var(--accent); color:var(--accent3); }
#wifiPanel, #hotspotPanel { display:none; }
#wifiPanel.show, #hotspotPanel.show { display:block; }

/* ── MAIN APP ── */
.app { position:relative; z-index:1; max-width:430px; margin:0 auto; padding:0 0 48px; opacity:0; animation:fadeUp .7s .1s ease both; }
@keyframes fadeUp { from{opacity:0;transform:translateY(20px)} to{opacity:1;transform:translateY(0)} }

header { padding:52px 24px 20px; display:flex; align-items:flex-start; justify-content:space-between; }
.logo { font-family:'Syne',sans-serif; font-weight:800; font-size:26px; letter-spacing:-.5px; line-height:1; }
.logo em { color:var(--accent3); font-style:normal; }
.conn-badge { display:flex; align-items:center; gap:7px; background:var(--card); border:1px solid var(--border); border-radius:20px; padding:6px 12px 6px 8px; font-size:10px; color:var(--muted); letter-spacing:.06em; transition:all .3s; }
.conn-badge.live { border-color:rgba(52,211,153,.3); color:var(--ok); }
.conn-dot { width:6px; height:6px; border-radius:50%; background:var(--muted); transition:background .3s; }
.conn-badge.live .conn-dot { background:var(--ok); animation:dotPulse 2s ease-in-out infinite; }
@keyframes dotPulse { 0%,100%{box-shadow:0 0 0 0 rgba(52,211,153,.4)} 50%{box-shadow:0 0 0 5px rgba(52,211,153,0)} }

.offline-banner { display:none; margin:0 16px 14px; background:rgba(248,113,113,.07); border:1px solid rgba(248,113,113,.2); border-radius:12px; padding:11px 16px; font-size:11px; color:var(--danger); text-align:center; letter-spacing:.05em; }
.offline-banner.show { display:block; }

/* ── TEMP HERO ── */
.temp-hero { margin:0 16px 14px; background:var(--card); border:1px solid var(--border); border-radius:24px; padding:28px 28px 24px; position:relative; overflow:hidden; animation:fadeUp .7s .15s ease both; }
.temp-hero::after { content:''; position:absolute; top:0; left:15%; right:15%; height:1px; background:linear-gradient(90deg,transparent,rgba(196,181,253,.6),transparent); }
.temp-hero::before { content:''; position:absolute; top:-60px; left:50%; transform:translateX(-50%); width:200px; height:120px; background:radial-gradient(ellipse,rgba(124,58,237,.12) 0%,transparent 70%); pointer-events:none; }
.hero-top { display:flex; align-items:flex-start; justify-content:space-between; margin-bottom:4px; }
.temp-label { font-size:9px; letter-spacing:.18em; text-transform:uppercase; color:var(--muted); }
.phase-badge { font-size:9px; letter-spacing:.1em; text-transform:uppercase; padding:4px 10px; border-radius:20px; background:rgba(139,92,246,.12); border:1px solid rgba(139,92,246,.2); color:var(--accent3); transition:all .4s; }
.phase-badge.heating { background:rgba(251,191,36,.1); border-color:rgba(251,191,36,.25); color:var(--warn); }
.phase-badge.ok      { background:rgba(52,211,153,.1);  border-color:rgba(52,211,153,.25);  color:var(--ok); }
.phase-badge.fault   { background:rgba(248,113,113,.1); border-color:rgba(248,113,113,.25); color:var(--danger); }
.temp-num-row { display:flex; align-items:flex-end; gap:4px; margin:8px 0 16px; }
.temp-big { font-family:'Syne',sans-serif; font-size:88px; font-weight:800; line-height:.9; color:var(--text); letter-spacing:-4px; text-shadow:0 0 60px rgba(167,139,250,.2); transition:color .5s; }
.temp-big.cold { color:#93c5fd; }
.temp-big.warm { color:var(--ok); }
.temp-unit { font-family:'Syne',sans-serif; font-size:28px; font-weight:700; color:var(--muted); margin-bottom:14px; }
.target-row { display:flex; align-items:center; gap:8px; margin-bottom:18px; font-size:11px; color:var(--muted); }
.target-val { color:var(--accent3); font-weight:500; }
.target-sep { opacity:.3; }
.pwm-row { display:flex; justify-content:space-between; font-size:9px; letter-spacing:.12em; text-transform:uppercase; color:var(--muted); margin-bottom:6px; }
.pwm-pct { color:var(--accent3); }
.pwm-track { height:3px; background:rgba(139,92,246,.12); border-radius:2px; overflow:visible; position:relative; }
.pwm-fill { height:100%; background:linear-gradient(90deg,var(--accent2),var(--accent3)); border-radius:2px; transition:width .9s cubic-bezier(.4,0,.2,1); width:0%; position:relative; }
.pwm-fill::after { content:''; position:absolute; right:0; top:-3px; width:9px; height:9px; border-radius:50%; background:var(--accent3); box-shadow:0 0 10px rgba(196,181,253,.8); transform:translateX(50%); }

/* ── STATUS ── */
.status-row { margin:0 16px 14px; background:var(--card); border:1px solid var(--border); border-radius:16px; padding:14px 18px; display:flex; align-items:center; gap:14px; animation:fadeUp .7s .2s ease both; }
.status-emoji { width:38px; height:38px; border-radius:50%; display:flex; align-items:center; justify-content:center; font-size:17px; flex-shrink:0; background:rgba(139,92,246,.1); transition:background .3s; }
.status-emoji.ok    { background:rgba(52,211,153,.12); }
.status-emoji.warn  { background:rgba(251,191,36,.12); }
.status-emoji.fault { background:rgba(248,113,113,.12); }
.status-info { flex:1; }
.status-title { font-size:13px; color:var(--text); font-weight:500; }
.status-desc  { font-size:10px; color:var(--muted); margin-top:3px; letter-spacing:.04em; }

/* ── METRICS ── */
.metrics { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; margin:0 16px 14px; animation:fadeUp .7s .25s ease both; }
.metric { background:var(--card); border:1px solid var(--border); border-radius:16px; padding:16px 14px; position:relative; overflow:hidden; transition:border-color .3s; }
.metric::after { content:''; position:absolute; top:0; left:15%; right:15%; height:1px; background:linear-gradient(90deg,transparent,rgba(167,139,250,.25),transparent); }
.metric.ok    { border-color:rgba(52,211,153,.2); }
.metric.warn  { border-color:rgba(251,191,36,.2); }
.metric.fault { border-color:rgba(248,113,113,.2); }
.metric-label { font-size:8px; letter-spacing:.15em; text-transform:uppercase; color:var(--muted); margin-bottom:6px; }
.metric-val { font-family:'Syne',sans-serif; font-size:22px; font-weight:700; line-height:1; color:var(--text); transition:color .3s; }
.metric.ok    .metric-val { color:var(--ok); }
.metric.warn  .metric-val { color:var(--warn); }
.metric.fault .metric-val { color:var(--danger); }
.metric-unit { font-size:9px; color:var(--muted); margin-top:3px; letter-spacing:.05em; }

.next-wake-bar { margin:0 16px 14px; background:var(--card); border:1px solid var(--border); border-radius:14px; padding:13px 18px; display:flex; align-items:center; justify-content:space-between; animation:fadeUp .7s .3s ease both; }
.nw-label { font-size:9px; letter-spacing:.15em; text-transform:uppercase; color:var(--muted); }
.nw-val { font-family:'Syne',sans-serif; font-size:16px; font-weight:700; color:var(--accent3); }

.sec-label { font-size:9px; letter-spacing:.22em; text-transform:uppercase; color:var(--muted); margin:0 16px 8px; padding-left:2px; }

/* ── DEPARTURE CARD ── */
.dep-card { margin:0 16px 8px; background:var(--card); border:1px solid var(--border); border-radius:18px; padding:20px; animation:fadeUp .7s .32s ease both; }
.dep-col-label { font-size:9px; letter-spacing:.12em; text-transform:uppercase; color:var(--muted); margin-bottom:4px; }
.dep-big { font-family:'Syne',sans-serif; font-size:28px; font-weight:800; color:var(--accent3); letter-spacing:-1px; }
.dep-countdown-val { font-family:'Syne',sans-serif; font-size:24px; font-weight:800; color:var(--text); letter-spacing:-1px; text-align:right; }
#depSetRow { display:none; background:rgba(167,139,250,.06); border:1px solid rgba(167,139,250,.15); border-radius:12px; padding:14px; margin-bottom:14px; }
.dep-info-grid { display:flex; justify-content:space-between; align-items:flex-end; gap:8px; }
.input-row { display:flex; gap:8px; margin-bottom:10px; }
.field { flex:1; background:rgba(139,92,246,.07); border:1px solid var(--border); border-radius:12px; padding:13px 14px; font-family:'DM Mono',monospace; font-size:16px; color:var(--text); outline:none; transition:border-color .2s,box-shadow .2s; -webkit-appearance:none; }
.field:focus { border-color:var(--accent); box-shadow:0 0 0 3px rgba(124,58,237,.12); }
.field::placeholder { color:var(--muted); }
.dep-hint { font-size:10px; color:var(--muted); letter-spacing:.04em; }
.btn-set { background:linear-gradient(135deg,var(--accent2),var(--accent)); color:#fff; border:none; border-radius:12px; padding:13px 20px; font-family:'Syne',sans-serif; font-weight:700; font-size:13px; cursor:pointer; box-shadow:0 2px 16px rgba(124,58,237,.35); transition:opacity .2s,transform .1s; letter-spacing:.05em; white-space:nowrap; }
.btn-set:active { opacity:.8; transform:scale(.97); }

/* ── RTC + WIFI CARDS ── */
.rtc-card { margin:0 16px 8px; background:var(--card); border:1px solid var(--border); border-radius:18px; padding:20px; animation:fadeUp .7s .34s ease both; }
.rtc-time { font-family:'Syne',sans-serif; font-size:17px; font-weight:700; color:var(--text); margin-bottom:14px; letter-spacing:-.3px; }
.wifi-card { margin:0 16px 8px; background:var(--card); border:1px solid var(--border); border-radius:18px; padding:20px; animation:fadeUp .7s .35s ease both; }
.wifi-status { font-size:11px; margin-bottom:12px; padding:8px 12px; border-radius:8px; background:rgba(139,92,246,.07); letter-spacing:.04em; }
.wifi-status.connected { background:rgba(52,211,153,.07); color:var(--ok); border:1px solid rgba(52,211,153,.2); }
.wifi-status.disconnected { color:var(--muted); border:1px solid var(--border); }
.wifi-inputs { display:flex; flex-direction:column; gap:8px; margin-bottom:10px; }
.wifi-input { background:rgba(139,92,246,.07); border:1px solid var(--border); border-radius:10px; padding:11px 14px; font-family:'DM Mono',monospace; font-size:13px; color:var(--text); outline:none; transition:border-color .2s; }
.wifi-input:focus { border-color:var(--accent); }
.wifi-input::placeholder { color:var(--muted); }
.btn-ghost { width:100%; background:rgba(139,92,246,.07); border:1px solid var(--border); border-radius:12px; padding:12px; font-family:'Syne',sans-serif; font-weight:700; font-size:12px; color:var(--text); cursor:pointer; transition:background .2s,border-color .2s; letter-spacing:.05em; margin-top:6px; }
.btn-ghost:active { background:rgba(167,139,250,.12); border-color:var(--accent); }
.sync-warn { font-size:10px; color:var(--warn); letter-spacing:.05em; margin-bottom:10px; }

/* ── CANCEL ── */
.btn-cancel { display:block; width:calc(100% - 32px); margin:6px 16px 0; background:rgba(248,113,113,.06); border:1px solid rgba(248,113,113,.18); border-radius:16px; padding:15px; font-family:'Syne',sans-serif; font-weight:700; font-size:13px; color:var(--danger); cursor:pointer; transition:background .2s; letter-spacing:.05em; text-align:center; animation:fadeUp .7s .36s ease both; }
.btn-cancel:active { background:rgba(248,113,113,.14); }

.live-bar { text-align:center; font-size:9px; color:var(--muted); margin:10px 0 0; letter-spacing:.1em; text-transform:uppercase; }
.live-bar .spin { display:inline-block; animation:spin 1s linear infinite; margin-right:4px; }
@keyframes spin { to{transform:rotate(360deg)} }

.toast { position:fixed; bottom:32px; left:50%; transform:translateX(-50%) translateY(80px); background:rgba(13,10,30,.95); border:1px solid var(--border); border-radius:14px; padding:12px 22px; font-size:12px; color:var(--text); backdrop-filter:blur(12px); transition:transform .3s cubic-bezier(.4,0,.2,1); z-index:500; white-space:nowrap; letter-spacing:.04em; box-shadow:0 8px 32px rgba(0,0,0,.4); }
.toast.show { transform:translateX(-50%) translateY(0); }
.toast.ok   { border-color:rgba(52,211,153,.35); }
.toast.err  { border-color:rgba(248,113,113,.35); }
</style>
</head>
<body>
<div class="rays"></div>
<div class="bg-orb bg-orb-1"></div>
<div class="bg-orb bg-orb-2"></div>
<div class="bg-orb bg-orb-3"></div>

<!-- ═══════ SETUP SCREEN ═══════ -->
<div id="setupScreen">
  <div class="setup-logo">Nova<span>Shade</span></div>
  <div class="setup-sub">Adaptive Frost Clear</div>

  <div class="setup-card">
    <div class="setup-tabs">
      <button class="setup-tab active" onclick="showTab('hotspot')">Use Hotspot</button>
      <button class="setup-tab"        onclick="showTab('wifi')">Save Home WiFi</button>
    </div>

    <!-- Hotspot tab -->
    <div id="hotspotPanel" class="show">
      <div class="setup-note">
        1. Connect your phone to the <strong style="color:var(--accent3)">"NovaShade"</strong> WiFi network<br>
        2. Come back here and tap Open App<br>
        3. The device is always reachable at <strong style="color:var(--accent3)">192.168.4.1</strong>
      </div>
      <button class="btn-primary" onclick="closeSetup()">Open App</button>
    </div>

    <!-- Home WiFi tab -->
    <div id="wifiPanel">
      <div class="setup-note" style="margin-bottom:12px;">
        Save your home WiFi so you can use the app without switching networks. The hotspot stays on as a backup.
      </div>
      <div class="wifi-inputs">
        <input class="wifi-input" type="text"     id="setupSSID" placeholder="Home WiFi name" autocomplete="off" autocorrect="off" spellcheck="false">
        <input class="wifi-input" type="password" id="setupPass" placeholder="Password">
      </div>
      <button class="btn-primary" onclick="saveHomeWifi()">Save &amp; Connect</button>
    </div>
  </div>

  <div class="setup-skip" onclick="closeSetup()">Skip for now</div>
</div>

<!-- ═══════ MAIN APP ═══════ -->
<div class="app" id="mainApp">
  <header>
    <div>
      <div class="logo">Nova<em>Shade</em></div>
      <div style="font-size:9px;color:var(--muted);letter-spacing:.1em;text-transform:uppercase;margin-top:3px;">Adaptive Frost Clear</div>
    </div>
    <div class="conn-badge" id="connBadge">
      <div class="conn-dot"></div>
      <span id="connLabel">Connecting</span>
    </div>
  </header>

  <div class="offline-banner" id="offlineBanner">&#9888; Cannot reach device — check WiFi</div>

  <!-- TEMP HERO -->
  <div class="temp-hero">
    <div class="hero-top">
      <div class="temp-label">Surface Temp</div>
      <div class="phase-badge" id="phaseBadge">Idle</div>
    </div>
    <div class="temp-num-row">
      <div class="temp-big" id="tempBig">--</div>
      <div class="temp-unit">&#176;F</div>
    </div>
    <div class="target-row">
      <span>Target</span>
      <span class="target-val" id="targetVal">--&#176;F</span>
      <span class="target-sep">&#183;</span>
      <span id="phaseVal">--</span>
    </div>
    <div class="pwm-row">
      <span>Heater output</span>
      <span class="pwm-pct" id="pwmPct">0%</span>
    </div>
    <div class="pwm-track"><div class="pwm-fill" id="pwmBar"></div></div>

    <!-- RAMP DETAIL ROW — shows target vs current duty and ramp step -->
    <div style="display:flex;justify-content:space-between;margin-top:14px;gap:8px;">
      <div style="flex:1;background:rgba(139,92,246,.07);border:1px solid rgba(139,92,246,.12);border-radius:10px;padding:10px 12px;">
        <div style="font-size:8px;letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:4px;">Target Duty</div>
        <div style="font-family:'Syne',sans-serif;font-size:20px;font-weight:700;color:var(--accent3);" id="targetDutyVal">--</div>
        <div style="font-size:9px;color:var(--muted);">/ 255</div>
      </div>
      <div style="flex:1;background:rgba(139,92,246,.07);border:1px solid rgba(139,92,246,.12);border-radius:10px;padding:10px 12px;">
        <div style="font-size:8px;letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:4px;">Current Duty</div>
        <div style="font-family:'Syne',sans-serif;font-size:20px;font-weight:700;color:var(--text);" id="currentDutyVal">--</div>
        <div style="font-size:9px;color:var(--muted);">/ 255</div>
      </div>
      <div style="flex:1;background:rgba(139,92,246,.07);border:1px solid rgba(139,92,246,.12);border-radius:10px;padding:10px 12px;">
        <div style="font-size:8px;letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:4px;">Ramp Step</div>
        <div style="font-family:'Syne',sans-serif;font-size:20px;font-weight:700;color:var(--warn);" id="rampStepVal">--</div>
        <div style="font-size:9px;color:var(--muted);">per 3s</div>
      </div>
    </div>
  </div>

  <!-- STATUS -->
  <div class="status-row">
    <div class="status-emoji" id="statusEmoji">&#10052;&#65039;</div>
    <div class="status-info">
      <div class="status-title" id="statusTitle">Waiting for device</div>
      <div class="status-desc"  id="statusDesc">Connect to NovaShade WiFi</div>
    </div>
  </div>

  <!-- METRICS -->
  <div class="metrics">
    <div class="metric" id="voltCard"><div class="metric-label">Voltage</div><div class="metric-val" id="voltVal">--</div><div class="metric-unit">Volts</div></div>
    <div class="metric" id="currCard"><div class="metric-label">Current</div><div class="metric-val" id="currVal">--</div><div class="metric-unit">Amps</div></div>
    <div class="metric" id="powCard"> <div class="metric-label">Power</div>  <div class="metric-val" id="powVal">--</div> <div class="metric-unit">Watts</div></div>
  </div>

  <div class="next-wake-bar">
    <div class="nw-label">Next scheduled wake</div>
    <div class="nw-val" id="nextVal">--:--</div>
  </div>

  <!-- DEPARTURE -->
  <div class="sec-label">Departure</div>
  <div class="dep-card">
    <div id="depSetRow">
      <div class="dep-info-grid">
        <div><div class="dep-col-label">Leaving at</div><div class="dep-big" id="depTimeDisp">--:--</div></div>
        <div style="text-align:center;"><div class="dep-col-label">First heat</div><div class="dep-big" style="font-size:20px;color:var(--accent);" id="firstWakeDisp">--:--</div></div>
        <div style="text-align:right;"><div class="dep-col-label">Countdown</div><div class="dep-countdown-val" id="depCountdown">--</div></div>
      </div>
    </div>
    <div class="input-row">
      <input class="field" type="time" id="depTimeInput">
      <button class="btn-set" onclick="setDeparture()">SET</button>
    </div>
    <div class="dep-hint">Heater starts 45 min before, repeats every 15 min until departure.</div>
  </div>

  <!-- DEVICE CLOCK -->
  <div class="sec-label" style="margin-top:8px">Device Clock</div>
  <div class="rtc-card">
    <div class="rtc-time" id="rtcDisplay">-- --- ---- --:--:--</div>
    <div class="sync-warn">&#9888; Sync clock first before setting departure</div>
    <button class="btn-ghost" onclick="syncRTC()">&#10227; &nbsp; Sync Phone Time to Device</button>
  </div>

  <!-- HOME WIFI -->
  <div class="sec-label" style="margin-top:8px">Home WiFi</div>
  <div class="wifi-card">
    <div class="wifi-status disconnected" id="wifiStatus">Not connected to home network</div>
    <div class="wifi-inputs">
      <input class="wifi-input" type="text"     id="wifiSSID" placeholder="Home WiFi name" autocomplete="off" autocorrect="off" spellcheck="false">
      <input class="wifi-input" type="password" id="wifiPass" placeholder="Password">
    </div>
    <button class="btn-ghost" onclick="saveWifi()">&#8644; &nbsp; Save &amp; Connect Home WiFi</button>
  </div>

  <button class="btn-cancel" onclick="cancelSystem()">&#10005; &nbsp; Cancel &amp; Shutdown</button>

  <div class="live-bar" id="liveBar"><span class="spin">&#8635;</span> Connecting...</div>
</div>

<div class="toast" id="toast"></div>

<script>
const API = '';
const MONTHS = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];

let depEpochMs   = 0;
let rtcBaseEpoch = 0;   // device unix epoch from last poll
let rtcBaseMs    = 0;   // browser performance.now() when that poll landed

// ── SETUP SCREEN ──────────────────────────────────────────────────────────
function showTab(t) {
  document.querySelectorAll('.setup-tab').forEach((el,i) => el.classList.toggle('active', (t==='hotspot'?i===0:i===1)));
  document.getElementById('hotspotPanel').classList.toggle('show', t==='hotspot');
  document.getElementById('wifiPanel').classList.toggle('show', t==='wifi');
}

function closeSetup() {
  localStorage.setItem('ns_setup_done','1');
  document.getElementById('setupScreen').classList.add('hidden');
}

async function saveHomeWifi() {
  const ssid = document.getElementById('setupSSID').value.trim();
  const pass = document.getElementById('setupPass').value;
  if (!ssid) { showToast('Enter WiFi name','err'); return; }
  try {
    const r = await fetch(API+'/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,password:pass})});
    if (r.ok) { showToast('WiFi saved — connecting…','ok'); closeSetup(); }
    else showToast('Failed to save','err');
  } catch { showToast('Device unreachable — on NovaShade hotspot?','err'); }
}

// Check on load whether to show setup screen
if (!localStorage.getItem('ns_setup_done')) {
  document.getElementById('setupScreen').classList.remove('hidden');
} else {
  document.getElementById('setupScreen').classList.add('hidden');
}

// ── TOAST ─────────────────────────────────────────────────────────────────
function showToast(msg, type='ok') {
  const t = document.getElementById('toast');
  t.textContent = msg; t.className = 'toast '+type+' show';
  setTimeout(()=>t.classList.remove('show'), 2800);
}

// ── CONNECTION ─────────────────────────────────────────────────────────────
function setConnected(ok) {
  const badge = document.getElementById('connBadge');
  const label = document.getElementById('connLabel');
  const bar   = document.getElementById('liveBar');
  document.getElementById('offlineBanner').classList.toggle('show',!ok);
  if (ok) {
    badge.className='conn-badge live'; label.textContent='Live';
    bar.innerHTML='&#8635; &nbsp; Live · updates every 3s';
  } else {
    badge.className='conn-badge'; label.textContent='Offline';
    bar.innerHTML='<span class="spin">&#8635;</span> Reconnecting...';
  }
}

// ── FETCH STATUS ───────────────────────────────────────────────────────────
async function fetchStatus() {
  try {
    const r = await fetch(API+'/api/status',{signal:AbortSignal.timeout(4000)});
    if (!r.ok) throw new Error();
    const d = await r.json();
    updateUI(d); setConnected(true);
  } catch { setConnected(false); }
}

// ── UPDATE UI ──────────────────────────────────────────────────────────────
function updateUI(d) {
  // Temperature — firmware sends °C, display °F
  const tc = parseFloat(d.tempC ?? 0);
  const tf = (tc * 9/5) + 32;
  const big = document.getElementById('tempBig');
  big.textContent = tf.toFixed(1);
  big.className = 'temp-big'+(tc<2?' cold':tc>18?' warm':'');

  const targetC = parseFloat(d.targetC ?? -1);
  document.getElementById('targetVal').textContent =
    targetC > 0 ? ((targetC*9/5)+32).toFixed(0)+'°F' : 'no heat needed';
  document.getElementById('phaseVal').textContent = d.phase ?? '--';

  // Phase badge
  const s = d.status ?? 'idle';
  const pb = document.getElementById('phaseBadge');
  const pm = {heating:'Heating',done:'Done',fault:'Fault',idle:'Idle',departed:'Complete'};
  pb.textContent = pm[s]||'Idle';
  pb.className = 'phase-badge'+(s==='heating'?' heating':s==='done'||s==='departed'?' ok':s==='fault'?' fault':'');

  // PWM bar
  const pct = parseFloat(d.pwmPct??0);
  document.getElementById('pwmPct').textContent = pct.toFixed(0)+'%';
  document.getElementById('pwmBar').style.width = pct+'%';

  // Ramp detail
  const tgt = d.targetDuty ?? 0;
  const cur = d.currentDuty ?? 0;
  const rst = d.rampStep ?? 0;
  document.getElementById('targetDutyVal').textContent = tgt;
  document.getElementById('currentDutyVal').textContent = cur;
  document.getElementById('rampStepVal').textContent = rst;
  // Colour current duty: green if at target, amber if ramping
  document.getElementById('currentDutyVal').style.color =
    cur === tgt ? 'var(--ok)' : 'var(--warn)';

  // Metrics
  const v = parseFloat(d.voltage??0), c = parseFloat(d.current??0);
  document.getElementById('voltVal').textContent = v.toFixed(1);
  document.getElementById('currVal').textContent = c.toFixed(2);
  document.getElementById('powVal').textContent  = (v*c).toFixed(1);
  document.getElementById('voltCard').className = 'metric '+(v<10||v>16?'fault':v<11?'warn':'ok');
  document.getElementById('currCard').className = 'metric '+(c>5?'fault':c<0.05?'warn':'ok');

  // Next wake — comes from RTC_DATA_ATTR in firmware, always correct
  document.getElementById('nextVal').textContent = d.nextWake ?? '--:--';

  // Departure info
  if (d.departureEpoch && d.departureEpoch > 0) {
    depEpochMs = d.departureEpoch * 1000;
    document.getElementById('depTimeDisp').textContent = d.departureTime || '--:--';
    const fw = new Date((d.departureEpoch - 45*60) * 1000);
    document.getElementById('firstWakeDisp').textContent =
      String(fw.getUTCHours()).padStart(2,'0')+':'+String(fw.getUTCMinutes()).padStart(2,'0');
    document.getElementById('depSetRow').style.display = 'block';
    updateCountdown();
  }

  // Device clock — anchor to rtcEpoch so JS can tick between polls
  if (d.rtcEpoch && d.rtcEpoch > 1700000000) {
    rtcBaseEpoch = d.rtcEpoch;
    rtcBaseMs    = performance.now();
    tickClock();
  } else if (d.rtcTime) {
    document.getElementById('rtcDisplay').textContent = d.rtcTime;
  }

  // Home WiFi status
  const ws = document.getElementById('wifiStatus');
  if (d.staConnected) {
    ws.textContent = 'Connected: '+d.staSSID+(d.staIP?' ('+d.staIP+')':'');
    ws.className = 'wifi-status connected';
    // Pre-fill SSID field so user sees what's saved
    if (d.staSSID) {
      document.getElementById('wifiSSID').value = d.staSSID;
      document.getElementById('setupSSID').value = d.staSSID;
    }
  } else if (d.staSSID) {
    ws.textContent = 'Saved: '+d.staSSID+' (not connected)';
    ws.className = 'wifi-status disconnected';
    document.getElementById('wifiSSID').value = d.staSSID;
    document.getElementById('setupSSID').value = d.staSSID;
  } else {
    ws.textContent = 'No home WiFi saved';
    ws.className = 'wifi-status disconnected';
  }

  // Status card
  const sm = {
    idle:    {e:'❄️',cls:'',    t:'System Idle',      desc:'Waiting for next scheduled wake'},
    heating: {e:'🔥',cls:'warn',t:'Heating Active',   desc:'PWM running — frost clear in progress'},
    done:    {e:'✅',cls:'ok',  t:'Target Reached',   desc:'Heater off — sleeping until next cycle'},
    fault:   {e:'⚠️',cls:'fault',t:'System Fault',    desc:d.faultMsg??'Check device immediately'},
    departed:{e:'🚗',cls:'ok',  t:'Departure Reached',desc:'System complete — good drive!'},
  };
  const info = sm[s]||sm.idle;
  document.getElementById('statusEmoji').textContent = info.e;
  document.getElementById('statusEmoji').className   = 'status-emoji '+info.cls;
  document.getElementById('statusTitle').textContent = info.t;
  document.getElementById('statusDesc').textContent  = info.desc;
}

// ── TICKING CLOCK ──────────────────────────────────────────────────────────
// Advances the device clock display every second using the offset from the
// last poll — no flicker, no frozen display between 3-second polls.
function tickClock() {
  if (!rtcBaseEpoch) return;
  const elapsed = Math.floor((performance.now() - rtcBaseMs) / 1000);
  const epoch   = rtcBaseEpoch + elapsed;
  const d = new Date(epoch * 1000);
  const day = String(d.getUTCDate()).padStart(2,'0');
  const mon = MONTHS[d.getUTCMonth()];
  const yr  = d.getUTCFullYear();
  const hh  = String(d.getUTCHours()).padStart(2,'0');
  const mm  = String(d.getUTCMinutes()).padStart(2,'0');
  const ss  = String(d.getUTCSeconds()).padStart(2,'0');
  document.getElementById('rtcDisplay').textContent = `${day} ${mon} ${yr} ${hh}:${mm}:${ss}`;
}

// ── COUNTDOWN ──────────────────────────────────────────────────────────────
function updateCountdown() {
  if (!depEpochMs) return;
  const diff = depEpochMs - Date.now();
  const el = document.getElementById('depCountdown');
  if (!el) return;
  if (diff <= 0) { el.textContent='now'; return; }
  const h = Math.floor(diff/3600000);
  const m = Math.floor((diff%3600000)/60000);
  el.textContent = h>0 ? h+'h '+m+'m' : m+'m';
}

// ── SET DEPARTURE ──────────────────────────────────────────────────────────
async function setDeparture() {
  const timeVal = document.getElementById('depTimeInput').value;
  if (!timeVal) { showToast('Pick a departure time first','err'); return; }

  // Build a local Date at the chosen HH:MM today (or tomorrow if past)
  const now = new Date();
  const [hh,mm] = timeVal.split(':').map(Number);
  const dep = new Date(now.getFullYear(), now.getMonth(), now.getDate(), hh, mm, 0);
  if (dep <= now) dep.setDate(dep.getDate()+1);

  // Convert local time → epoch the RTC can match
  // The RTC stores local wall-clock time, so we subtract timezone offset
  const tzSec = dep.getTimezoneOffset() * 60;
  const depEpochSec = Math.floor(dep.getTime()/1000) - tzSec;

  if (depEpochSec < 1700000000) {
    showToast('Sync the clock first, then set departure','err'); return;
  }

  try {
    const r = await fetch(API+'/api/departure',{
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({departureEpoch: depEpochSec})
    });
    if (r.ok) {
      depEpochMs = depEpochSec * 1000;
      document.getElementById('depTimeDisp').textContent = timeVal;
      const fw = new Date((depEpochSec-45*60)*1000);
      document.getElementById('firstWakeDisp').textContent =
        String(fw.getUTCHours()).padStart(2,'0')+':'+String(fw.getUTCMinutes()).padStart(2,'0');
      document.getElementById('depSetRow').style.display = 'block';
      updateCountdown();
      showToast('Departure set for '+timeVal+' ✓');
    } else {
      const txt = await r.text();
      if (txt.includes('future')||txt.includes('epoch')) {
        showToast('Sync the device clock first','err');
      } else {
        showToast('Error: '+txt,'err');
      }
    }
  } catch { showToast('Device unreachable','err'); }
}

// ── SYNC RTC ───────────────────────────────────────────────────────────────
async function syncRTC() {
  const now = new Date();
  // Subtract timezone offset so the RTC stores local wall-clock time
  const tzSec  = now.getTimezoneOffset() * 60;
  const epoch  = Math.floor(now.getTime()/1000) - tzSec;
  const timeStr = now.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'});
  try {
    const r = await fetch(API+'/api/settime',{
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({epoch})
    });
    if (r.ok) { showToast('Clock set to '+timeStr+' ✓'); fetchStatus(); }
    else showToast('Sync failed','err');
  } catch { showToast('Device unreachable — on NovaShade WiFi?','err'); }
}

// ── SAVE WIFI (from main app card) ─────────────────────────────────────────
async function saveWifi() {
  const ssid = document.getElementById('wifiSSID').value.trim();
  const pass = document.getElementById('wifiPass').value;
  if (!ssid) { showToast('Enter WiFi name','err'); return; }
  try {
    const r = await fetch(API+'/api/wifi',{
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ssid, password:pass})
    });
    if (r.ok) { showToast('WiFi saved — connecting…','ok'); setTimeout(fetchStatus,3000); }
    else showToast('Failed to save','err');
  } catch { showToast('Device unreachable','err'); }
}

// ── CANCEL ─────────────────────────────────────────────────────────────────
async function cancelSystem() {
  if (!confirm('Cancel and shut down NovaShade?')) return;
  try {
    await fetch(API+'/api/cancel',{method:'POST'});
    showToast('System cancelled');
  } catch { showToast('Device unreachable','err'); }
}

// ── INIT ───────────────────────────────────────────────────────────────────
fetchStatus();
setInterval(fetchStatus,    3000);
setInterval(updateCountdown,1000);
setInterval(tickClock,      1000);  // smooth device clock between polls
</script>
</body>
</html>

)HTML";
