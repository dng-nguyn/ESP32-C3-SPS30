/**
 * ESP32-C3 + SPS30 + MQTT (256dpi) + Deep Sleep
 * sensor-first, Wi-Fi/MQTT-last, dynamic SPS30 warmup (8s / 16s / 30s), LED indicator.
 *
 * What this sketch does:
 * - Powers up and initializes the SPS30
 * - Follows the SPS30 datasheet warmup guidance: try to read at 8s and 16s, else 30s
 * - After measuring, SPS30 is put into sleep mode.
 * - Data ready -> bring up Wi-Fi + MQTT -> publish averaged values over 30 seconds.
 * - kill wifi before deep sleep for 3 minutes
 * - every 1440 boots perform a sensor clean up
 * - uses an optional ws2812 (onboard esp32-c3 zero) for debug status.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <SensirionUartSps30.h>
#include <Adafruit_NeoPixel.h>
#include <esp_sleep.h>

// ================= USER CONFIG =================
const int SLEEP_INTERVAL_SECONDS   = 180; // time to sleep between measurements
const int AVERAGING_WINDOW_SECONDS = 30; // time to average measurements, each measurement is sampled every second.

/** 
max time for startup, refer to datasheet. Minimum recommended is 30 seconds.
this script uses dynamic warmup time to maximize efficiency
See more: https://sensirion.com/media/documents/188A2C3C/6166F165/Sensirion_Particulate_Matter_AppNotes_SPS30_Low_Power_Operation_D1.pdf, Section 2.1
*/
const int STARTUP_WAIT_SECONDS     = 30; 
// sps30 pin 2 is rx and pin 3 is tx
int sensorRxPin         = 1;          // RX pin on sensor connected to TX on esp32
int sensorTxPin         = 0;          // TX pin on sensor connected to RX on esp32
// led config
const bool USE_DEBUG_LED = true; // false to disable onboard led or if your board doesn't have one

#define LED_PIN    10 // gpio pin used for led. esp32-c3 zero is on gpio10.
#define NUM_LEDS   1

// every 3 days → 3*24*60*60 = 259200 s
// with 180 s wake interval → 259200 / 180 = 1440 boots
const uint32_t CLEANING_INTERVAL_BOOT =
  (3UL * 24UL * 60UL * 60UL) / SLEEP_INTERVAL_SECONDS;  // = 1440 with current settings

// WiFi/MQTT credentials
const char* ssid     = "ssid";
const char* password = "password";
IPAddress local_IP(192, 168, 1, 254);
IPAddress gateway (192, 168, 1, 1);
IPAddress subnet  (255, 255, 255, 0);

const char* mqtt_server    = "192.168.1.255";
const int   mqtt_port      = 1883;
const char* mqtt_client_id = "esp32-sps30-sensor-low-power";
const char* MQTT_USER      = "mqtt";
const char* MQTT_PASS      = "mqtt";

// =============== HOME ASSISTANT MQTT DISCOVERY ===============
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* HA_DEVICE_NAME      = "SPS30 Outdoor Sensor";
const char* HA_DEVICE_MODEL     = "SPS30 + ESP32-C3 Zero";
const char* HA_DEVICE_MFR       = "Sensirion";

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// errors
const uint8_t LED_WIFI_R   = 161, LED_WIFI_G   = 0,   LED_WIFI_B   = 0;   // red: wifi error
const uint8_t LED_MQTT_R   = 121, LED_MQTT_G   = 0,   LED_MQTT_B   = 121; // purple: mqtt error
const uint8_t LED_SENSOR_R = 0,   LED_SENSOR_G = 44,  LED_SENSOR_B = 44;  // cyan: sensor error

// phases
const uint8_t LED_START_R  = 134, LED_START_G  = 57,  LED_START_B  = 0;   // orange: sensor warming up
const uint8_t LED_MEAS_R   = 0,   LED_MEAS_G   = 48,  LED_MEAS_B   = 0;   // green: sensor measuring

// breathing phases timing
const uint16_t BREATH_DURATION_MS     = 2000;          // 1 s up + 1 s down
const uint16_t BREATH_UPDATES_PER_SEC = 120;           // 120 updates per second
const uint16_t BREATH_TOTAL_STEPS     =
    (BREATH_DURATION_MS / 1000.0) * BREATH_UPDATES_PER_SEC; // 2000/1000 * 120 = 240

// =================================================
#define SENSOR_SERIAL_INTERFACE Serial1
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionUartSps30 sensor;
static int16_t error;

WiFiClient net;
MQTTClient mqttClient(256);

// survives deep sleep
RTC_DATA_ATTR int bootCount = 0;

// =================================================
// helper struct for averaged data
struct Sps30Averages {
  bool valid;
  float mc1p0, mc2p5, mc4p0, mc10p0;
  float nc0p5, nc1p0, nc2p5, nc4p0, nc10p0;
  float tps;
  int readings;
};

// =================================================
// LED helpers

void ledColorRaw(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void ledOff() {
  if (!USE_DEBUG_LED) return;
  ledColorRaw(0, 0, 0);
}

void ledColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!USE_DEBUG_LED) return;
  ledColorRaw(r, g, b);
}

// Only hold when LED is enabled (prevents delays for users who disable LED)
inline void ledHold(uint32_t ms) {
  if (USE_DEBUG_LED) delay(ms);
}

/**
 * Blocking 2-second breathing effect.
 * If LED is disabled, return immediately (no waiting).
 */
void ledBreath2s(uint8_t r, uint8_t g, uint8_t b, bool pumpMQTT = false) {
  if (!USE_DEBUG_LED) {
    // LED disabled → no visual and no waiting
    return;
  }

  uint16_t steps = BREATH_TOTAL_STEPS;   // 240
  if (steps < 2) steps = 2;
  uint16_t half = steps / 2;             // 120 → 1s
  if (half == 0) half = 1;
  uint16_t stepDelay = BREATH_DURATION_MS / steps;  // ~8 ms

  // fade in
  for (uint16_t i = 0; i < half; i++) {
    uint8_t cr = (uint32_t)r * i / half;
    uint8_t cg = (uint32_t)g * i / half;
    uint8_t cb = (uint32_t)b * i / half;
    ledColorRaw(cr, cg, cb);
    if (pumpMQTT && (i % 6 == 0)) mqttClient.loop();
    delay(stepDelay);
  }

  // fade out
  for (uint16_t i = 0; i < half; i++) {
    uint8_t cr = (uint32_t)r * (half - i) / half;
    uint8_t cg = (uint32_t)g * (half - i) / half;
    uint8_t cb = (uint32_t)b * (half - i) / half;
    ledColorRaw(cr, cg, cb);
    if (pumpMQTT && (i % 6 == 0)) mqttClient.loop();
    delay(stepDelay);
  }

  ledColorRaw(0, 0, 0);
}

// ================= Non-blocking LED breather =================
struct LedBreather {
  bool active = false;
  uint8_t r = 0, g = 0, b = 0;
  uint32_t start_ms = 0;
  uint16_t duration_ms = BREATH_DURATION_MS; // defaults to 2000ms
} ledBreather;

void ledBreathStart(uint8_t r, uint8_t g, uint8_t b, uint16_t duration_ms = BREATH_DURATION_MS) {
  if (!USE_DEBUG_LED) return;
  ledBreather.active = true;
  ledBreather.r = r; ledBreather.g = g; ledBreather.b = b;
  ledBreather.duration_ms = duration_ms ? duration_ms : 2000;
  ledBreather.start_ms = millis();
}

void ledBreathStop() {
  ledBreather.active = false;
  ledOff();
}

/**
 * Call this frequently (e.g., inside loops). It NEVER blocks.
 * Produces a triangle-wave "breathing" over ledBreather.duration_ms.
 */
void ledBreathTick() {
  if (!USE_DEBUG_LED || !ledBreather.active) return;

  uint32_t now = millis();
  uint32_t elapsed = (now - ledBreather.start_ms) % ledBreather.duration_ms;
  uint32_t half = ledBreather.duration_ms / 2;
  if (half == 0) half = 1;

  // 0..1 up, then 1..0 down
  float phase = (elapsed < half)
                  ? (float)elapsed / half
                  : (float)(ledBreather.duration_ms - elapsed) / half;

  uint8_t cr = (uint8_t)((float)ledBreather.r * phase);
  uint8_t cg = (uint8_t)((float)ledBreather.g * phase);
  uint8_t cb = (uint8_t)((float)ledBreather.b * phase);
  ledColorRaw(cr, cg, cb);
}

// =================================================

void enter_deep_sleep() {
  Serial.flush();
  // radio cleanup
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // timer
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  // add DNS = gateway for static config
  WiFi.config(local_IP, gateway, subnet, gateway);
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++tries > 40) { break; }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    return true;
  }
  Serial.println("\nWiFi FAILED.");
  ledColor(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B);  // red for WiFi error
  return false;
}

bool connectMQTT() {
  Serial.printf("Connecting to MQTT %s:%d ...\n", mqtt_server, mqtt_port);
  for (int i = 0; i < 5; i++) {
    if (mqttClient.connect(mqtt_client_id, MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT connected.");
      ledOff();
      return true;
    }
    Serial.print("MQTT connect failed, code = ");
    Serial.println(mqttClient.lastError());
    delay(500);
  }
  Serial.println("MQTT connect FAILED.");
  ledColor(LED_MQTT_R, LED_MQTT_G, LED_MQTT_B);  // purple for MQTT error
  return false;
}

/**
 * Blocking publish using QoS 1.
 * - returns true only if the broker ACKed (PUBACK) within timeout
 * - does a short drain after to process anything else
 */
bool publishFloatBlocking(const char* topic, float value, uint16_t extraDrainMs = 80) {
  if (!mqttClient.connected()) {
    Serial.println("publishFloatBlocking: MQTT not connected.");
    return false;
  }

  String payload = String(value, 2);

  // QoS 1 + retained
  bool ok = mqttClient.publish(topic, payload, true, 1);
  if (!ok) {
    Serial.print("publishFloatBlocking FAILED for topic: ");
    Serial.println(topic);
    return false;
  }

  // drain a bit to finish any pending packets
  unsigned long end = millis() + extraDrainMs;
  while (millis() < end) {
    mqttClient.loop();
    delay(5);
  }

  Serial.print("  "); Serial.print(topic); Serial.print(" = "); Serial.println(payload);
  return true;
}

/**
 * Proper SPS30 wake-up for UART mode:
 * 1) send 0xFF to create the low pulse on RX
 * 2) within 100 ms: send wake-up command
 * 3) if first wake-up just enabled the interface, send second one
 */
bool wakeSps30FromSleep() {
  // step 1: activate interface
  SENSOR_SERIAL_INTERFACE.write(0xFF);
  SENSOR_SERIAL_INTERFACE.flush();
  delay(5);   // must be < 100 ms

  // step 2: send wake-up
  int16_t err = sensor.wakeUp();
  if (err) {
    Serial.printf("First wakeUp() failed with %d, retrying...\n", err);
    delay(10);
    err = sensor.wakeUp();   // second wake-up per datasheet note
  }
  return (err == NO_ERROR);
}

/**
 * Dynamic SPS30 startup strictly per datasheet:
 * - we attempt to read at 8 s and 16 s
 * - otherwise we just keep breathing until 30 s
 * - no “immediate” reads before 8 s
 *
 * 200–3000 #/cm3 → 8 s
 * 100–200  #/cm3 → 16 s
 * else → 30 s
 */
int dynamicStartupWait() {
  int elapsed_s = 0;

  while (elapsed_s < STARTUP_WAIT_SECONDS) {
    // keep the original blocking breath for startup phase
    if (USE_DEBUG_LED) {
      ledBreath2s(LED_START_R, LED_START_G, LED_START_B);
    } else {
      // LED disabled → still actually wait to respect warmup timing
      delay(BREATH_DURATION_MS);
    }
    elapsed_s += 2;

    if (elapsed_s == 8 || elapsed_s == 16) {
      float mc1p0, mc2p5, mc4p0, mc10p0;
      float nc0p5, nc1p0, nc2p5, nc4p0, nc10p0;
      float tps;
      int16_t err = sensor.readMeasurementValuesFloat(
        mc1p0, mc2p5, mc4p0, mc10p0,
        nc0p5, nc1p0, nc2p5, nc4p0, nc10p0,
        tps
      );

      if (err == NO_ERROR) {
        Serial.printf("[warmup] t=%ds, nc10p0=%.1f\n", elapsed_s, nc10p0);

        if (elapsed_s == 8 &&
            nc10p0 >= 200.0f && nc10p0 <= 3000.0f) {
          Serial.println("[warmup] datasheet match: 200–3000 → 8 s");
          return elapsed_s;
        }

        if (elapsed_s == 16 &&
            nc10p0 >= 100.0f && nc10p0 < 200.0f) {
          Serial.println("[warmup] datasheet match: 100–200 → 16 s");
          return elapsed_s;
        }
      } else {
        // log failed read
        Serial.printf("[warmup] t=%ds, read FAILED with error: %d\n", elapsed_s, err);
      }
    }
  }

  Serial.println("[warmup] ready to measure.");
  return STARTUP_WAIT_SECONDS;
}

// -------------------------------------------------
// measurement + averaging (sensor is already measuring)
// 1 Hz sampling with non-blocking LED breathing
const uint16_t MEAS_SAMPLE_PERIOD_MS = 1000;

Sps30Averages measureForWindow() {
  Sps30Averages avg;
  avg.valid = false;
  avg.readings = 0;
  avg.mc1p0 = avg.mc2p5 = avg.mc4p0 = avg.mc10p0 = 0;
  avg.nc0p5 = avg.nc1p0 = avg.nc2p5 = avg.nc4p0 = avg.nc10p0 = 0;
  avg.tps   = 0;

  Serial.printf("Averaging data over %d seconds at 1 Hz samples...\n", AVERAGING_WINDOW_SECONDS);

  uint32_t start = millis();
  uint32_t nextSample = start;  // first sample now
  uint32_t endTime = start + (uint32_t)AVERAGING_WINDOW_SECONDS * 1000UL;

  // Start a non-blocking breath while measuring
  ledBreathStart(LED_MEAS_R, LED_MEAS_G, LED_MEAS_B, BREATH_DURATION_MS);

  while ((int32_t)(millis() - endTime) < 0) {
    // time to sample?
    uint32_t now = millis();
    if ((int32_t)(now - nextSample) >= 0) {
      float mc1p0, mc2p5, mc4p0, mc10p0;
      float nc0p5, nc1p0, nc2p5, nc4p0, nc10p0;
      float typicalParticleSize;
      int16_t err = sensor.readMeasurementValuesFloat(
        mc1p0, mc2p5, mc4p0, mc10p0,
        nc0p5, nc1p0, nc2p5, nc4p0, nc10p0,
        typicalParticleSize
      );

      if (err == NO_ERROR) {
        avg.readings++;
        avg.mc1p0 += mc1p0; avg.mc2p5 += mc2p5; avg.mc4p0 += mc4p0; avg.mc10p0 += mc10p0;
        avg.nc0p5 += nc0p5; avg.nc1p0 += nc1p0; avg.nc2p5 += nc2p5; avg.nc4p0 += nc4p0; avg.nc10p0 += nc10p0;
        avg.tps   += typicalParticleSize;
      } else {
        // optional: Serial.printf("read error: %d\n", err);
      }

      nextSample += MEAS_SAMPLE_PERIOD_MS;
    }

    // keep LED animation smooth
    ledBreathTick();

    // small yield; also keeps WiFi stack snappy if it happens to be up
    delay(5);
  }

  ledBreathStop();

  if (avg.readings > 0) {
    avg.valid = true;
    avg.mc1p0  /= avg.readings;
    avg.mc2p5  /= avg.readings;
    avg.mc4p0  /= avg.readings;
    avg.mc10p0 /= avg.readings;

    avg.nc0p5  /= avg.readings;
    avg.nc1p0  /= avg.readings;
    avg.nc2p5  /= avg.readings;
    avg.nc4p0  /= avg.readings;
    avg.nc10p0 /= avg.readings;

    avg.tps    /= avg.readings;
  }

  return avg;
}

// =================================================
// ======== HOME ASSISTANT DISCOVERY HELPERS =======
// =================================================

bool publishHADiscoverySensor(
  const char* object_id,
  const char* name,
  const char* state_topic,
  const char* unit,
  const char* device_class,
  const char* icon
) {
  if (!mqttClient.connected()) {
    return false;
  }

  // Topic: homeassistant/sensor/<device_id>/<object_id>/config
  char topic[160];
  snprintf(topic, sizeof(topic),
           "%s/sensor/%s/%s/config",
           HA_DISCOVERY_PREFIX, mqtt_client_id, object_id);

  // Build JSON payload
  String payload = "{";
  payload += "\"name\":\"" + String(name) + "\",";
  payload += "\"state_topic\":\"" + String(state_topic) + "\",";
  if (unit && unit[0] != '\0') {
    payload += "\"unit_of_measurement\":\"" + String(unit) + "\",";
  }
  if (device_class && device_class[0] != '\0') {
    payload += "\"device_class\":\"" + String(device_class) + "\",";
  }
  if (icon && icon[0] != '\0') {
    payload += "\"icon\":\"" + String(icon) + "\",";
  }
  payload += "\"state_class\":\"measurement\",";
  payload += "\"unique_id\":\"" + String(mqtt_client_id) + "_" + String(object_id) + "\",";
  payload += "\"device\":{";
    payload += "\"identifiers\":[\"" + String(mqtt_client_id) + "\"],";
    payload += "\"name\":\"" + String(HA_DEVICE_NAME) + "\",";
    payload += "\"model\":\"" + String(HA_DEVICE_MODEL) + "\",";
    payload += "\"manufacturer\":\"" + String(HA_DEVICE_MFR) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqttClient.publish(topic, payload, true, 1);

  // short drain
  unsigned long end = millis() + 120;
  while (millis() < end) {
    mqttClient.loop();
    delay(5);
  }
  return ok;
}

void publishAllHADiscovery() {
  // Mass concentrations (µg/m³)
  publishHADiscoverySensor("mc1p0",  "SPS30 PM1.0",  "sensor/sps30/mc1p0",  "µg/m³", "pm1",  nullptr);
  publishHADiscoverySensor("mc2p5",  "SPS30 PM2.5",  "sensor/sps30/mc2p5",  "µg/m³", "pm25", nullptr);
  publishHADiscoverySensor("mc4p0",  "SPS30 PM4.0",  "sensor/sps30/mc4p0",  "µg/m³", "pm4",  nullptr);
  publishHADiscoverySensor("mc10p0", "SPS30 PM10",   "sensor/sps30/mc10p0", "µg/m³", "pm10", nullptr);

  // Number concentrations (#/cm³)
  publishHADiscoverySensor("nc0p5",  "SPS30 NC0.5",  "sensor/sps30/nc0p5",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc1p0",  "SPS30 NC1.0",  "sensor/sps30/nc1p0",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc2p5",  "SPS30 NC2.5",  "sensor/sps30/nc2p5",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc4p0",  "SPS30 NC4.0",  "sensor/sps30/nc4p0",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc10p0", "SPS30 NC10",   "sensor/sps30/nc10p0", "#/cm³", "", "mdi:counter");

  // Typical particle size (µm)
  publishHADiscoverySensor("tps",    "SPS30 Particle Size", "sensor/sps30/typical_particle_size", "µm", "", "mdi:circle-opacity");
}

// =================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  bootCount++; // Increment the boot count
  Serial.println("\n=== ESP32-C3 SPS30 MQTT ===");
  Serial.printf("Boot count: %d\n", bootCount);

  pixels.begin();
  ledOff();

  // UART to SPS30
  SENSOR_SERIAL_INTERFACE.begin(115200, SERIAL_8N1, sensorRxPin, sensorTxPin);
  sensor.begin(SENSOR_SERIAL_INTERFACE);

  // --- Phase 1: Initialize/wake sensor ---
  Serial.println("\n--- Phase 1: Inittialize sensor ---");
  bool firstBoot = (bootCount == 1);

  if (firstBoot) {
    Serial.println("First boot detected. Resetting sensor to ensure clean state.");
    int8_t serial[32];
    int16_t errSN = sensor.readSerialNumber(serial, sizeof(serial));
    if (errSN == 0) {
      Serial.print("Sensor Serial Number: ");
      for (int i = 0; i < sizeof(serial); i++) {
        if (serial[i] == 0) break;  // stop at null terminator
        Serial.print((char)serial[i]);
      }
      Serial.println();
    } else {
      Serial.printf("Failed to read serial number, error %d\n", errSN);
    }

    uint8_t fwMaj, fwMin, r1, hwRev, r2, shdlcMaj, shdlcMin;
    int16_t errV = sensor.readVersion(fwMaj, fwMin, r1, hwRev, r2, shdlcMaj, shdlcMin);
    if (errV == 0) {
      Serial.printf("FW v%d.%d | HW rev %d | SHDLC v%d.%d\n",
                    fwMaj, fwMin, hwRev, shdlcMaj, shdlcMin);
    } else {
      Serial.printf("Failed to read version info, error %d\n", errV);
    }
    error = sensor.deviceReset();
    if (error) {
      Serial.print("FATAL: Could not reset sensor. Error: ");
      Serial.println(error);
      ledColor(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B);  // cyan
      enter_deep_sleep();
    }
    delay(200);
  } else {
    Serial.println("Waking from deep sleep. Kicking UART + wakeUp().");
    if (!wakeSps30FromSleep()) {
      Serial.println("FATAL: Could not wake sensor from sleep.");
      ledColor(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B);  // cyan
      enter_deep_sleep();
    }
  }
  Serial.println("Sensor is now in Idle Mode.");

  // optional 3-day manual fan cleaning
  bool doCleaning = false;
  if (CLEANING_INTERVAL_BOOT > 0) {
    // run every N boots, but skip very first boot
    if ((bootCount > 1) && ((bootCount - 1) % CLEANING_INTERVAL_BOOT == 0)) {
      doCleaning = true;
    }
  }
  if (doCleaning) {
    Serial.println("Running SPS30 manual fan cleaning...");
    int16_t cErr = sensor.startFanCleaning();
    if (cErr) {
      Serial.printf("Warning: manual fan cleaning failed: %d\n", cErr);
    }
    // typical time 10s, give it a moment
    delay(10000);
  }

  // --- Phase 2: start measurement ---
  Serial.println("\n--- Phase 2: Starting measurement... ---");
  error = sensor.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
  if (error) {
    Serial.print("FATAL: Could not start measurement. Error: ");
    Serial.println(error);
    ledColor(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B);  // cyan
    enter_deep_sleep();
  }
  Serial.println("Measurement started.");

  // --- Dynamic warmup wait (8/16/30) ---
  Serial.println("Waiting for sensor warmup...");
  int waited = dynamicStartupWait();
  Serial.printf("Warmup done after ~%d seconds.\n", waited);

  // --- Averaging window (non-blocking LED, 1 Hz sampling) ---
  Sps30Averages averages = measureForWindow();

  // --- Stop measurement + sleep sensor ---
  Serial.println("\n--- Phase 3: Stopping measurement (to Idle Mode)... ---");
  error = sensor.stopMeasurement();
  if (error) {
    Serial.printf("Warning: Error stopping measurement: %d\n", error);
  } else {
    Serial.println("Sensor stopped successfully.");
  }

  Serial.println("\n--- Phase 4: Putting sensor to sleep... ---");
  error = sensor.sleep();
  if (error) {
    Serial.printf("Warning: Error putting sensor to sleep: %d\n", error);
  } else {
    Serial.println("Sensor sleep command sent successfully.");
  }
  delay(30);

  // -------------------------------------------------
  // bring up Wi-Fi + MQTT to publish
  // -------------------------------------------------
  bool wifiOK = connectWiFi();
  if (!wifiOK) {
    Serial.println("Wi-Fi not available, going to deep sleep.");
    ledColor(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B);
    ledHold(10000); 
    ledOff();
    enter_deep_sleep();
  }

  mqttClient.begin(mqtt_server, mqtt_port, net);
  mqttClient.setTimeout(5000);  // 5s for QoS 1 ops
  bool mqttOK = connectMQTT();

  // Publish HA discovery as soon as MQTT is up so HA can see the entities
  if (mqttOK) {
    publishAllHADiscovery();
  }

  if (averages.valid && mqttOK) {
    Serial.println("\nPublishing values...");
    bool allPublishOK = true;

    allPublishOK &= publishFloatBlocking("sensor/sps30/mc1p0", averages.mc1p0);
    allPublishOK &= publishFloatBlocking("sensor/sps30/mc2p5", averages.mc2p5);
    allPublishOK &= publishFloatBlocking("sensor/sps30/mc4p0", averages.mc4p0);
    allPublishOK &= publishFloatBlocking("sensor/sps30/mc10p0", averages.mc10p0);

    allPublishOK &= publishFloatBlocking("sensor/sps30/nc0p5", averages.nc0p5);
    allPublishOK &= publishFloatBlocking("sensor/sps30/nc1p0", averages.nc1p0);
    allPublishOK &= publishFloatBlocking("sensor/sps30/nc2p5", averages.nc2p5);
    allPublishOK &= publishFloatBlocking("sensor/sps30/nc4p0", averages.nc4p0);
    allPublishOK &= publishFloatBlocking("sensor/sps30/nc10p0", averages.nc10p0);

    allPublishOK &= publishFloatBlocking("sensor/sps30/typical_particle_size", averages.tps);

    // final drain to be 100% sure all acks came back
    unsigned long drainEnd = millis() + 300;
    while (millis() < drainEnd) {
      mqttClient.loop();
      delay(10);
    }

    if (!allPublishOK) {
      Serial.println("One or more MQTT publishes failed!");
      ledColor(LED_MQTT_R, LED_MQTT_G, LED_MQTT_B);
      ledHold(5000);
    }
  } else {
    if (!averages.valid && !mqttOK) {
      Serial.println("\nNo valid readings AND MQTT not connected, skipping publish.");

      if (USE_DEBUG_LED) {
        // 10s total via 5 x 2s breaths, alternating:
        // cyan (sensor error) ↔ purple (MQTT error)
        for (int i = 0; i < 5; ++i) {
          if ((i % 2) == 0) {
            ledBreath2s(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B); // cyan
          } else {
            ledBreath2s(LED_MQTT_R,  LED_MQTT_G,  LED_MQTT_B);     // purple
          }
        }
        ledOff();
      }
      // LED disabled → no waiting at all

    } else if (!averages.valid) {
      Serial.println("\nNo valid sensor readings, skipping publish.");
      ledColor(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B); // cyan (static)
      ledHold(10000); 

    } else if (!mqttOK) {
      Serial.println("\nMQTT not connected, skipping publish.");
      ledColor(LED_MQTT_R, LED_MQTT_G, LED_MQTT_B);       // purple (static)
      ledHold(10000); 
    }
  }

  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  ledOff();
  Serial.printf("\nGoing to deep sleep for %d seconds...\n", SLEEP_INTERVAL_SECONDS);
  enter_deep_sleep();
}

void loop() {
  // not used
}
