/**
 * v2 build
 * 
 * Features:
 * - Welford's Algorithm (Mean, StdDev).
 * - Median calculation (Store & Sort).
 * - Sample Count tracking.
 * - Publishes Avg, Min, Max, StdDev, Median, Count to MQTT.
 * - Advertises ONLY Avg to Home Assistant.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <SensirionUartSps30.h>
#include <Adafruit_NeoPixel.h>
#include <esp_sleep.h>
#include <math.h>
#include <cfloat> // for FLT_MAX
#include <vector>
#include <algorithm> // for std::sort

// ================= USER CONFIG =================
const int SLEEP_INTERVAL_SECONDS   = 180; 
const int AVERAGING_WINDOW_SECONDS = 60;  
const int STARTUP_WAIT_SECONDS     = 30;  

int sensorRxPin         = 1;
int sensorTxPin         = 0;
const bool USE_DEBUG_LED = true; 

#define LED_PIN    10 
#define NUM_LEDS   1

// Cleaning every ~4 days
const uint32_t CLEANING_INTERVAL_BOOT = (3UL * 24UL * 60UL * 60UL) / SLEEP_INTERVAL_SECONDS;

// WiFi/MQTT
const char* ssid     = "ssid";
const char* password = "password";
IPAddress local_IP(192, 168, 1, 254);
IPAddress gateway (192, 168, 1, 1);
IPAddress subnet  (255, 255, 255, 0);

const char* mqtt_server    = "192.168.1.253";
const int   mqtt_port      = 1883;
const char* mqtt_client_id = "esp32-sps30-sensor-low-power";
const char* MQTT_USER      = "mqtt";
const char* MQTT_PASS      = "mqtt";

// HA Discovery
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* HA_DEVICE_NAME      = "SPS30 Outdoor Sensor";
const char* HA_DEVICE_MODEL     = "SPS30 + ESP32-C3 Zero";
const char* HA_DEVICE_MFR       = "Sensirion";

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Colors
const uint8_t LED_WIFI_R   = 161, LED_WIFI_G   = 0,   LED_WIFI_B   = 0;   // red
const uint8_t LED_MQTT_R   = 121, LED_MQTT_G   = 0,   LED_MQTT_B   = 121; // purple
const uint8_t LED_SENSOR_R = 0,   LED_SENSOR_G = 44,  LED_SENSOR_B = 44;  // cyan
const uint8_t LED_START_R  = 134, LED_START_G  = 57,  LED_START_B  = 0;   // orange
const uint8_t LED_MEAS_R   = 0,   LED_MEAS_G   = 48,  LED_MEAS_B   = 0;   // green

const uint16_t BREATH_DURATION_MS     = 2000;
const uint16_t BREATH_UPDATES_PER_SEC = 120;
const uint16_t BREATH_TOTAL_STEPS     = (BREATH_DURATION_MS / 1000.0) * BREATH_UPDATES_PER_SEC;

// =================================================
#define SENSOR_SERIAL_INTERFACE Serial1
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionUartSps30 sensor;
static int16_t error;

WiFiClient net;
MQTTClient mqttClient(512);

RTC_DATA_ATTR int bootCount = 0;

// =================================================
// Advanced Stats Class
class Sps30Metric {
  public:
    float mean;
    float minVal;
    float maxVal;
    
    std::vector<float> samples;

    Sps30Metric() { reset(); }

    void reset() {
      count = 0;
      mean = 0.0;
      M2 = 0.0;
      minVal = FLT_MAX;
      maxVal = -FLT_MAX;
      samples.clear();
      samples.reserve(AVERAGING_WINDOW_SECONDS + 5); 
    }

    void update(float x) {
      count++;
      samples.push_back(x);

      // Min/Max
      if (x < minVal) minVal = x;
      if (x > maxVal) maxVal = x;
      
      // Welford Mean/Variance
      float delta = x - mean;
      mean += delta / count;
      float delta2 = x - mean;
      M2 += delta * delta2;
    }

    float getStdDev() {
      if (count < 2) return 0.0;
      return sqrt(M2 / (count - 1));
    }

    float getMedian() {
      if (samples.empty()) return 0.0;
      std::sort(samples.begin(), samples.end());
      size_t n = samples.size();
      if (n % 2 == 0) {
        return (samples[n / 2 - 1] + samples[n / 2]) / 2.0;
      } else {
        return samples[n / 2];
      }
    }
    
    int getCount() { return count; }

  private:
    int count;
    float M2; 
};

struct Sps30Data {
  bool valid;
  int sampleCount; // Stores how many readings we took
  
  Sps30Metric mc1p0;
  Sps30Metric mc2p5;
  Sps30Metric mc4p0;
  Sps30Metric mc10p0;

  float nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, tps;
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

inline void ledHold(uint32_t ms) {
  if (USE_DEBUG_LED) delay(ms);
}

void ledBreath2s(uint8_t r, uint8_t g, uint8_t b, bool pumpMQTT = false) {
  if (!USE_DEBUG_LED) return;
  uint16_t steps = BREATH_TOTAL_STEPS;
  if (steps < 2) steps = 2;
  uint16_t half = steps / 2;
  uint16_t stepDelay = BREATH_DURATION_MS / steps;

  for (uint16_t i = 0; i < half; i++) {
    uint8_t cr = (uint32_t)r * i / half;
    uint8_t cg = (uint32_t)g * i / half;
    uint8_t cb = (uint32_t)b * i / half;
    ledColorRaw(cr, cg, cb);
    if (pumpMQTT && (i % 6 == 0)) mqttClient.loop();
    delay(stepDelay);
  }
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

struct LedBreather {
  bool active = false;
  uint8_t r = 0, g = 0, b = 0;
  uint32_t start_ms = 0;
  uint16_t duration_ms = BREATH_DURATION_MS;
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

void ledBreathTick() {
  if (!USE_DEBUG_LED || !ledBreather.active) return;
  uint32_t now = millis();
  uint32_t elapsed = (now - ledBreather.start_ms) % ledBreather.duration_ms;
  uint32_t half = ledBreather.duration_ms / 2;
  if (half == 0) half = 1;
  float phase = (elapsed < half) ? (float)elapsed / half : (float)(ledBreather.duration_ms - elapsed) / half;
  uint8_t cr = (uint8_t)((float)ledBreather.r * phase);
  uint8_t cg = (uint8_t)((float)ledBreather.g * phase);
  uint8_t cb = (uint8_t)((float)ledBreather.b * phase);
  ledColorRaw(cr, cg, cb);
}

// =================================================

void enter_deep_sleep() {
  Serial.flush();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  WiFi.config(local_IP, gateway, subnet, gateway);
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++tries > 40) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    return true;
  }
  Serial.println("\nWiFi FAILED.");
  ledColor(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B);
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
    delay(500);
  }
  Serial.println("MQTT connect FAILED.");
  ledColor(LED_MQTT_R, LED_MQTT_G, LED_MQTT_B);
  return false;
}

bool publishFloatBlocking(const char* topic, float value, uint16_t extraDrainMs = 80) {
  if (!mqttClient.connected()) return false;
  String payload = String(value, 2);
  bool ok = mqttClient.publish(topic, payload, true, 1);
  if (!ok) Serial.printf("Failed to publish %s\n", topic);
  
  unsigned long end = millis() + extraDrainMs;
  while (millis() < end) { mqttClient.loop(); delay(5); }
  return ok;
}

bool wakeSps30FromSleep() {
  SENSOR_SERIAL_INTERFACE.write(0xFF);
  SENSOR_SERIAL_INTERFACE.flush();
  delay(5);
  int16_t err = sensor.wakeUp();
  if (err) {
    delay(10);
    err = sensor.wakeUp();
  }
  return (err == NO_ERROR);
}

int dynamicStartupWait() {
  int elapsed_s = 0;
  while (elapsed_s < STARTUP_WAIT_SECONDS) {
    if (USE_DEBUG_LED) ledBreath2s(LED_START_R, LED_START_G, LED_START_B);
    else delay(BREATH_DURATION_MS);
    elapsed_s += 2;

    if (elapsed_s == 8 || elapsed_s == 16) {
      float mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, tps;
      int16_t err = sensor.readMeasurementValuesFloat(mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, tps);
      if (err == NO_ERROR) {
        Serial.printf("[warmup] t=%ds, nc10p0=%.1f\n", elapsed_s, nc10p0);
        if (elapsed_s == 8 && nc10p0 >= 200.0f && nc10p0 <= 3000.0f) return elapsed_s;
        if (elapsed_s == 16 && nc10p0 >= 100.0f && nc10p0 < 200.0f) return elapsed_s;
      }
    }
  }
  return STARTUP_WAIT_SECONDS;
}

// -------------------------------------------------
// measurement + averaging 
// 1 Hz sampling
const uint16_t MEAS_SAMPLE_PERIOD_MS = 1000;

Sps30Data measureForWindow() {
  Sps30Data data;
  data.valid = false;
  data.sampleCount = 0;
  data.mc1p0.reset(); data.mc2p5.reset(); data.mc4p0.reset(); data.mc10p0.reset();
  
  float sum_nc0p5=0, sum_nc1p0=0, sum_nc2p5=0, sum_nc4p0=0, sum_nc10p0=0, sum_tps=0;
  int count_nc = 0;

  Serial.printf("Sampling data over %d seconds...\n", AVERAGING_WINDOW_SECONDS);

  uint32_t start = millis();
  uint32_t nextSample = start;
  uint32_t endTime = start + (uint32_t)AVERAGING_WINDOW_SECONDS * 1000UL;

  ledBreathStart(LED_MEAS_R, LED_MEAS_G, LED_MEAS_B, BREATH_DURATION_MS);

  while ((int32_t)(millis() - endTime) < 0) {
    uint32_t now = millis();
    if ((int32_t)(now - nextSample) >= 0) {
      float mc1p0, mc2p5, mc4p0, mc10p0;
      float nc0p5, nc1p0, nc2p5, nc4p0, nc10p0;
      float tps;
      
      int16_t err = sensor.readMeasurementValuesFloat(
        mc1p0, mc2p5, mc4p0, mc10p0,
        nc0p5, nc1p0, nc2p5, nc4p0, nc10p0,
        tps
      );

      if (err == NO_ERROR) {
        // Updates for Mean/Min/Max/Std/Median
        data.mc1p0.update(mc1p0);
        data.mc2p5.update(mc2p5);
        data.mc4p0.update(mc4p0);
        data.mc10p0.update(mc10p0);

        // Simple Sums for NC
        sum_nc0p5 += nc0p5; sum_nc1p0 += nc1p0; sum_nc2p5 += nc2p5; 
        sum_nc4p0 += nc4p0; sum_nc10p0 += nc10p0; sum_tps += tps;
        count_nc++;
      }
      nextSample += MEAS_SAMPLE_PERIOD_MS;
    }
    ledBreathTick();
    delay(5);
  }
  ledBreathStop();

  if (count_nc > 0) {
    data.valid = true;
    data.sampleCount = count_nc;
    float N = (float)count_nc;
    data.nc0p5  = sum_nc0p5 / N;
    data.nc1p0  = sum_nc1p0 / N;
    data.nc2p5  = sum_nc2p5 / N;
    data.nc4p0  = sum_nc4p0 / N;
    data.nc10p0 = sum_nc10p0 / N;
    data.tps    = sum_tps / N;
  }

  return data;
}

// =================================================
// ======== HOME ASSISTANT DISCOVERY HELPERS =======

bool publishHADiscoverySensor(const char* object_id, const char* name, const char* state_topic, const char* unit, const char* device_class, const char* icon) {
  if (!mqttClient.connected()) return false;
  char topic[160];
  snprintf(topic, sizeof(topic), "%s/sensor/%s/%s/config", HA_DISCOVERY_PREFIX, mqtt_client_id, object_id);

  String payload = "{";
  payload += "\"name\":\"" + String(name) + "\",";
  payload += "\"state_topic\":\"" + String(state_topic) + "\",";
  if (unit && unit[0] != '\0') payload += "\"unit_of_measurement\":\"" + String(unit) + "\",";
  if (device_class && device_class[0] != '\0') payload += "\"device_class\":\"" + String(device_class) + "\",";
  if (icon && icon[0] != '\0') payload += "\"icon\":\"" + String(icon) + "\",";
  payload += "\"state_class\":\"measurement\",";
  payload += "\"unique_id\":\"" + String(mqtt_client_id) + "_" + String(object_id) + "\",";
  payload += "\"device\":{\"identifiers\":[\"" + String(mqtt_client_id) + "\"],\"name\":\"" + String(HA_DEVICE_NAME) + "\",\"model\":\"" + String(HA_DEVICE_MODEL) + "\",\"manufacturer\":\"" + String(HA_DEVICE_MFR) + "\"}";
  payload += "}";

  bool ok = mqttClient.publish(topic, payload, true, 1);
  unsigned long end = millis() + 50;
  while(millis() < end) { mqttClient.loop(); delay(2); }
  return ok;
}

void publishAllHADiscovery() {
  // Only advertise the MAIN averages to HA
  
  // --- PM Mass ---
  publishHADiscoverySensor("mc1p0",  "SPS30 PM1.0",  "sensor/sps30/mc1p0",  "µg/m³", "pm1",  nullptr);
  publishHADiscoverySensor("mc2p5",  "SPS30 PM2.5",  "sensor/sps30/mc2p5",  "µg/m³", "pm25", nullptr);
  publishHADiscoverySensor("mc4p0",  "SPS30 PM4.0",  "sensor/sps30/mc4p0",  "µg/m³", "pm4",  nullptr);
  publishHADiscoverySensor("mc10p0", "SPS30 PM10",   "sensor/sps30/mc10p0", "µg/m³", "pm10", nullptr);

  // --- Number concentrations ---
  publishHADiscoverySensor("nc0p5",  "SPS30 NC0.5",  "sensor/sps30/nc0p5",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc1p0",  "SPS30 NC1.0",  "sensor/sps30/nc1p0",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc2p5",  "SPS30 NC2.5",  "sensor/sps30/nc2p5",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc4p0",  "SPS30 NC4.0",  "sensor/sps30/nc4p0",  "#/cm³", "", "mdi:counter");
  publishHADiscoverySensor("nc10p0", "SPS30 NC10",   "sensor/sps30/nc10p0", "#/cm³", "", "mdi:counter");

  // --- Other ---
  publishHADiscoverySensor("tps",    "SPS30 Particle Size", "sensor/sps30/typical_particle_size", "µm", "", "mdi:circle-opacity");
}

// =================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  bootCount++; 
  Serial.println("\n=== ESP32-C3 SPS30 MQTT (Avg/Min/Max/Std/Med/Count) ===");
  Serial.printf("Boot count: %d\n", bootCount);

  pixels.begin();
  ledOff();

  SENSOR_SERIAL_INTERFACE.begin(115200, SERIAL_8N1, sensorRxPin, sensorTxPin);
  sensor.begin(SENSOR_SERIAL_INTERFACE);

  // Phase 1: Initialize/wake
  Serial.println("\n--- Phase 1: Initialize sensor ---");
  bool firstBoot = (bootCount == 1);
  if (firstBoot) {
    sensor.deviceReset();
    delay(200);
  } else {
    wakeSps30FromSleep();
  }

  // Optional Cleaning
  if (CLEANING_INTERVAL_BOOT > 0 && bootCount > 1 && ((bootCount - 1) % CLEANING_INTERVAL_BOOT == 0)) {
    Serial.println("Running SPS30 manual fan cleaning...");
    sensor.startFanCleaning();
    delay(10000);
  }

  // Phase 2: Start
  Serial.println("\n--- Phase 2: Starting measurement... ---");
  error = sensor.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
  if (error) {
    ledColor(LED_SENSOR_R, LED_SENSOR_G, LED_SENSOR_B);
    enter_deep_sleep();
  }

  // Warmup
  int waited = dynamicStartupWait();
  Serial.printf("Warmup done after ~%d seconds.\n", waited);

  // Measurement (Window)
  Sps30Data data = measureForWindow();

  // Phase 3: Stop + Sleep Sensor
  sensor.stopMeasurement();
  sensor.sleep();
  delay(30);

  // Phase 4: Publish
  bool wifiOK = connectWiFi();
  if (!wifiOK) {
    ledColor(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B);
    ledHold(5000); 
    enter_deep_sleep();
  }

  mqttClient.begin(mqtt_server, mqtt_port, net);
  mqttClient.setTimeout(5000);
  bool mqttOK = connectMQTT();

  if (mqttOK) {
    publishAllHADiscovery();
  }

  if (data.valid && mqttOK) {
    Serial.println("\nPublishing values...");
    bool ok = true;

    // Helper macro to publish group
    #define PUB_STATS(name, obj) \
      ok &= publishFloatBlocking("sensor/sps30/" name,          obj.mean); \
      ok &= publishFloatBlocking("sensor/sps30/" name "/min",   obj.minVal); \
      ok &= publishFloatBlocking("sensor/sps30/" name "/max",   obj.maxVal); \
      ok &= publishFloatBlocking("sensor/sps30/" name "/std",   obj.getStdDev()); \
      ok &= publishFloatBlocking("sensor/sps30/" name "/median",obj.getMedian())

    PUB_STATS("mc1p0",  data.mc1p0);
    PUB_STATS("mc2p5",  data.mc2p5);
    PUB_STATS("mc4p0",  data.mc4p0);
    PUB_STATS("mc10p0", data.mc10p0);

    // Number Concentrations
    ok &= publishFloatBlocking("sensor/sps30/nc0p5",  data.nc0p5);
    ok &= publishFloatBlocking("sensor/sps30/nc1p0",  data.nc1p0);
    ok &= publishFloatBlocking("sensor/sps30/nc2p5",  data.nc2p5);
    ok &= publishFloatBlocking("sensor/sps30/nc4p0",  data.nc4p0);
    ok &= publishFloatBlocking("sensor/sps30/nc10p0", data.nc10p0);
    
    // TPS
    ok &= publishFloatBlocking("sensor/sps30/typical_particle_size", data.tps);

    // Sample Count (MQTT only, no HA)
    ok &= publishFloatBlocking("sensor/sps30/samples", (float)data.sampleCount);

    if (!ok) {
      ledColor(LED_MQTT_R, LED_MQTT_G, LED_MQTT_B);
      ledHold(5000);
    }
  }

  if (mqttClient.connected()) mqttClient.disconnect();
  ledOff();
  Serial.printf("Deep sleep for %d s...\n", SLEEP_INTERVAL_SECONDS);
  enter_deep_sleep();
}

void loop() {}
