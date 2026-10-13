#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
// =====================================================

// 1. WIFI - DIEN THONG TIN CUA BAN
// =====================================================
const char *WIFI_SSID = "Redmi A3";
const char *WIFI_PASSWORD = "12345678";
// =====================================================
// 2. THINGSBOARD EDGE
// =====================================================
// IPv4 hien tai cua may tinh dang chay ThingsBoard Edge.
// ESP32 va may tinh phai o cung mang.
// Vi du: 10.254.173.61
const char *TB_HOST = "10.221.126.61";
// Docker Edge cua du an dang map host 11883 -> container 1883.
constexpr uint16_t TB_PORT = 11883;
// Dung DEVICE ACCESS TOKEN cua ESP32 Weather Station.
// Khong dung Edge routing key/secret.
const char *TB_ACCESS_TOKEN = "AcUQSdSNp1tpDXPD5vxd";
// =====================================================
// 3. GPIO
// =====================================================
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t UV_SENSOR_PIN = 34;
constexpr uint8_t WIND_SENSOR_PIN = 25;
constexpr uint8_t RAIN_SENSOR_PIN = 26;
// =====================================================
// 4. THONG SO CO KHI / HIEU CHUAN
// =====================================================
// ----- CAM BIEN GIO -----
// Rotor 3 coc: ban kinh tu truc den tam coc ~70 mm.
constexpr float WIND_RADIUS_M = 0.070F;
// Rotor cua ban co 1 nam cham => 1 xung / 1 vong.
constexpr float WIND_PULSES_PER_REVOLUTION = 1.0F;
// Day la he so hieu chuan thuc nghiem.
// Hien tai = 1.0 va offset = 0 => ket qua "toc do gio"
// dang la gia tri UOC TINH theo cong thuc hinh hoc cua tai lieu.
// Khi co may do gio chuan, thay 2 he so nay bang ket qua hoi quy.
constexpr float WIND_CAL_FACTOR = 1.0F;
constexpr float WIND_CAL_OFFSET_MPS = 0.0F;
// ----- CAM BIEN MUA -----
// Duong kinh mieng pheu do tu STL ~118.2 mm.
constexpr float RAIN_FUNNEL_DIAMETER_MM = 118.2F;
constexpr float RAIN_FUNNEL_RADIUS_MM = RAIN_FUNNEL_DIAMETER_MM / 2.0F;
constexpr float RAIN_FUNNEL_AREA_MM2 =
    PI * RAIN_FUNNEL_RADIUS_MM * RAIN_FUNNEL_RADIUS_MM;
// ----- HIEU CHUAN MUA THUC NGHIEM -----

constexpr float RAIN_CAL_VOLUME_ML = 250.0F;
constexpr float RAIN_CAL_TIPS_MIN = 52.0F;
constexpr float RAIN_CAL_TIPS_NOMINAL = 53.0F;
constexpr float RAIN_CAL_TIPS_MAX = 54.0F;
constexpr float RAIN_CAL_EQUIV_MM =
    (RAIN_CAL_VOLUME_ML * 1000.0F) / RAIN_FUNNEL_AREA_MM2;
constexpr float RAIN_MM_PER_TIP =
    RAIN_CAL_EQUIV_MM / RAIN_CAL_TIPS_NOMINAL;
// Bien thien cua he so tu cac lan thu 52...54 tip.
// Nhieu tip hon -> mm/tip nho hon.
constexpr float RAIN_MM_PER_TIP_MIN =
    RAIN_CAL_EQUIV_MM / RAIN_CAL_TIPS_MAX;   // ~0.4219
constexpr float RAIN_MM_PER_TIP_MAX =
    RAIN_CAL_EQUIV_MM / RAIN_CAL_TIPS_MIN;   // ~0.4381
constexpr float RAIN_CAL_REPEATABILITY_PCT =
    ((RAIN_MM_PER_TIP_MAX - RAIN_MM_PER_TIP_MIN) * 0.5F /
     RAIN_MM_PER_TIP) * 100.0F;              // ~1.9%
constexpr bool RAIN_CALIBRATED = true;
// =====================================================
// 5. CANH BAO NHIET DO + RPC
// =====================================================
// Gia tri mac dinh lan dau. Sau khi thay doi bang RPC,
// cau hinh duoc luu vao NVS nen van giu sau khi ESP32 khoi dong lai.
constexpr bool DEFAULT_TEMP_ALARM_ENABLED = true;
constexpr float DEFAULT_TEMP_ALARM_THRESHOLD_C = 35.0F;
// Gioi han hop le cho nguong nhiet do nhap tu Dashboard.
constexpr float TEMP_THRESHOLD_MIN_C = -40.0F;
constexpr float TEMP_THRESHOLD_MAX_C = 85.0F;
bool temperatureAlarmEnabled = DEFAULT_TEMP_ALARM_ENABLED;
float temperatureAlarmThresholdC = DEFAULT_TEMP_ALARM_THRESHOLD_C;
bool temperatureAlarmActive = false;
// NVS/Preferences cua ESP32.
Preferences preferences;
// =====================================================
// 6. THOI GIAN
// =====================================================
constexpr unsigned long TELEMETRY_INTERVAL_MS = 5000;
unsigned long lastTelemetryMs = 0;
// Chong nhieu xung Hall.
constexpr uint32_t WIND_DEBOUNCE_US = 10000;   // 10 ms
constexpr uint32_t RAIN_DEBOUNCE_US = 100000;  // 100 ms
// =====================================================
// 7. DOI TUONG CAM BIEN / MQTT
// =====================================================
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool aht20Available = false;
bool bmp280Available = false;
// =====================================================
// 8. BIEN CAM BIEN
// =====================================================
float temperatureC = NAN;
float humidityPct = NAN;
float pressureHpa = NAN;
int uvRaw = 0;
float uvVoltage = 0.0F;
float uvIndexEstimated = 0.0F;
float windHz = 0.0F;
float windRpm = 0.0F;
float windSpeedMpsEstimated = 0.0F;
float windSpeedKmhEstimated = 0.0F;
uint32_t windPulsesInterval = 0;
uint32_t rainTipsInterval = 0;
uint32_t rainTipsTotalSnapshot = 0;
float rainMmInterval = 0.0F;
float rainMmTotal = 0.0F;
// =====================================================
// 9. BIEN ISR
// =====================================================
volatile uint32_t windPulseCounter = 0;
volatile uint32_t rainTipCounterInterval = 0;
volatile uint32_t rainTipCounterTotal = 0;
volatile uint32_t lastWindPulseUs = 0;
volatile uint32_t lastRainPulseUs = 0;
// =====================================================
// 10. INTERRUPT - GIO
// =====================================================
void IRAM_ATTR onWindPulse() {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastWindPulseUs) >= WIND_DEBOUNCE_US) {
        windPulseCounter++;
        lastWindPulseUs = nowUs;
    }
}
// =====================================================
// 11. INTERRUPT - MUA
// =====================================================
void IRAM_ATTR onRainTip() {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastRainPulseUs) >= RAIN_DEBOUNCE_US) {
        rainTipCounterInterval++;
        rainTipCounterTotal++;
        lastRainPulseUs = nowUs;
    }
}
// =====================================================
// 12. WIFI
// =====================================================
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }
    Serial.println();
    Serial.print("Dang ket noi Wi-Fi: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Da ket noi Wi-Fi");
    Serial.print("IP ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.print("ThingsBoard Edge: ");
    Serial.print(TB_HOST);
    Serial.print(":");
    Serial.println(TB_PORT);
}
// =====================================================
// 13. THINGSBOARD ATTRIBUTES
// =====================================================
void publishAttributes() {
    JsonDocument doc;
    doc["firmware"] = "esp32-weather-rpc-alarm-1.1";
    doc["board"] = "ESP32";
    doc["wind_radius_m"] = WIND_RADIUS_M;
    doc["wind_pulses_per_revolution"] = WIND_PULSES_PER_REVOLUTION;
    doc["wind_cal_factor"] = WIND_CAL_FACTOR;
    doc["wind_cal_offset_mps"] = WIND_CAL_OFFSET_MPS;
    doc["rain_funnel_diameter_mm"] = RAIN_FUNNEL_DIAMETER_MM;
    doc["rain_funnel_area_mm2"] = RAIN_FUNNEL_AREA_MM2;
    doc["rain_calibrated"] = RAIN_CALIBRATED;
    doc["rain_cal_volume_ml"] = RAIN_CAL_VOLUME_ML;
    doc["rain_cal_tips_nominal"] = RAIN_CAL_TIPS_NOMINAL;
    doc["rain_mm_per_tip"] = RAIN_MM_PER_TIP;
    doc["rain_mm_per_tip_min"] = RAIN_MM_PER_TIP_MIN;
    doc["rain_mm_per_tip_max"] = RAIN_MM_PER_TIP_MAX;
    doc["rain_repeatability_pct"] = RAIN_CAL_REPEATABILITY_PCT;
    doc["temperature_alarm_enabled"] = temperatureAlarmEnabled;
    doc["temperature_alarm_threshold_c"] = temperatureAlarmThresholdC;
    char payload[512];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(
        "v1/devices/me/attributes",
        reinterpret_cast<uint8_t *>(payload),
        static_cast<unsigned int>(len),
        true
    );
}
// =====================================================
// 14. LUU / DOC CAU HINH CANH BAO TU NVS
// =====================================================
void loadAlarmConfig() {
    preferences.begin("weather", false);
    temperatureAlarmEnabled =
        preferences.getBool("tempAlarmEn", DEFAULT_TEMP_ALARM_ENABLED);
    temperatureAlarmThresholdC =
        preferences.getFloat("tempThC", DEFAULT_TEMP_ALARM_THRESHOLD_C);
    if (temperatureAlarmThresholdC < TEMP_THRESHOLD_MIN_C ||
        temperatureAlarmThresholdC > TEMP_THRESHOLD_MAX_C) {
        temperatureAlarmThresholdC = DEFAULT_TEMP_ALARM_THRESHOLD_C;
        preferences.putFloat("tempThC", temperatureAlarmThresholdC);
    }
    Serial.println();
    Serial.println("=== CAU HINH CANH BAO ===");
    Serial.print("Canh bao nhiet do     : ");
    Serial.println(temperatureAlarmEnabled ? "BAT" : "TAT");
    Serial.print("Nguong nhiet do       : ");
    Serial.print(temperatureAlarmThresholdC, 1);
    Serial.println(" C");
    Serial.println("=========================");
}
// =====================================================
// 15. GUI PHAN HOI RPC
// =====================================================
void sendRpcResponse(const String &requestId, JsonDocument &response) {
    String topic =
        String("v1/devices/me/rpc/response/") + requestId;
    char responsePayload[384];
    size_t responseLen =
        serializeJson(response, responsePayload, sizeof(responsePayload));
    bool ok = mqttClient.publish(
        topic.c_str(),
        reinterpret_cast<uint8_t *>(responsePayload),
        static_cast<unsigned int>(responseLen),
        false
    );
    Serial.print("RPC response -> ");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(ok ? "OK" : "FAILED");
}
bool parseBooleanRpcParam(JsonVariantConst params, bool &value) {
    if (params.is<bool>()) {
        value = params.as<bool>();
        return true;
    }
    if (params.is<int>() || params.is<long>()) {
        value = params.as<long>() != 0;
        return true;
    }
    if (params.is<const char *>()) {
        String text = params.as<const char *>();
        text.trim();
        text.toLowerCase();
        if (text == "true" || text == "on" || text == "1" ||
            text == "enable" || text == "enabled") {
            value = true;
            return true;
        }
        if (text == "false" || text == "off" || text == "0" ||
            text == "disable" || text == "disabled") {
            value = false;
            return true;
        }
    }
    if (params.is<JsonObjectConst>()) {
        JsonObjectConst obj = params.as<JsonObjectConst>();
        if (obj["enabled"].is<bool>()) {
            value = obj["enabled"].as<bool>();
            return true;
        }
        if (obj["value"].is<bool>()) {
            value = obj["value"].as<bool>();
            return true;
        }
    }
    return false;
}
bool parseThresholdRpcParam(JsonVariantConst params, float &value) {
    if (params.is<float>() || params.is<double>() ||
        params.is<int>() || params.is<long>()) {
        value = params.as<float>();
        return true;
    }
    if (params.is<const char *>()) {
        String text = params.as<const char *>();
        text.trim();
        if (text.length() > 0) {
            char *endPtr = nullptr;
            float parsed = strtof(text.c_str(), &endPtr);
            if (endPtr != text.c_str() && *endPtr == '\0') {
                value = parsed;
                return true;
            }
        }
    }
    if (params.is<JsonObjectConst>()) {
        JsonObjectConst obj = params.as<JsonObjectConst>();
        if (!obj["threshold"].isNull()) {
            value = obj["threshold"].as<float>();
            return true;
        }
        if (!obj["value"].isNull()) {
            value = obj["value"].as<float>();
            return true;
        }
    }
    return false;
}
// =====================================================
// 16. MQTT CALLBACK - NHAN RPC TU THINGSBOARD
// =====================================================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
    const char *rpcPrefix = "v1/devices/me/rpc/request/";
    if (strncmp(topic, rpcPrefix, strlen(rpcPrefix)) != 0) {
        return;
    }
    String requestId = String(topic + strlen(rpcPrefix));
    JsonDocument request;
    DeserializationError error =
        deserializeJson(request, payload, length);
    JsonDocument response;
    if (error) {
        response["success"] = false;
        response["error"] = "invalid_json";
        sendRpcResponse(requestId, response);
        return;
    }
    const char *method = request["method"] | "";
    JsonVariantConst params = request["params"];
    Serial.println();
    Serial.println("========== RPC REQUEST ==========");
    Serial.print("Request ID            : ");
    Serial.println(requestId);
    Serial.print("Method                : ");
    Serial.println(method);
    // -------------------------------------------------
    // RPC 1: BAT/TAT CANH BAO
    // -------------------------------------------------
    if (strcmp(method, "setAlarmEnabled") == 0) {
        bool newState = false;
        if (!parseBooleanRpcParam(params, newState)) {
            response["success"] = false;
            response["error"] = "invalid_boolean_param";
        } else {
            temperatureAlarmEnabled = newState;
            preferences.putBool("tempAlarmEn", temperatureAlarmEnabled);
            // Cap nhat ngay trang thai alarm theo nhiet do hien tai.
            temperatureAlarmActive =
                temperatureAlarmEnabled &&
                !isnan(temperatureC) &&
                temperatureC > temperatureAlarmThresholdC;
            response["success"] = true;
            response["alarm_enabled"] = temperatureAlarmEnabled;
            response["threshold_c"] = temperatureAlarmThresholdC;
            response["temperature_alarm"] = temperatureAlarmActive;
            Serial.print("Canh bao nhiet do     : ");
            Serial.println(temperatureAlarmEnabled ? "BAT" : "TAT");
            publishAttributes();
        }
    }
    // -------------------------------------------------
    // RPC 2: DOC TRANG THAI BAT/TAT
    // Dung cho Initial state cua Single Switch.
    // -------------------------------------------------
    else if (strcmp(method, "getAlarmEnabled") == 0) {
        // Tra ve JSON primitive true/false de Single Switch doc truc tiep.
        response.set(temperatureAlarmEnabled);
    }
    // -------------------------------------------------
    // RPC 3: DOI NGUONG NHIET DO
    // -------------------------------------------------
    else if (strcmp(method, "setTemperatureThreshold") == 0) {
        float newThreshold = NAN;
        if (!parseThresholdRpcParam(params, newThreshold) ||
            isnan(newThreshold) ||
            newThreshold < TEMP_THRESHOLD_MIN_C ||
            newThreshold > TEMP_THRESHOLD_MAX_C) {
            response["success"] = false;
            response["error"] = "threshold_out_of_range";
            response["min_c"] = TEMP_THRESHOLD_MIN_C;
            response["max_c"] = TEMP_THRESHOLD_MAX_C;
        } else {
            temperatureAlarmThresholdC = newThreshold;
            preferences.putFloat("tempThC", temperatureAlarmThresholdC);
            temperatureAlarmActive =
                temperatureAlarmEnabled &&
                !isnan(temperatureC) &&
                temperatureC > temperatureAlarmThresholdC;
            response["success"] = true;
            response["threshold_c"] = temperatureAlarmThresholdC;
            response["alarm_enabled"] = temperatureAlarmEnabled;
            response["temperature_alarm"] = temperatureAlarmActive;
            Serial.print("Nguong nhiet do moi   : ");
            Serial.print(temperatureAlarmThresholdC, 1);
            Serial.println(" C");
            publishAttributes();
        }
    }
    // -------------------------------------------------
    // RPC 4: DOC NGUONG HIEN TAI
    // Dung cho Initial state cua Knob Control.
    // -------------------------------------------------
    else if (strcmp(method, "getTemperatureThreshold") == 0) {
        // Tra ve JSON primitive number de Knob/Slider doc truc tiep.
        response.set(temperatureAlarmThresholdC);
    }
    // -------------------------------------------------
    // RPC 5: DOC TOAN BO CAU HINH - DUNG DE DEBUG
    // -------------------------------------------------
    else if (strcmp(method, "getAlarmConfig") == 0) {
        response["success"] = true;
        response["alarm_enabled"] = temperatureAlarmEnabled;
        response["threshold_c"] = temperatureAlarmThresholdC;
        if (isnan(temperatureC)) {
            response["temperature_c"] = nullptr;
        } else {
            response["temperature_c"] = temperatureC;
        }
        response["temperature_alarm"] = temperatureAlarmActive;
    }
    else {
        response["success"] = false;
        response["error"] = "unknown_method";
        response["method"] = method;
    }
    sendRpcResponse(requestId, response);
    Serial.println("=================================");
}
// =====================================================
// 17. MQTT THINGSBOARD EDGE
// =====================================================
void connectThingsBoard() {
    if (mqttClient.connected()) {
        return;
    }
    while (!mqttClient.connected()) {
        Serial.print("Dang ket noi ThingsBoard Edge...");
        uint64_t chipId = ESP.getEfuseMac();
        char clientId[48];
        snprintf(
            clientId,
            sizeof(clientId),
            "ESP32-Weather-%04X%08X",
            static_cast<uint16_t>(chipId >> 32),
            static_cast<uint32_t>(chipId)
        );
        bool connected = mqttClient.connect(
            clientId,
            TB_ACCESS_TOKEN,
            ""
        );
        if (connected) {
            Serial.println(" thanh cong");
            mqttClient.subscribe(
                "v1/devices/me/rpc/request/+"
            );
            Serial.println(
                "Da subscribe RPC: v1/devices/me/rpc/request/+"
            );
            publishAttributes();
        } else {
            Serial.print(" that bai, MQTT state = ");
            Serial.println(mqttClient.state());
            delay(3000);
        }
    }
}
// =====================================================
// 18. DOC UV GUVA-S12SD - MODULE DO ANALOG 3 CHAN
// 
constexpr int UV_SAMPLE_COUNT = 100;
constexpr float UV_MV_PER_INDEX = 100.0F;
constexpr float UV_DARK_OFFSET_MV = 100.0F;
constexpr float UV_DEADBAND_MV = 20.0F;
constexpr float UV_INDEX_LIMIT = 11.0F;
float calculateUvIndex(float mv) {
    float correctedMv = mv - UV_DARK_OFFSET_MV;
    if (correctedMv <= UV_DEADBAND_MV) {
        return 0.0F;
    }
    correctedMv -= UV_DEADBAND_MV;
    float uvi = correctedMv / UV_MV_PER_INDEX;
    if (uvi > UV_INDEX_LIMIT) {
        uvi = UV_INDEX_LIMIT;
    }
    return uvi;
}
void readUvSensor() {
    uint32_t rawSum = 0;
    uint32_t mvSum = 0;
    for (int i = 0; i < UV_SAMPLE_COUNT; i++) {
        rawSum += analogRead(UV_SENSOR_PIN);
        mvSum += analogReadMilliVolts(UV_SENSOR_PIN);
        delay(2);
    }
    float rawAvg = rawSum / (float)UV_SAMPLE_COUNT;
    float uvMv = mvSum / (float)UV_SAMPLE_COUNT;
    uvRaw = (int)roundf(rawAvg);
    uvVoltage = uvMv / 1000.0F;
    uvIndexEstimated = calculateUvIndex(uvMv);
    Serial.print("UV RAW = ");
    Serial.print(rawAvg, 1);
    Serial.print(" | UV mV = ");
    Serial.print(uvMv, 1);
    Serial.print(" | UV V = ");
    Serial.print(uvVoltage, 4);
    Serial.print(" | UV Index = ");
    Serial.println(uvIndexEstimated, 2);
}
// =====================================================
// 19. DOC AHT20 + BMP280
// =====================================================
void readEnvironmentalSensors() {
    if (aht20Available) {
        sensors_event_t humidityEvent;
        sensors_event_t temperatureEvent;
        aht20.getEvent(&humidityEvent, &temperatureEvent);
        temperatureC = temperatureEvent.temperature;
        humidityPct = humidityEvent.relative_humidity;
    }
    if (bmp280Available) {
        pressureHpa = bmp280.readPressure() / 100.0F;
    }
    readUvSensor();
    temperatureAlarmActive =
        temperatureAlarmEnabled &&
        !isnan(temperatureC) &&
        temperatureC > temperatureAlarmThresholdC;
}
// =====================================================
// 20. SNAPSHOT VA TINH GIO + MUA
// =====================================================
void calculateWindAndRain(float elapsedSeconds) {
    noInterrupts();
    windPulsesInterval = windPulseCounter;
    windPulseCounter = 0;
    rainTipsInterval = rainTipCounterInterval;
    rainTipCounterInterval = 0;
    rainTipsTotalSnapshot = rainTipCounterTotal;
    interrupts();
    if (elapsedSeconds <= 0.0F) {
        elapsedSeconds = TELEMETRY_INTERVAL_MS / 1000.0F;
    }
    // ----- GIO -----
    windHz =
        static_cast<float>(windPulsesInterval) / elapsedSeconds;
    float revolutionsPerSecond =
        windHz / WIND_PULSES_PER_REVOLUTION;
    windRpm = revolutionsPerSecond * 60.0F;
    // Theo tai lieu: v = so vong * chu vi / thoi gian.
    const float circumferenceM =
        2.0F * PI * WIND_RADIUS_M;
    float cupLinearSpeedMps =
        revolutionsPerSecond * circumferenceM;
    // He so hieu chuan thuc nghiem.
    windSpeedMpsEstimated =
        WIND_CAL_FACTOR * cupLinearSpeedMps +
        WIND_CAL_OFFSET_MPS;
    if (windSpeedMpsEstimated < 0.0F) {
        windSpeedMpsEstimated = 0.0F;
    }
    windSpeedKmhEstimated =
        windSpeedMpsEstimated * 3.6F;
    // ----- MUA -----
    if (RAIN_CALIBRATED) {
        rainMmInterval =
            static_cast<float>(rainTipsInterval) *
            RAIN_MM_PER_TIP;
        rainMmTotal =
            static_cast<float>(rainTipsTotalSnapshot) *
            RAIN_MM_PER_TIP;
    } else {
        rainMmInterval = 0.0F;
        rainMmTotal = 0.0F;
    }
}
// =====================================================
// 21. GUI TELEMETRY LEN THINGSBOARD
// =====================================================
void publishTelemetry() {
    JsonDocument telemetry;
    // =================================================
    // CHI GUI CAC TELEMETRY CAN THIET LEN THINGSBOARD
    // =================================================
    // 1. Nhiet do - dung cho Dashboard va Alarm
    if (!isnan(temperatureC)) {
        telemetry["temperature"] = roundf(temperatureC * 100.0F) / 100.0F;
    }
    // 2. Do am
    if (!isnan(humidityPct)) {
        telemetry["humidity"] = humidityPct;
    }
    // 3. Ap suat - chi giu don vi hPa
    if (!isnan(pressureHpa)) {
        telemetry["pressure_hpa"] = pressureHpa;
    }
    // 4. Chi so UV uoc tinh
    telemetry["uv_index_estimated"] = uvIndexEstimated;
    // 5. Toc do gio - chi giu m/s
    telemetry["wind_speed_mps_estimated"] =
        windSpeedMpsEstimated;
    // 6. Luong mua trong chu ky gui du lieu
    // ThingsBoard co the SUM theo tung gio de tao bieu do mua theo gio.
    telemetry["rain_mm_interval"] = rainMmInterval;
    // 7. Tong luong mua ke tu luc ESP32 khoi dong
    telemetry["rain_mm_total"] = rainMmTotal;
    // 8. Trang thai canh bao nhiet do do ESP32 tinh
    telemetry["temperature_alarm"] = temperatureAlarmActive;
    /*
     * CAC GIA TRI KHONG CON GUI LEN THINGSBOARD:
     *
     * pressure_pa
     * pressure_kpa
     * alarm_enabled
     * temperature_threshold_c
     * uv_raw
     * uv_voltage
     * wind_hall_state
     * wind_pulses_interval
     * wind_hz
     * wind_rpm
     * wind_speed_kmh_estimated
     * rain_hall_state
     * rain_tips_interval
     * rain_tips_total
     * rain_calibrated
     * rain_mm_per_tip
     * wifi_rssi
     * sensor_ok
     *
     * Luu y:
     * - Trang thai bat/tat canh bao va nguong nhiet do da duoc
     *   gui bang CLIENT ATTRIBUTES trong publishAttributes().
     * - Cac thong so ky thuat van duoc tinh va hien tren Serial
     *   de kiem tra, nhung khong luu vao telemetry ThingsBoard.
     */
    char payload[512];
    size_t len = serializeJson(
        telemetry,
        payload,
        sizeof(payload)
    );
    bool published = mqttClient.publish(
        "v1/devices/me/telemetry",
        reinterpret_cast<uint8_t *>(payload),
        static_cast<unsigned int>(len),
        false
    );
    // =================================================
    // SERIAL MONITOR - GIU DAY DU THONG SO DE DEBUG
    // =================================================
    Serial.println();
    Serial.println("========== WEATHER DATA ==========");
    Serial.print("Nhiet do              : ");
    Serial.print(temperatureC, 1);
    Serial.println(" C");
    Serial.print("Canh bao nhiet do     : ");
    Serial.println(temperatureAlarmEnabled ? "BAT" : "TAT");
    Serial.print("Nguong canh bao       : ");
    Serial.print(temperatureAlarmThresholdC, 1);
    Serial.println(" C");
    Serial.print("Trang thai canh bao   : ");
    Serial.println(
        temperatureAlarmActive ? "ALARM" : "BINH THUONG"
    );
    Serial.print("Do am                 : ");
    Serial.print(humidityPct, 1);
    Serial.println(" %");
    Serial.print("Ap suat               : ");
    Serial.print(pressureHpa, 1);
    Serial.println(" hPa");
    Serial.print("UV Voltage            : ");
    Serial.print(uvVoltage, 3);
    Serial.println(" V");
    Serial.print("UV Index uoc tinh     : ");
    Serial.println(uvIndexEstimated, 2);
    Serial.print("Xung gio / chu ky     : ");
    Serial.println(windPulsesInterval);
    Serial.print("Tan so gio            : ");
    Serial.print(windHz, 2);
    Serial.println(" Hz");
    Serial.print("Rotor                 : ");
    Serial.print(windRpm, 1);
    Serial.println(" RPM");
    Serial.print("Toc do gio uoc tinh   : ");
    Serial.print(windSpeedMpsEstimated, 3);
    Serial.println(" m/s");
    Serial.print("                       ");
    Serial.print(windSpeedKmhEstimated, 2);
    Serial.println(" km/h");
    Serial.print("Gau lat / chu ky      : ");
    Serial.println(rainTipsInterval);
    Serial.print("Tong so lan gau lat   : ");
    Serial.println(rainTipsTotalSnapshot);
    if (RAIN_CALIBRATED) {
        Serial.print("He so mua             : ");
        Serial.print(RAIN_MM_PER_TIP, 4);
        Serial.println(" mm/tip");
        Serial.print("Luong mua chu ky      : ");
        Serial.print(rainMmInterval, 3);
        Serial.println(" mm");
        Serial.print("Tong luong mua        : ");
        Serial.print(rainMmTotal, 3);
        Serial.println(" mm");
    } else {
        Serial.println(
            "Luong mua (mm)        : CHUA HIEU CHUAN"
        );
    }
    Serial.print("Gui ThingsBoard Edge  : ");
    Serial.println(published ? "Thanh cong" : "That bai");
    Serial.println("==================================");
}
// =====================================================
// 22. SETUP
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println();
    Serial.println("=== ESP32 WEATHER STATION FINAL ===");
    // I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    // AHT20
    aht20Available = aht20.begin();
    Serial.println(aht20Available ? "AHT20: OK" : "AHT20: NOT FOUND");
    // BMP280 - module cua ban tung quet duoc 0x77.
    bmp280Available = bmp280.begin(0x77);
    if (!bmp280Available) {
        bmp280Available = bmp280.begin(0x76);
    }
    Serial.println(bmp280Available ? "BMP280: OK" : "BMP280: NOT FOUND");
    // UV GUVA-S12SD module do: OUT thuong nam trong khoang 0...1V.
    // ADC_2_5db phu hop hon ADC_11db cho dai dien ap nay.
    pinMode(UV_SENSOR_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(UV_SENSOR_PIN, ADC_2_5db);
    // KY-003 co dien tro pull-up NGOAI 1k len 3V3.
    pinMode(WIND_SENSOR_PIN, INPUT);
    pinMode(RAIN_SENSOR_PIN, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(WIND_SENSOR_PIN),
        onWindPulse,
        FALLING
    );
    attachInterrupt(
        digitalPinToInterrupt(RAIN_SENSOR_PIN),
        onRainTip,
        FALLING
    );
    // Doc cau hinh canh bao da luu trong NVS.
    loadAlarmConfig();
    connectWiFi();
    mqttClient.setServer(TB_HOST, TB_PORT);
    mqttClient.setBufferSize(1536);
    mqttClient.setCallback(mqttCallback);
    connectThingsBoard();
    lastTelemetryMs = millis();
}
// =====================================================
// 23. LOOP
// =====================================================
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }
    if (!mqttClient.connected()) {
        connectThingsBoard();
    }
    mqttClient.loop();
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        float elapsedSeconds =
            (nowMs - lastTelemetryMs) / 1000.0F;
        lastTelemetryMs = nowMs;
        readEnvironmentalSensors();
        calculateWindAndRain(elapsedSeconds);
        publishTelemetry();
    }
}
