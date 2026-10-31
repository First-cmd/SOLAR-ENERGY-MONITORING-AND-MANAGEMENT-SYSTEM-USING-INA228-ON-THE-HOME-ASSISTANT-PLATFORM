/*
 * ============================================================================
 *  ESP32 + INA228 + Relay x4 -> Home Assistant ผ่าน MQTT Auto-Discovery
 * ============================================================================
 *  อ่านค่ากระแส/แรงดัน/กำลังไฟ/พลังงานสะสม แล้วส่งเข้า Home Assistant
 *  Relay 1 ตัดต่อระหว่างแบตเตอรี่กับคอนโทรลชาร์จเจอร์
 *  Relay 2 ควบคุมโหลดพลังงานส่วนเกิน พร้อมระบบป้องกันแรงดันต่ำ
 *  พร้อม hysteresis, delay, ปุ่มเปิด/ปิดโหมด และปุ่มสั่ง Relay 2 ด้วยมือ
 *  คำสั่งมือ "ไม่สามารถข้ามเงื่อนไขความปลอดภัย" ได้
 *  + ระบบพยากรณ์แบตเตอรี่ (SoC แบบ coulomb counting, เวลาจนเต็ม) -- แสดงผลเท่านั้น
 *
 *  แก้ SECTION 1 (WiFi/MQTT) ให้ตรงกับของคุณก่อนอัปโหลด
 * ============================================================================
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Preferences.h>   // เก็บค่า SoC ไว้ใน flash กันหายตอนรีบูต
// ============================================================================
// SECTION 1: WIFI / MQTT CONFIG -- แก้ตรงนี้ให้ตรงกับของคุณ
// ============================================================================
#define WIFI_SSID       "Home_Server/2.4G"
#define WIFI_PASSWORD   "homeserverwifi"
#define MQTT_SERVER     "192.168.10.20"   // IP ของ MQTT broker (เช่น Home Assistant)
#define MQTT_PORT       1883
#define MQTT_USER       "mqtt"                // เว้นว่างถ้าไม่ auth
#define MQTT_PASSWORD   "espmqtt"                // เว้นว่างถ้าไม่ auth
#define MQTT_CLIENT_ID  "esp32_solar_monitor"
// ============================================================================
// SECTION 2: I2C / INA228 CONFIG
// ============================================================================
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22
#define INA228_ADDR        0x40   // High-Side, addr A0=GND, A1=GND
#define SHUNT_OHMS         0.00022f   // ปรับจาก 0.002 ตามอัตราส่วนที่วัดได้จริง
#define MAX_CURRENT_A      5.0f     // 5A margin (บอร์ดรองรับสูงสุด ~10A)
#define REG_CONFIG        0x00
#define REG_ADC_CONFIG    0x01
#define REG_SHUNT_CAL     0x02
#define REG_VBUS          0x05
#define REG_CURRENT       0x07
#define REG_POWER         0x08
#define REG_ENERGY        0x09
#define VBUS_LSB_V         0.0001953125f
#define VBUS_CALIBRATION_OFFSET  0.5f   // ชดเชยแรงดันที่อ่านต่ำกว่าจริง (เช่น จาก diode drop ใน PWM controller)
float currentLSB, powerLSB, energyLSB;
#define SAMPLE_INTERVAL_MS 5000   // อ่านค่าและส่ง MQTT ทุกๆ 5 วินาที
// ============================================================================
// SECTION 3: RELAY PINS & SAFETY THRESHOLDS
// ============================================================================
#define RELAY1_PIN            27   // ตัดต่อระหว่างแบตเตอรี่กับคอนโทรลชาร์จเจอร์
#define RELAY2_PIN            26   // โหลดพลังงานส่วนเกิน (ควบคุมอัตโนมัติ)
#define RELAY3_PIN            25   // ยังไม่พร้อมใช้งาน จึงสั่งปิดไว้
#define RELAY4_PIN            33   // รีเลย์เสริม 4 (ควบคุมจาก Home Assistant)
#define RELAY_ON               LOW    // โมดูลรีเลย์แบบ Active-LOW: LOW = ON
#define RELAY_OFF               HIGH  // HIGH = OFF
#define OVERVOLTAGE_CUTOFF          15.0f   // สูงต่อเนื่อง 5 วินาที -> แยกแบตออกจากคอนโทรลชาร์จเจอร์
#define OVERVOLTAGE_RECONNECT       12.8f   // ต่ำกว่าค่านี้และครบเวลาพัก -> ต่อกลับ
#define OVERVOLTAGE_DELAY_MS         5000UL
#define CHARGE_MIN_OFF_TIME_MS     600000UL // พักอย่างน้อย 10 นาที
#define SURPLUS_LOAD_ON_V            14.2f   // สูงต่อเนื่อง 3 นาที -> เปิดโหลดส่วนเกิน
#define SURPLUS_LOAD_OFF_V           12.7f   // ต่ำต่อเนื่อง 30 วินาที -> ปิดโหลด
#define SURPLUS_LOAD_ON_DELAY_MS   180000UL
#define SURPLUS_LOAD_OFF_DELAY_MS   30000UL
#define UNDERVOLTAGE_CUTOFF    11.5f   // แรงดันแบตต่ำเกิน -> ตัดโหลด
#define UNDERVOLTAGE_RECONNECT 12.5f   // แรงดันฟื้นตัวสูงกว่านี้ -> ต่อโหลดกลับ

bool chargeSafetyOK = true;   // true = ปลอดภัย ไม่ต้องตัดชาร์จ
bool loadSafetyOK   = true;   // true = ปลอดภัย ไม่ต้องตัดโหลด
// คำสั่ง Relay 1 จาก Home Assistant: true = สั่งปิดค้างไว้
bool manualRelay1Off = false;
bool relay1State = true;
bool relay2State = false; // โหลดส่วนเกินเริ่มต้นปิด
bool relay4State = false; // เริ่มต้นปิด
bool manualRelay2On = false; // คำขอเปิด Relay 2 ด้วยมือ (ใช้เมื่อโหมดอัตโนมัติปิด)
// โหมดจัดการพลังงานส่วนเกินเริ่มต้นเป็น OFF ทุกครั้งหลังบูต
bool surplusModeEnabled = false;
unsigned long overVoltageStartMs = 0;
unsigned long chargeCutoffAtMs = 0;
unsigned long surplusHighStartMs = 0;
unsigned long surplusLowStartMs = 0;
// ============================================================================
// SECTION 3B: BATTERY FORECAST -- ใช้แสดงผลเท่านั้น ไม่ถูกใช้ตัดสินใจควบคุมรีเลย์
// ============================================================================
#define BATTERY_CAPACITY_AH      7.0f    // ความจุแบต 12V 5.5Ah (ลดค่านี้ลงเมื่อแบตเสื่อม)
#define CHARGE_EFFICIENCY        0.90f   // ประสิทธิภาพการชาร์จตะกั่วกรด (Ah ที่ไหลเข้า ไม่ได้เก็บทั้งหมด)
#define BATTERY_INTERNAL_R_OHM   0.08f   // ความต้านทานภายในโดยประมาณ ใช้ชดเชยแรงดันขณะชาร์จตอนเริ่มต้นครั้งแรก
#define FULL_DETECT_V            13.2f   // แรงดันขั้นต่ำที่ถือว่าเข้าใกล้เต็ม
#define FULL_DETECT_I_A          0.25f   // กระแสชาร์จต่ำกว่านี้ (~C/20) ขณะแรงดันสูง = เต็ม
#define FULL_DETECT_TIME_MS      120000UL // ต้องเข้าเงื่อนไขต่อเนื่อง 2 นาทีก่อนรีเซ็ต SoC = 100%
#define SOC_AT_LOAD_CUTOFF_PCT   5.0f    // ตอนระบบตัดโหลดแรงดันต่ำ ให้ SoC ไม่เกินค่านี้
#define FORECAST_CURRENT_ALPHA   0.05f   // ค่าเฉลี่ยกระแส (EMA) ต่อรอบ 5 วินาที ~ หน่วงราว 100 วินาที
#define FORECAST_MIN_CURRENT_A   0.05f   // กระแสเฉลี่ยต่ำกว่านี้ถือว่านิ่ง ไม่พยากรณ์เวลา
#define FORECAST_MAX_MIN         5940    // เกิน 99 ชั่วโมงถือว่าไม่ระบุ (unknown)
#define SOC_SAVE_INTERVAL_MS     600000UL // บันทึก SoC ลง flash ไม่ถี่กว่าทุก 10 นาที
Preferences prefs;
float socAh = 0.0f;                  // ประจุที่เหลือในแบต (Ah)
float netCurrentNowA = 0.0f;         // กระแสชาร์จล่าสุดจาก INA228
float avgEffCurrentA = 0.0f;         // กระแสชาร์จเฉลี่ย (หักประสิทธิภาพชาร์จแล้ว)
bool forecastInit = false;
unsigned long lastForecastMs = 0;
unsigned long fullStartMs = 0;
unsigned long lastSocSaveMs = 0;
float lastSavedSocAh = -1.0f;
int forecastToFullMin = -1;          // -1 = ไม่ระบุ
const char* forecastStatus = "กำลังเริ่มต้น";
// ============================================================================
// SECTION 4: MQTT TOPICS (state topics ที่ HA จะไปอ่านค่า)
// ============================================================================
const char* TOPIC_CURRENT = "solar_monitor/sensor/current";
const char* TOPIC_VOLTAGE = "solar_monitor/sensor/voltage";
const char* TOPIC_BATTERY_PERCENT = "solar_monitor/sensor/battery_percent";
const char* TOPIC_POWER   = "solar_monitor/sensor/power";
const char* TOPIC_ENERGY  = "solar_monitor/sensor/energy";
const char* TOPIC_AVAILABILITY = "solar_monitor/status";  // online/offline
const char* TOPIC_RELAY1_STATE = "solar_monitor/relay1/state";  // publish
const char* TOPIC_RELAY1_SET   = "solar_monitor/relay1/set";    // subscribe
const char* TOPIC_RELAY2_STATE = "solar_monitor/relay2/state";
const char* TOPIC_RELAY2_MANUAL_STATE = "solar_monitor/relay2_manual/state";
const char* TOPIC_RELAY2_MANUAL_SET   = "solar_monitor/relay2_manual/set";
const char* TOPIC_RELAY4_STATE = "solar_monitor/relay4/state";
const char* TOPIC_RELAY4_SET   = "solar_monitor/relay4/set";
const char* TOPIC_SURPLUS_MODE_STATE = "solar_monitor/surplus_mode/state";
const char* TOPIC_SURPLUS_MODE_SET   = "solar_monitor/surplus_mode/set";
const char* TOPIC_SOC              = "solar_monitor/sensor/soc";
const char* TOPIC_TIME_TO_FULL     = "solar_monitor/sensor/time_to_full";
const char* TOPIC_FORECAST_STATUS  = "solar_monitor/sensor/forecast_status";
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastSampleMs = 0;
// ============================================================================
// LOW LEVEL I2C HELPERS
// ============================================================================
void ina228_writeReg16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}
uint64_t ina228_readRegN(uint8_t reg, uint8_t nBytes) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)INA228_ADDR, (int)nBytes);
  uint64_t result = 0;
  for (uint8_t i = 0; i < nBytes; i++) {
    if (Wire.available()) result = (result << 8) | Wire.read();
  }
  return result;
}
int32_t signExtend(uint32_t value, uint8_t bits) {
  uint32_t signBit = 1UL << (bits - 1);
  return (int32_t)((value ^ signBit) - signBit);
}
// ============================================================================
// INA228 SETUP & READ
// ============================================================================
void ina228_configure() {
  currentLSB = MAX_CURRENT_A / 524288.0f;  // 2^19
  powerLSB   = 3.2f * currentLSB;
  energyLSB  = 16.0f * powerLSB;
  float shuntCal = 13107.2e6f * currentLSB * SHUNT_OHMS;
  uint16_t shuntCalReg = (uint16_t)(shuntCal + 0.5f);
  ina228_writeReg16(REG_CONFIG, 0x0000);
  ina228_writeReg16(REG_ADC_CONFIG, 0xFB69);  // continuous bus+shunt+temp
  ina228_writeReg16(REG_SHUNT_CAL, shuntCalReg);
  Serial.printf("SHUNT_CAL = %u, CURRENT_LSB = %.8f A/bit\n", shuntCalReg, currentLSB);
}
float ina228_readBusVoltage() {
  uint64_t raw = ina228_readRegN(REG_VBUS, 3) >> 4;
  return (float)raw * VBUS_LSB_V + VBUS_CALIBRATION_OFFSET;
}
float ina228_readCurrent() {
  uint64_t raw = ina228_readRegN(REG_CURRENT, 3) >> 4;
  int32_t signedVal = signExtend((uint32_t)raw, 20);
  return (float)signedVal * currentLSB;
}
float ina228_readPower() {
  uint64_t raw = ina228_readRegN(REG_POWER, 3);
  return (float)raw * powerLSB;
}
float ina228_readEnergy() {
  uint64_t raw = ina228_readRegN(REG_ENERGY, 5);
  return (float)raw * energyLSB;
}
// แบต UPS 12V แบบตะกั่วกรด: เปอร์เซ็นต์โดยประมาณจากแรงดันขณะพัก
// ช่วงนี้ใช้เพื่อแสดงผลเท่านั้น ไม่ใช้ตัดสินใจควบคุมรีเลย์
int estimateBatteryPercent(float voltage) {
  const float emptyV = 11.8f;
  const float fullV = 12.7f;
  float percent = (voltage - emptyV) * 100.0f / (fullV - emptyV);
  return (int)roundf(constrain(percent, 0.0f, 100.0f));
}
// ============================================================================
// BATTERY FORECAST FUNCTIONS
// ============================================================================
// ไม่มีเซนเซอร์วัดกระแสโหลด จึงรู้เฉพาะกระแสชาร์จ:
//  - ขณะชาร์จ: นับ Ah ที่เข้าแบต แล้วพยากรณ์เวลาจนเต็ม
//  - ขณะไม่ชาร์จ (กลางคืน/ฝน): SoC ตามแรงดันแบต (ประมาณการ) และไม่พยากรณ์เวลา
void updateBatteryForecast(float battV, float chargeI) {
  unsigned long now = millis();
  if (chargeI < 0.0f) chargeI = 0.0f;
  netCurrentNowA = chargeI;
  float effI = chargeI * CHARGE_EFFICIENCY;

  if (!forecastInit) {
    float stored = prefs.getFloat("soc_ah", -1.0f);
    if (stored >= 0.0f && stored <= BATTERY_CAPACITY_AH) {
      socAh = stored;                                   // ใช้ค่าที่บันทึกไว้ก่อนรีบูต
    } else {
      float vRest = battV - chargeI * BATTERY_INTERNAL_R_OHM;  // ประมาณแรงดันขณะพัก
      socAh = BATTERY_CAPACITY_AH * estimateBatteryPercent(vRest) / 100.0f;
    }
    avgEffCurrentA = effI;
    lastForecastMs = now;
    lastSavedSocAh = socAh;
    forecastInit = true;
  }

  float dtH = (float)(now - lastForecastMs) / 3600000.0f;
  lastForecastMs = now;
  avgEffCurrentA += FORECAST_CURRENT_ALPHA * (effI - avgEffCurrentA);
  bool charging = avgEffCurrentA > FORECAST_MIN_CURRENT_A;

  if (charging) {
    socAh = constrain(socAh + effI * dtH, 0.0f, BATTERY_CAPACITY_AH);   // integrate กระแสเป็น Ah
  } else {
    // ไม่ชาร์จ: ปรับ SoC เข้าหาค่าจากแรงดันช้าๆ (ไม่รู้กระแสโหลด)
    float socV = BATTERY_CAPACITY_AH * estimateBatteryPercent(battV) / 100.0f;
    socAh += 0.1f * (socV - socAh);
    socAh = constrain(socAh, 0.0f, BATTERY_CAPACITY_AH);
  }

  // re-anchor: เต็ม (แรงดันสูง + กระแสชาร์จต่ำต่อเนื่อง หรือเพิ่งตัดชาร์จเพราะแรงดันเกิน)
  bool looksFull = (relay1State && battV >= FULL_DETECT_V && chargeI < FULL_DETECT_I_A) || !chargeSafetyOK;
  if (looksFull) {
    if (fullStartMs == 0) fullStartMs = now;
    if (now - fullStartMs >= FULL_DETECT_TIME_MS) socAh = BATTERY_CAPACITY_AH;
  } else {
    fullStartMs = 0;
  }
  // re-anchor: ระบบตัดโหลดแรงดันต่ำทำงาน = แบตเกือบหมด
  if (!loadSafetyOK) {
    socAh = fminf(socAh, BATTERY_CAPACITY_AH * SOC_AT_LOAD_CUTOFF_PCT / 100.0f);
  }

  // พยากรณ์เวลาจนเต็ม
  forecastToFullMin = -1;
  bool isFull = socAh >= BATTERY_CAPACITY_AH - 0.01f;
  if (isFull) {
    forecastStatus = "เต็ม";
    forecastToFullMin = 0;
  } else if (charging) {
    forecastStatus = "กำลังชาร์จ";
    float mins = (BATTERY_CAPACITY_AH - socAh) / avgEffCurrentA * 60.0f;
    if (mins <= FORECAST_MAX_MIN) forecastToFullMin = (int)(mins + 0.5f);
  } else {
    forecastStatus = "ไม่ได้ชาร์จ";
  }

  // บันทึก SoC ลง flash (จำกัดความถี่ + เฉพาะเมื่อเปลี่ยนเกิน ~1%)
  if (now - lastSocSaveMs >= SOC_SAVE_INTERVAL_MS && fabsf(socAh - lastSavedSocAh) >= 0.05f) {
    prefs.putFloat("soc_ah", socAh);
    lastSavedSocAh = socAh;
    lastSocSaveMs = now;
  }
}
void publishBatteryForecast() {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", socAh / BATTERY_CAPACITY_AH * 100.0f);
  mqttClient.publish(TOPIC_SOC, buf);
  snprintf(buf, sizeof(buf), "%d", forecastToFullMin);   mqttClient.publish(TOPIC_TIME_TO_FULL, buf);
  mqttClient.publish(TOPIC_FORECAST_STATUS, forecastStatus);
  Serial.printf("FORECAST: SoC=%.0f%%  ToFull=%dmin  [%s]\n",
                socAh / BATTERY_CAPACITY_AH * 100.0f, forecastToFullMin, forecastStatus);
}
// ============================================================================
// RELAY CONTROL
// ============================================================================
void applyRelayStates() {
  bool newRelay1 = chargeSafetyOK && !manualRelay1Off;
  if (newRelay1 != relay1State) {
    relay1State = newRelay1;
    digitalWrite(RELAY1_PIN, relay1State ? RELAY_ON : RELAY_OFF);
    Serial.printf("[RELAY1] %s (safetyOK=%d, manualOff=%d)\n",
                  relay1State ? "ON" : "OFF", chargeSafetyOK, manualRelay1Off);
    mqttClient.publish(TOPIC_RELAY1_STATE, relay1State ? "ON" : "OFF", true);
  }
}
void setRelay2Load(bool turnOn) {
  // ไม่ว่าคำสั่งมาจากอัตโนมัติหรือสั่งมือ ระบบแรงดันต่ำมีสิทธิ์ตัดเสมอ
  bool allowedState = turnOn && loadSafetyOK;
  if (allowedState == relay2State) return;
  relay2State = allowedState;
  digitalWrite(RELAY2_PIN, relay2State ? RELAY_ON : RELAY_OFF);
  Serial.printf("[SURPLUS LOAD / RELAY2] %s\n", relay2State ? "ON" : "OFF");
  mqttClient.publish(TOPIC_RELAY2_STATE, relay2State ? "ON" : "OFF", true);
}
void evaluateSafety(float battV) {
  unsigned long now = millis();
  // Relay 1: แยกแบตออกจากคอนโทรลชาร์จเจอร์เมื่อสูงเกินต่อเนื่อง
  // และพักอย่างน้อย 10 นาทีก่อนต่อกลับ
  if (chargeSafetyOK) {
    if (battV >= OVERVOLTAGE_CUTOFF) {
      if (overVoltageStartMs == 0) overVoltageStartMs = now;
      if (now - overVoltageStartMs >= OVERVOLTAGE_DELAY_MS) {
        chargeSafetyOK = false;
        chargeCutoffAtMs = now;
        overVoltageStartMs = 0;
      }
    } else {
      overVoltageStartMs = 0;
    }
  } else if (battV <= OVERVOLTAGE_RECONNECT &&
             now - chargeCutoffAtMs >= CHARGE_MIN_OFF_TIME_MS) {
    chargeSafetyOK = true;
  }
  // Relay 2: ตัดโหลดเมื่อแรงดันต่ำเกิน (hysteresis)
  if (battV < UNDERVOLTAGE_CUTOFF) {
    loadSafetyOK = false;
  } else if (battV > UNDERVOLTAGE_RECONNECT) {
    loadSafetyOK = true;
  }
}
void evaluateSurplusEnergy(float battV) {
  unsigned long now = millis();
  if (!surplusModeEnabled) {
    surplusHighStartMs = 0;
    surplusLowStartMs = 0;
    setRelay2Load(manualRelay2On);
    return;
  }
  if (!loadSafetyOK) {
    surplusHighStartMs = 0;
    surplusLowStartMs = 0;
    setRelay2Load(false);
    return;
  }
  if (!relay2State) {
    surplusLowStartMs = 0;
    if (battV >= SURPLUS_LOAD_ON_V) {
      if (surplusHighStartMs == 0) surplusHighStartMs = now;
      if (now - surplusHighStartMs >= SURPLUS_LOAD_ON_DELAY_MS) {
        surplusHighStartMs = 0;
        setRelay2Load(true);
      }
    } else {
      surplusHighStartMs = 0;
    }
  } else {
    surplusHighStartMs = 0;
    if (battV <= SURPLUS_LOAD_OFF_V) {
      if (surplusLowStartMs == 0) surplusLowStartMs = now;
      if (now - surplusLowStartMs >= SURPLUS_LOAD_OFF_DELAY_MS) {
        surplusLowStartMs = 0;
        setRelay2Load(false);
      }
    } else {
      surplusLowStartMs = 0;
    }
  }
}
// ============================================================================
// MQTT CALLBACK (รับคำสั่งจาก Home Assistant)
// ============================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.toUpperCase();
  Serial.printf("[MQTT] รับคำสั่ง topic=%s payload=%s\n", topic, msg.c_str());
  if (strcmp(topic, TOPIC_RELAY1_SET) == 0) {
    // "OFF" = บังคับปิดค้าง, "ON" = ยกเลิกบังคับปิด (แต่ยังต้องผ่าน safety check)
    manualRelay1Off = (msg == "OFF");
    applyRelayStates();
  } else if (strcmp(topic, TOPIC_RELAY2_MANUAL_SET) == 0) {
    if (msg == "ON" || msg == "OFF") {
      if (surplusModeEnabled) {
        // ขณะโหมดอัตโนมัติทำงาน จะไม่รับคำสั่งเปิดด้วยมือ
        manualRelay2On = false;
      } else {
        manualRelay2On = (msg == "ON");
        setRelay2Load(manualRelay2On);
      }
      mqttClient.publish(TOPIC_RELAY2_MANUAL_STATE,
                         manualRelay2On ? "ON" : "OFF", true);
    }
  } else if (strcmp(topic, TOPIC_SURPLUS_MODE_SET) == 0) {
    if (msg == "ON" || msg == "OFF") {
      surplusModeEnabled = (msg == "ON");
      surplusHighStartMs = 0;
      surplusLowStartMs = 0;
      if (surplusModeEnabled) {
        manualRelay2On = false;
        mqttClient.publish(TOPIC_RELAY2_MANUAL_STATE, "OFF", true);
        setRelay2Load(false);
      } else {
        setRelay2Load(manualRelay2On);
      }
      mqttClient.publish(TOPIC_SURPLUS_MODE_STATE,
                         surplusModeEnabled ? "ON" : "OFF", true);
      Serial.printf("[SURPLUS MODE] %s\n", surplusModeEnabled ? "ON" : "OFF");
    }
  } else if (strcmp(topic, TOPIC_RELAY4_SET) == 0) {
    if (msg == "ON" || msg == "OFF") {
      relay4State = (msg == "ON");
      digitalWrite(RELAY4_PIN, relay4State ? RELAY_ON : RELAY_OFF);
      mqttClient.publish(TOPIC_RELAY4_STATE, relay4State ? "ON" : "OFF", true);
    }
  }
}
// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY (ส่งครั้งเดียวตอนเชื่อมต่อสำเร็จ)
// ============================================================================
// ช่วยสร้าง Discovery ของ sensor พยากรณ์ และแจ้งเตือนถ้าส่งไม่ออก (เช่น payload เกิน buffer)
void publishSensorConfig(const char* deviceBlock, const char* objId, const char* name,
                         const char* stateTopic, const char* unit, const char* devCla,
                         const char* statCla, const char* valTpl, const char* icon) {
  String p = String("{\"name\":\"") + name + "\",\"uniq_id\":\"solar_" + objId
           + "\",\"stat_t\":\"" + stateTopic + "\",";
  if (unit[0])    p += String("\"unit_of_meas\":\"") + unit + "\",";
  if (devCla[0])  p += String("\"dev_cla\":\"") + devCla + "\",";
  if (statCla[0]) p += String("\"stat_cla\":\"") + statCla + "\",";
  if (valTpl[0])  p += String("\"val_tpl\":\"") + valTpl + "\",";
  if (icon[0])    p += String("\"icon\":\"") + icon + "\",";
  p += String("\"avty_t\":\"") + TOPIC_AVAILABILITY + "\"," + deviceBlock + "}";
  String topic = String("homeassistant/sensor/solar_") + objId + "/config";
  if (!mqttClient.publish(topic.c_str(), p.c_str(), true)) {
    Serial.printf("[MQTT] Discovery %s ส่งไม่ออก (payload %u ไบต์)\n", objId, (unsigned)p.length());
  }
}
void publishForecastDiscovery(const char* deviceBlock) {
  // เวลาที่ไม่ระบุ (-1) ให้ HA แสดงเป็น unknown
  const char* minTpl = "{{ none if value|int(-1) < 0 else value|int }}";
  publishSensorConfig(deviceBlock, "soc", "Battery SoC (Coulomb)", TOPIC_SOC,
                      "%", "battery", "measurement", "", "");
  publishSensorConfig(deviceBlock, "time_to_full", "Time To Full", TOPIC_TIME_TO_FULL,
                      "min", "duration", "", minTpl, "mdi:battery-clock");
  publishSensorConfig(deviceBlock, "forecast_status", "Battery Forecast Status", TOPIC_FORECAST_STATUS,
                      "", "", "", "", "mdi:battery-charging");
}
void publishDiscoveryConfig() {
  const char* deviceBlock =
    "\"device\":{\"identifiers\":[\"esp32_solar_monitor\"],"
    "\"name\":\"Solar Monitor\",\"manufacturer\":\"DIY\",\"model\":\"ESP32+INA228\"}";
  String payload;
  payload = String("{\"name\":\"Solar Charge Current\",\"uniq_id\":\"solar_current\",")
          + "\"stat_t\":\"" + TOPIC_CURRENT + "\",\"unit_of_meas\":\"A\","
          + "\"dev_cla\":\"current\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + "\"val_tpl\":\"{{ value }}\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/sensor/solar_current/config", payload.c_str(), true);
  payload = String("{\"name\":\"Battery Voltage\",\"uniq_id\":\"solar_voltage\",")
          + "\"stat_t\":\"" + TOPIC_VOLTAGE + "\",\"unit_of_meas\":\"V\","
          + "\"dev_cla\":\"voltage\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + "\"val_tpl\":\"{{ value }}\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/sensor/solar_voltage/config", payload.c_str(), true);
  payload = String("{\"name\":\"Battery Level (Estimate)\",\"uniq_id\":\"solar_battery_percent\",")
          + "\"stat_t\":\"" + TOPIC_BATTERY_PERCENT + "\",\"unit_of_meas\":\"%\","
          + "\"dev_cla\":\"battery\",\"stat_cla\":\"measurement\","
          + "\"avty_t\":\"" + TOPIC_AVAILABILITY + "\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/sensor/solar_battery_percent/config", payload.c_str(), true);
  payload = String("{\"name\":\"Charge Power\",\"uniq_id\":\"solar_power\",")
          + "\"stat_t\":\"" + TOPIC_POWER + "\",\"unit_of_meas\":\"W\","
          + "\"dev_cla\":\"power\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + "\"val_tpl\":\"{{ value }}\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/sensor/solar_power/config", payload.c_str(), true);
  payload = String("{\"name\":\"Cumulative Energy\",\"uniq_id\":\"solar_energy\",")
          + "\"stat_t\":\"" + TOPIC_ENERGY + "\",\"unit_of_meas\":\"J\","
          + "\"dev_cla\":\"energy\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + "\"val_tpl\":\"{{ value }}\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/sensor/solar_energy/config", payload.c_str(), true);
  payload = String("{\"name\":\"Controller-Battery Relay\",\"uniq_id\":\"solar_relay1\",")
          + "\"stat_t\":\"" + TOPIC_RELAY1_STATE + "\",\"cmd_t\":\"" + TOPIC_RELAY1_SET + "\","
          + "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + deviceBlock + "}";
  mqttClient.publish("homeassistant/switch/solar_relay1/config", payload.c_str(), true);
  // ลบ sensor ที่เกี่ยวกับ ACS712 / time_to_empty ออกจาก HA (ล้าง retained config เดิม)
  mqttClient.publish("homeassistant/sensor/solar_load_current/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_load_power/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_time_to_empty/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_net_current/config", "", true);
  // ลบสวิตช์ Relay 2 รุ่นเดิม แล้วใช้สวิตช์ Manual ที่แยกสถานะคำขอออกจากสถานะจริง
  mqttClient.publish("homeassistant/switch/solar_relay2/config", "", true);
  // Relay 3 ยังไม่พร้อมใช้งาน จึงลบสวิตช์เดิมออก
  mqttClient.publish("homeassistant/switch/solar_relay3/config", "", true);
  // ลบ entity ของระบบเรียนรู้พลังงาน และ Sun guard ที่เอาออกแล้ว (ล้าง retained config เดิม)
  mqttClient.publish("homeassistant/sensor/solar_solar_today_wh/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_pred_total/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_remaining/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_remaining_pct/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_scale/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_learned_days/config", "", true);
  mqttClient.publish("homeassistant/sensor/solar_solar_status/config", "", true);
  mqttClient.publish("homeassistant/binary_sensor/solar_sun_cut_active/config", "", true);
  mqttClient.publish("homeassistant/switch/solar_sun_guard/config", "", true);
  payload = String("{\"name\":\"Surplus Load Status\",\"uniq_id\":\"solar_surplus_load_status\",")
          + "\"stat_t\":\"" + TOPIC_RELAY2_STATE + "\","
          + "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"power\","
          + "\"avty_t\":\"" + TOPIC_AVAILABILITY + "\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/binary_sensor/solar_surplus_load/config", payload.c_str(), true);
  payload = String("{\"name\":\"Relay 2 Manual\",\"uniq_id\":\"solar_relay2_manual\",")
          + "\"stat_t\":\"" + TOPIC_RELAY2_MANUAL_STATE + "\","
          + "\"cmd_t\":\"" + TOPIC_RELAY2_MANUAL_SET + "\","
          + "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"icon\":\"mdi:electric-switch\","
          + "\"avty_t\":\"" + TOPIC_AVAILABILITY + "\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/switch/solar_relay2_manual/config", payload.c_str(), true);
  payload = String("{\"name\":\"Surplus Energy Mode\",\"uniq_id\":\"solar_surplus_energy_mode\",")
          + "\"stat_t\":\"" + TOPIC_SURPLUS_MODE_STATE + "\","
          + "\"cmd_t\":\"" + TOPIC_SURPLUS_MODE_SET + "\","
          + "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"icon\":\"mdi:solar-power\","
          + "\"avty_t\":\"" + TOPIC_AVAILABILITY + "\"," + deviceBlock + "}";
  mqttClient.publish("homeassistant/switch/solar_surplus_energy_mode/config", payload.c_str(), true);
  payload = String("{\"name\":\"Relay 4\",\"uniq_id\":\"solar_relay4\",")
          + "\"stat_t\":\"" + TOPIC_RELAY4_STATE + "\",\"cmd_t\":\"" + TOPIC_RELAY4_SET + "\","
          + "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"avty_t\":\"" + TOPIC_AVAILABILITY + "\","
          + deviceBlock + "}";
  mqttClient.publish("homeassistant/switch/solar_relay4/config", payload.c_str(), true);
  publishForecastDiscovery(deviceBlock);
  Serial.println("[MQTT] ส่ง Discovery config ครบแล้ว — เช็คใน HA ได้เลย");
}
// ============================================================================
// WIFI / MQTT CONNECTION
// ============================================================================
void connectWiFi() {
  Serial.printf("กำลังเชื่อมต่อ WiFi: %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" เชื่อมต่อสำเร็จ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("กำลังเชื่อมต่อ MQTT... ");
    bool ok;
    // ตั้ง Last Will: ถ้า ESP32 หลุดกะทันหัน HA จะเห็นเป็น "offline" อัตโนมัติ
    if (strlen(MQTT_USER) > 0) {
      ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                               TOPIC_AVAILABILITY, 0, true, "offline");
    } else {
      ok = mqttClient.connect(MQTT_CLIENT_ID, TOPIC_AVAILABILITY, 0, true, "offline");
    }
    if (ok) {
      Serial.println("สำเร็จ");
      mqttClient.subscribe(TOPIC_RELAY1_SET);
      mqttClient.subscribe(TOPIC_RELAY2_MANUAL_SET);
      mqttClient.subscribe(TOPIC_RELAY4_SET);
      mqttClient.subscribe(TOPIC_SURPLUS_MODE_SET);
      publishDiscoveryConfig();
      mqttClient.publish(TOPIC_AVAILABILITY, "online", true);
      // ประกาศสถานะรีเลย์ปัจจุบันทันทีหลังต่อ MQTT ติด
      mqttClient.publish(TOPIC_RELAY1_STATE, relay1State ? "ON" : "OFF", true);
      mqttClient.publish(TOPIC_RELAY2_STATE, relay2State ? "ON" : "OFF", true);
      mqttClient.publish(TOPIC_RELAY2_MANUAL_STATE,
                         manualRelay2On ? "ON" : "OFF", true);
      mqttClient.publish(TOPIC_RELAY4_STATE, relay4State ? "ON" : "OFF", true);
      mqttClient.publish(TOPIC_SURPLUS_MODE_STATE,
                         surplusModeEnabled ? "ON" : "OFF", true);
    } else {
      Serial.printf("ล้มเหลว rc=%d ลองใหม่ใน 3 วิ\n", mqttClient.state());
      delay(3000);
    }
  }
}
// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, RELAY_ON);
  digitalWrite(RELAY2_PIN, RELAY_OFF);
  digitalWrite(RELAY3_PIN, RELAY_OFF);
  digitalWrite(RELAY4_PIN, RELAY_OFF);
  prefs.begin("solar", false);       // NVS สำหรับเก็บ SoC
  prefs.remove("solar_blob");        // ล้างข้อมูลเรียนรู้เก่าที่ค้างใน flash (ลบบรรทัดนี้ได้หลังอัปโหลดครั้งแรก)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  ina228_configure();
  connectWiFi();
  mqttClient.setBufferSize(512);   // เพิ่มจาก default 256 ไบต์ กัน Discovery JSON ยาวเกินแล้วส่งไม่ออก
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Serial.println("=== ESP32 INA228 + Relay -> Home Assistant พร้อมทำงาน ===");
}
// ============================================================================
// LOOP
// ============================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();
  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    float battV = ina228_readBusVoltage();
    int batteryPercent = estimateBatteryPercent(battV);
    float I     = ina228_readCurrent();
    float P     = ina228_readPower();
    float E     = ina228_readEnergy();
    Serial.printf("V=%.2fV  I=%.3fA  P=%.2fW  E=%.1fJ\n", battV, I, P, E);
    evaluateSafety(battV);
    evaluateSurplusEnergy(battV);
    applyRelayStates();
    updateBatteryForecast(battV, I);   // แสดงผลเท่านั้น ไม่ยุ่งกับรีเลย์
    char buf[16];
    dtostrf(I, 4, 3, buf);     mqttClient.publish(TOPIC_CURRENT, buf);
    dtostrf(battV, 4, 2, buf); mqttClient.publish(TOPIC_VOLTAGE, buf);
    snprintf(buf, sizeof(buf), "%d", batteryPercent); mqttClient.publish(TOPIC_BATTERY_PERCENT, buf);
    dtostrf(P, 4, 2, buf);     mqttClient.publish(TOPIC_POWER, buf);
    dtostrf(E, 6, 1, buf);     mqttClient.publish(TOPIC_ENERGY, buf);
    publishBatteryForecast();
  }
}