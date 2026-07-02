//////////////////////////////////////////////////////////////////////////////////////////
//
//    grok_integratedTCP.cpp
//    FULLY COMPLETE VERSION - Copy and paste this entire block
//    ADS1220 exact behavior + optional IMU/MAG
//    Correct packet types for collect_data.py
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <deque>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// ====================== ADS1220 PINS (YOUR WIRING) ======================
#define A_DRDY_PIN 9
#define A_CS_PIN 10
#define B_DRDY_PIN 7
#define B_CS_PIN 8
#define C_DRDY_PIN 5
#define C_CS_PIN 6
#define D_DRDY_PIN 3
#define D_CS_PIN 4
#define E_DRDY_PIN 2
#define E_CS_PIN A1

const int SPI_SCK = 13;
const int SPI_MISO = 11;
const int SPI_MOSI = 12;

const int MAX_SENSORS = 5;
const int NUM_SENSORS = 3;
const uint8_t dr_code = DR_330SPS;
const SPISettings spi_settings(2000000, MSBFIRST, SPI_MODE1);

// ====================== IMU/MAG PINS ======================
constexpr int IMU_CS_PIN = A4;
constexpr int MAG_CS_PIN = A5;
constexpr int IMU_INT_PIN = A6;
constexpr int MAG_INT_PIN = A7;

// ====================== Network ======================
const char* ssid = "Hi-STIFFS_Host";
const char* password = "BYUCropBio";
const char* host_ip = "192.168.137.1";
const int host_port = 8080;
WiFiClient client;

const char* NANO_ID = "01";
const unsigned long BATCH_SEND_INTERVAL_MS = 10;

// Packet types
#define PKT_IMU 1
#define PKT_MAG 2
#define PKT_ADS 3

// ====================== ADS State ======================
struct SensorConfig {
  char id; int cs_pin; int drdy_pin;
};

SensorConfig all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN}, {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN}, {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];
int32_t raw_values[MAX_SENSORS][2];
uint64_t timestamps[MAX_SENSORS];
uint8_t current_channels[MAX_SENSORS] = {0};
volatile uint8_t ready_mask = 0;
uint64_t time_init = 0;

std::deque<std::vector<uint8_t>> packet_queue;
uint16_t seq_num = 0;
unsigned long last_batch_send = 0;

bool hasSerial = false;
enum State_Serial { NO_SERIAL, HAS_SERIAL };
State_Serial SerialState;
enum State_WiFi { CONNECTING, CONNECTED };
State_WiFi WiFiState = CONNECTING;

// ====================== ImuMagSpi ======================
namespace ImuMagSpi {
  struct ImuSample { uint64_t ts_us; int16_t ax, ay, az, gx, gy, gz; };
  struct MagSample { uint64_t ts_us; int16_t mx, my, mz; };

  struct Config {
    SPIClass* spi = &SPI;
    SPISettings spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE3);
    int imuCsPin = -1, magCsPin = -1, magIntPin = -1;
    BaseType_t taskCore = 1;
    UBaseType_t imuTaskPriority = configMAX_PRIORITIES - 2;
    UBaseType_t magTaskPriority = configMAX_PRIORITIES - 3;
    uint32_t imuTaskStack = 4096, magTaskStack = 4096;
    size_t imuQueueDepth = 512, magQueueDepth = 32;
  };

  namespace {
    Config g_cfg;
    bool g_configured = false;
    bool g_imuInited = false;
    bool g_magInited = false;
    TaskHandle_t g_imuTask = nullptr;
    TaskHandle_t g_magTask = nullptr;
    QueueHandle_t g_imuQueue = nullptr;
    QueueHandle_t g_magQueue = nullptr;
    portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
    volatile uint32_t g_imuCount = 0, g_magCount = 0;
    volatile uint32_t g_imuDropped = 0, g_magDropped = 0;
    volatile int64_t g_imuIsrTs = 0, g_magIsrTs = 0;

    // ISM registers
    constexpr uint8_t ISM_WHO_AM_I = 0x0F;
    constexpr uint8_t ISM_COUNTER_BDR_REG1 = 0x0B;
    constexpr uint8_t ISM_INT1_CTRL = 0x0D;
    constexpr uint8_t ISM_CTRL1_XL = 0x10;
    constexpr uint8_t ISM_CTRL2_G = 0x11;
    constexpr uint8_t ISM_CTRL3_C = 0x12;
    constexpr uint8_t ISM_OUTX_L_G = 0x22;
    constexpr uint8_t ISM_CTRL1_XL_VAL = 0xA8;
    constexpr uint8_t ISM_CTRL2_G_VAL = 0xAC;
    constexpr uint8_t ISM_CTRL3_C_VAL = 0x44;
    constexpr uint8_t ISM_INT1_CTRL_VAL = 0x01;
    constexpr uint8_t ISM_COUNTER_BDR_VAL = 0x80;

    // MAG registers
    constexpr uint8_t MAG_WHO_AM_I = 0x0F;
    constexpr uint8_t MAG_CTRL_REG1 = 0x20;
    constexpr uint8_t MAG_CTRL_REG2 = 0x21;
    constexpr uint8_t MAG_CTRL_REG3 = 0x22;
    constexpr uint8_t MAG_CTRL_REG4 = 0x23;
    constexpr uint8_t MAG_CTRL_REG5 = 0x24;
    constexpr uint8_t MAG_OUT_X_L = 0x28;
    constexpr uint8_t MAG_CTRL_REG1_VAL = 0x02;
    constexpr uint8_t MAG_CTRL_REG2_VAL = 0x00;
    constexpr uint8_t MAG_CTRL_REG3_VAL = 0x00;
    constexpr uint8_t MAG_CTRL_REG4_VAL = 0x00;
    constexpr uint8_t MAG_CTRL_REG5_VAL = 0x40;

    inline void wr(int cs, uint8_t reg, uint8_t val) {
      g_cfg.spi->beginTransaction(g_cfg.spiSettings);
      digitalWrite(cs, LOW);
      g_cfg.spi->transfer(reg & 0x7F);
      g_cfg.spi->transfer(val);
      digitalWrite(cs, HIGH);
      g_cfg.spi->endTransaction();
    }
    inline uint8_t rd(int cs, uint8_t reg) {
      g_cfg.spi->beginTransaction(g_cfg.spiSettings);
      digitalWrite(cs, LOW);
      g_cfg.spi->transfer(reg | 0x80);
      uint8_t v = g_cfg.spi->transfer(0);
      digitalWrite(cs, HIGH);
      g_cfg.spi->endTransaction();
      return v;
    }
    inline void imuBurst(uint8_t reg, uint8_t* buf, size_t n) {
      g_cfg.spi->beginTransaction(g_cfg.spiSettings);
      digitalWrite(g_cfg.imuCsPin, LOW);
      g_cfg.spi->transfer(reg | 0x80);
      g_cfg.spi->transferBytes(nullptr, buf, n);
      digitalWrite(g_cfg.imuCsPin, HIGH);
      g_cfg.spi->endTransaction();
    }
    inline void magBurst(uint8_t reg, uint8_t* buf, size_t n) {
      g_cfg.spi->beginTransaction(g_cfg.spiSettings);
      digitalWrite(g_cfg.magCsPin, LOW);
      g_cfg.spi->transfer(reg | 0xC0);
      g_cfg.spi->transferBytes(nullptr, buf, n);
      digitalWrite(g_cfg.magCsPin, HIGH);
      g_cfg.spi->endTransaction();
    }

    bool waitWho(int cs, uint8_t expected) {
      for (int i = 0; i < 10; ++i) if (rd(cs, 0x0F) == expected) return true; else delay(5);
      return false;
    }

    void imuTask(void*) {
      uint8_t buf[12];
      for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ImuSample s; s.ts_us = g_imuIsrTs;
        imuBurst(ISM_OUTX_L_G, buf, sizeof(buf));
        s.gx = (int16_t)(buf[0] | (buf[1]<<8)); s.gy = (int16_t)(buf[2] | (buf[3]<<8)); s.gz = (int16_t)(buf[4] | (buf[5]<<8));
        s.ax = (int16_t)(buf[6] | (buf[7]<<8)); s.ay = (int16_t)(buf[8] | (buf[9]<<8)); s.az = (int16_t)(buf[10] | (buf[11]<<8));
        if (xQueueSend(g_imuQueue, &s, 0) == pdTRUE) { portENTER_CRITICAL(&g_mux); g_imuCount++; portEXIT_CRITICAL(&g_mux); }
        else { portENTER_CRITICAL(&g_mux); g_imuDropped++; portEXIT_CRITICAL(&g_mux); }
      }
    }

    void magTask(void*) {
      uint8_t buf[6];
      for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        MagSample s; s.ts_us = g_magIsrTs;
        magBurst(MAG_OUT_X_L, buf, sizeof(buf));
        s.mx = (int16_t)(buf[0] | (buf[1]<<8)); s.my = (int16_t)(buf[2] | (buf[3]<<8)); s.mz = (int16_t)(buf[4] | (buf[5]<<8));
        if (xQueueSend(g_magQueue, &s, 0) == pdTRUE) { portENTER_CRITICAL(&g_mux); g_magCount++; portEXIT_CRITICAL(&g_mux); }
        else { portENTER_CRITICAL(&g_mux); g_magDropped++; portEXIT_CRITICAL(&g_mux); }
      }
    }
  }

  void begin(const Config& cfg) { g_cfg = cfg; g_configured = true; }
  bool initImu() {
    if (!g_configured || g_cfg.imuCsPin < 0) return false;
    pinMode(g_cfg.imuCsPin, OUTPUT); digitalWrite(g_cfg.imuCsPin, HIGH); delay(50);
    if (!waitWho(g_cfg.imuCsPin, 0x6B)) return false;
    wr(g_cfg.imuCsPin, ISM_CTRL3_C, ISM_CTRL3_C_VAL);
    wr(g_cfg.imuCsPin, ISM_CTRL1_XL, ISM_CTRL1_XL_VAL);
    wr(g_cfg.imuCsPin, ISM_CTRL2_G, ISM_CTRL2_G_VAL);
    wr(g_cfg.imuCsPin, ISM_COUNTER_BDR_REG1, ISM_COUNTER_BDR_VAL);
    wr(g_cfg.imuCsPin, ISM_INT1_CTRL, ISM_INT1_CTRL_VAL);
    g_imuInited = true; return true;
  }
  bool initMag() {
    if (!g_configured || g_cfg.magCsPin < 0) return false;
    pinMode(g_cfg.magCsPin, OUTPUT); digitalWrite(g_cfg.magCsPin, HIGH); delay(10);
    if (!waitWho(g_cfg.magCsPin, 0x3D)) return false;
    wr(g_cfg.magCsPin, MAG_CTRL_REG2, MAG_CTRL_REG2_VAL);
    wr(g_cfg.magCsPin, MAG_CTRL_REG4, MAG_CTRL_REG4_VAL);
    wr(g_cfg.magCsPin, MAG_CTRL_REG5, MAG_CTRL_REG5_VAL);
    wr(g_cfg.magCsPin, MAG_CTRL_REG1, MAG_CTRL_REG1_VAL);
    wr(g_cfg.magCsPin, MAG_CTRL_REG3, MAG_CTRL_REG3_VAL);
    g_magInited = true; return true;
  }
  bool startTasks() {
    if (g_imuInited && !g_imuTask) {
      g_imuQueue = xQueueCreate(g_cfg.imuQueueDepth, sizeof(ImuSample));
      if (g_imuQueue) xTaskCreatePinnedToCore(imuTask, "imu", g_cfg.imuTaskStack, nullptr, g_cfg.imuTaskPriority, &g_imuTask, g_cfg.taskCore);
    }
    if (g_magInited && !g_magTask) {
      g_magQueue = xQueueCreate(g_cfg.magQueueDepth, sizeof(MagSample));
      if (g_magQueue) xTaskCreatePinnedToCore(magTask, "mag", g_cfg.magTaskStack, nullptr, g_cfg.magTaskPriority, &g_magTask, g_cfg.taskCore);
      if (g_cfg.magIntPin >= 0) {
        uint8_t dummy[6];
        for (int i = 0; i < 5; ++i) {
          magBurst(MAG_OUT_X_L, dummy, sizeof(dummy));
          delayMicroseconds(50);
          if (digitalRead(g_cfg.magIntPin) == LOW) break;
        }
      }
    }
    return true;
  }
  void IRAM_ATTR onImuDrdyFromIsr(int64_t ts) { g_imuIsrTs = ts; BaseType_t hpw = pdFALSE; vTaskNotifyGiveFromISR(g_imuTask, &hpw); if (hpw) portYIELD_FROM_ISR(); }
  void IRAM_ATTR onMagDrdyFromIsr(int64_t ts) { g_magIsrTs = ts; BaseType_t hpw = pdFALSE; vTaskNotifyGiveFromISR(g_magTask, &hpw); if (hpw) portYIELD_FROM_ISR(); }
  QueueHandle_t getImuQueue() { return g_imuQueue; }
  QueueHandle_t getMagQueue() { return g_magQueue; }
  void kickMag() { if (g_magTask) xTaskNotifyGive(g_magTask); }
} // ImuMagSpi

// ====================== ADS ISRs ======================
void IRAM_ATTR handleDrdyA() { const int i=0; unsigned long t=micros(); SPI.beginTransaction(spi_settings); digitalWrite(A_CS_PIN,LOW); delayMicroseconds(1); byte b[3]={SPI.transfer(0),SPI.transfer(0),SPI.transfer(0)}; digitalWrite(A_CS_PIN,HIGH); SPI.endTransaction(); long v=((long)b[0]<<16)|((long)b[1]<<8)|b[2]; int32_t val=(v<<8)>>8; if(current_channels[i]==0){raw_values[i][0]=val;adcs[i].select_mux_channels(MUX_AIN2_AIN3);current_channels[i]=1;}else{raw_values[i][1]=val;adcs[i].select_mux_channels(MUX_AIN0_AIN1);current_channels[i]=0;timestamps[i]=t-time_init;ready_mask|=(1<<i);}}
void IRAM_ATTR handleDrdyB() { const int i=1; unsigned long t=micros(); SPI.beginTransaction(spi_settings); digitalWrite(B_CS_PIN,LOW); delayMicroseconds(1); byte b[3]={SPI.transfer(0),SPI.transfer(0),SPI.transfer(0)}; digitalWrite(B_CS_PIN,HIGH); SPI.endTransaction(); long v=((long)b[0]<<16)|((long)b[1]<<8)|b[2]; int32_t val=(v<<8)>>8; if(current_channels[i]==0){raw_values[i][0]=val;adcs[i].select_mux_channels(MUX_AIN2_AIN3);current_channels[i]=1;}else{raw_values[i][1]=val;adcs[i].select_mux_channels(MUX_AIN0_AIN1);current_channels[i]=0;timestamps[i]=t-time_init;ready_mask|=(1<<i);}}
void IRAM_ATTR handleDrdyC() { const int i=2; unsigned long t=micros(); SPI.beginTransaction(spi_settings); digitalWrite(C_CS_PIN,LOW); delayMicroseconds(1); byte b[3]={SPI.transfer(0),SPI.transfer(0),SPI.transfer(0)}; digitalWrite(C_CS_PIN,HIGH); SPI.endTransaction(); long v=((long)b[0]<<16)|((long)b[1]<<8)|b[2]; int32_t val=(v<<8)>>8; if(current_channels[i]==0){raw_values[i][0]=val;adcs[i].select_mux_channels(MUX_AIN2_AIN3);current_channels[i]=1;}else{raw_values[i][1]=val;adcs[i].select_mux_channels(MUX_AIN0_AIN1);current_channels[i]=0;timestamps[i]=t-time_init;ready_mask|=(1<<i);}}
void IRAM_ATTR handleDrdyD() { const int i=3; unsigned long t=micros(); SPI.beginTransaction(spi_settings); digitalWrite(D_CS_PIN,LOW); delayMicroseconds(1); byte b[3]={SPI.transfer(0),SPI.transfer(0),SPI.transfer(0)}; digitalWrite(D_CS_PIN,HIGH); SPI.endTransaction(); long v=((long)b[0]<<16)|((long)b[1]<<8)|b[2]; int32_t val=(v<<8)>>8; if(current_channels[i]==0){raw_values[i][0]=val;adcs[i].select_mux_channels(MUX_AIN2_AIN3);current_channels[i]=1;}else{raw_values[i][1]=val;adcs[i].select_mux_channels(MUX_AIN0_AIN1);current_channels[i]=0;timestamps[i]=t-time_init;ready_mask|=(1<<i);}}
void IRAM_ATTR handleDrdyE() { const int i=4; unsigned long t=micros(); SPI.beginTransaction(spi_settings); digitalWrite(E_CS_PIN,LOW); delayMicroseconds(1); byte b[3]={SPI.transfer(0),SPI.transfer(0),SPI.transfer(0)}; digitalWrite(E_CS_PIN,HIGH); SPI.endTransaction(); long v=((long)b[0]<<16)|((long)b[1]<<8)|b[2]; int32_t val=(v<<8)>>8; if(current_channels[i]==0){raw_values[i][0]=val;adcs[i].select_mux_channels(MUX_AIN2_AIN3);current_channels[i]=1;}else{raw_values[i][1]=val;adcs[i].select_mux_channels(MUX_AIN0_AIN1);current_channels[i]=0;timestamps[i]=t-time_init;ready_mask|=(1<<i);}}

void IRAM_ATTR handleImuDrdy() { ImuMagSpi::onImuDrdyFromIsr(esp_timer_get_time()); }
void IRAM_ATTR handleMagDrdy() { ImuMagSpi::onMagDrdyFromIsr(esp_timer_get_time()); }

// ====================== Helpers ======================
void broadcast_command(uint8_t cmd) {
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, LOW);
  SPI.transfer(cmd);
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, HIGH);
}
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
}
void enableADCInterrupts() {
  void (*h[5])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE};
  for (int i = 0; i < NUM_SENSORS; i++) attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), h[i], FALLING);
}
void initializeADCs() {
  broadcast_command(RESET); delay(10);
  disableADCInterrupts();
  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].begin(all_configs[i].cs_pin, all_configs[i].drdy_pin);
    adcs[i].set_OperationMode(MODE_TURBO);
    adcs[i].set_data_rate(dr_code);
    adcs[i].PGA_ON();
    adcs[i].set_pga_gain(PGA_GAIN_128);
    adcs[i].set_VREF(VREF_ANALOG);
    adcs[i].set_conv_mode_continuous();
    adcs[i].select_mux_channels(MUX_AIN0_AIN1);
    current_channels[i] = 0;
    if (hasSerial) Serial.printf("Setup Sensor %d CS:%d DRDY:%d\n", i, all_configs[i].cs_pin, all_configs[i].drdy_pin);
    delay(100);
  }
  if (hasSerial) Serial.println("ADS datastream starting");
  delay(100);
  uint16_t sps = 660;
  unsigned long p = 1000000UL / sps;
  unsigned long stagger = p / 5;
  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS-1) delayMicroseconds(stagger);
  }
  enableADCInterrupts();
  delay(50);
  time_init = esp_timer_get_time();
}

// ====================== FIXED Packet Builder ======================
bool checkDataReady() {
  uint8_t mask = (1U << NUM_SENSORS) - 1;
  if (ready_mask == mask) { ready_mask = 0; return true; }
  return false;
}

void queueDataPacket() {
  std::vector<uint8_t> payload;
  payload.reserve(1024);
  payload.push_back(atoi(NANO_ID));

  // IMU
  QueueHandle_t imuQ = ImuMagSpi::getImuQueue();
  ImuMagSpi::ImuSample imuS;
  while (imuQ && xQueueReceive(imuQ, &imuS, 0) == pdTRUE) {
    payload.push_back(PKT_IMU);
    uint8_t* p = (uint8_t*)&imuS;
    payload.insert(payload.end(), p, p + sizeof(ImuMagSpi::ImuSample));
  }

  // MAG
  QueueHandle_t magQ = ImuMagSpi::getMagQueue();
  ImuMagSpi::MagSample magS;
  while (magQ && xQueueReceive(magQ, &magS, 0) == pdTRUE) {
    payload.push_back(PKT_MAG);
    uint8_t* p = (uint8_t*)&magS;
    payload.insert(payload.end(), p, p + sizeof(ImuMagSpi::MagSample));
  }

  // ADS
  if (checkDataReady()) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint64_t ts = timestamps[i];
      int32_t ch1 = raw_values[i][0];
      int32_t ch2 = raw_values[i][1];
      uint8_t adc_id = all_configs[i].id - 'A';
      payload.push_back(PKT_ADS);
      uint8_t* tp = (uint8_t*)&ts; payload.insert(payload.end(), tp, tp + 8);
      payload.push_back(adc_id);
      uint8_t* c1p = (uint8_t*)&ch1; payload.insert(payload.end(), c1p, c1p + 4);
      uint8_t* c2p = (uint8_t*)&ch2; payload.insert(payload.end(), c2p, c2p + 4);
    }
  }

  if (payload.size() <= 1) return;

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < payload.size(); i++) {
    crc ^= payload[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }

  std::vector<uint8_t> packet;
  uint16_t len = payload.size();
  packet.push_back(len & 0xFF); packet.push_back((len >> 8) & 0xFF);
  packet.push_back(seq_num & 0xFF); packet.push_back((seq_num >> 8) & 0xFF); seq_num++;
  packet.push_back(crc & 0xFF); packet.push_back((crc >> 8) & 0xFF);
  packet.insert(packet.end(), payload.begin(), payload.end());

  packet_queue.push_back(std::move(packet));
}

bool sendQueuedDataTCP() {
  if (!client.connected()) return false;
  bool sent = false;
  while (!packet_queue.empty()) {
    const auto& p = packet_queue.front();
    if (client.write(p.data(), p.size()) != (int)p.size()) return false;
    packet_queue.pop_front();
    sent = true;
  }
  if (sent) client.flush();
  return true;
}

void sendDataSerial() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print((double)timestamps[i] / 1000000.0, 6);
    Serial.print(","); Serial.print(raw_values[i][0]);
    Serial.print(","); Serial.print(raw_values[i][1]);
  }
  Serial.println();
}

void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();
  if (now - lastRetry < 5000) return;
  lastRetry = now;
  if (hasSerial) Serial.println("Connecting to Hi-STIFFS_Host");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    if (hasSerial) { Serial.println("WiFi connected"); Serial.print("IP: "); Serial.println(WiFi.localIP()); }
    if (client.connect(host_ip, host_port)) {
      if (hasSerial) Serial.println("TCP connected");
      WiFiState = CONNECTED;
      initializeADCs();
      ready_mask = 0;
      packet_queue.clear();
    }
  }
}

void monitorConnection() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck >= 1000) {
    lastCheck = now;
    if (WiFi.status() != WL_CONNECTED || !client.connected()) {
      client.stop();
      WiFiState = CONNECTING;
    }
  }
}

void handleOTA() { ArduinoOTA.handle(); }

// ====================== SETUP ======================
void setup() {
  Serial.begin(1000000);
  unsigned long st = millis();
  while (!Serial && millis() - st < 5000) {}
  hasSerial = (bool)Serial;
  if (hasSerial) Serial.println("grok_integratedTCP starting");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  WiFi.mode(WIFI_STA);

  // IMU/MAG
  ImuMagSpi::Config cfg;
  cfg.imuCsPin = IMU_CS_PIN;
  cfg.magCsPin = MAG_CS_PIN;
  cfg.magIntPin = MAG_INT_PIN;
  ImuMagSpi::begin(cfg);
  bool imuOk = ImuMagSpi::initImu();
  bool magOk = ImuMagSpi::initMag();
  if (hasSerial) {
    Serial.print("IMU: "); Serial.println(imuOk ? "OK" : "missing");
    Serial.print("MAG: "); Serial.println(magOk ? "OK" : "missing");
  }
  ImuMagSpi::startTasks();

  pinMode(IMU_INT_PIN, INPUT);
  pinMode(MAG_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), handleImuDrdy, RISING);
  attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), handleMagDrdy, RISING);
  if (magOk) ImuMagSpi::kickMag();

  initializeADCs();
  disableADCInterrupts();

  SerialState = hasSerial ? HAS_SERIAL : NO_SERIAL;
  last_batch_send = millis();
}

// ====================== LOOP ======================
void loop() {
  switch (SerialState) {
    case NO_SERIAL:
      if (WiFiState == CONNECTED) {
        queueDataPacket();
        if (millis() - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
          sendQueuedDataTCP();
          last_batch_send = millis();
        }
        monitorConnection();
      } else {
        disableADCInterrupts();
        handleOTA();
        connectToHost();
      }
      break;
    case HAS_SERIAL:
      if (WiFiState == CONNECTED) {
        queueDataPacket();
        sendDataSerial();
        if (millis() - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
          sendQueuedDataTCP();
          last_batch_send = millis();
        }
        monitorConnection();
      } else {
        disableADCInterrupts();
        handleOTA();
        connectToHost();
      }
      break;
  }
}