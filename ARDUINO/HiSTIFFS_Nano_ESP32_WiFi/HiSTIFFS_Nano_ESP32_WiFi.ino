/*
 * 9-DOF IMU + MAG High-Rate Binary Streaming Sketch — MULTI-TASK REFACTORED
 * Arduino Nano ESP32 (WiFi client) — ISM330DHCX (IMU) + LIS3MDL (MAG)
 * =======================================================================
 *
 * PURPOSE
 * -------
 * Achieve the highest sustainable unique sample rate for 3D pose tracking
 * over rolling 1-2 s windows. The ISM330DHCX (accel +
 * gyro) drives every packet at TARGET_IMU_RATE_HZ. The LIS3MDL magnetometer
 * is read only at its target rate and the three int16 values are simply
 * latched and reused in all intervening packets. This keeps SPI traffic and
 * CPU load minimal while still supplying fresh 9-DOF vectors at the IMU rate.
 *
 * ARCHITECTURE (FreeRTOS + dedicated esp_timers)
 * ----------------------------------------------
 * SamplingTask (configMAX_PRIORITIES-4, pinned to core 1)
 *   - Woken by hardware samplingTimer_IMU ISR at exactly IMU_INTERVAL_US.
 *   - Burst-reads 12 bytes from IMU.
 *   - Simple uint32 counter; every Nth wake (TARGET_IMU_RATE_HZ / TARGET_MAG_RATE_HZ)
 *     performs a fresh 6-byte MAG read and updates the three latched int16 values.
 *   - Builds a 26-byte packet and enqueues it under bufferMutex ONLY when
 *     the lightweight flag g_isConnected is true.
 *   - Never blocks on WiFi/TCP
 *
 * NetworkTask (configMAX_PRIORITIES-5, pinned to core 0)
 *   - Owns the complete WiFi STA + persistent TCP client state machine.
 *   - CONNECTING state: calls connectToHost() (blocking, up to ~8 s).
 *   - CONNECTED state: blocks on batchSemaphore (50 ms esp_timer), then
 *     executes sendQueuedDataTCP(). Snapshot of queue head/tail is taken
 *     under mutex; actual TCP writes occur outside the mutex; commit of
 *     q_tail/q_count always happens after a successful write so that stale
 *     data is never retransmitted.
 *   - Periodic monitorConnection() health check.
 *
 * Timing sources (both ESP_TIMER_TASK)
 *   - samplingTimer_IMU: period = IMU_INTERVAL_US → hardware-precise wake-ups.
 *   - batch_timer: period = BATCH_SEND_INTERVAL_MS * 1000 µs (50 ms) →
 *     wakes NetworkTask to transmit the accumulated batch.
 *
 * Buffer & synchronization
 *   - Zero-heap circular buffer: static packet_pool[MAX_PACKETS_IMU][PACKET_SIZE].
 *   - bufferMutex protects only the absolute minimum critical sections
 *     (enqueue, snapshot, commit). SamplingTask reads g_isConnected
 *     lock-free for the fast-path decision.
 *
 * CONFIGURATION (datasheet-driven helpers)
 * ----------------------------------------
 * TARGET_IMU_RATE_HZ = 3000 → selectIMU_ODR_Bits() returns the smallest
 *   HP ODR bits >= target (ISM330DHCX CTRL1_XL / CTRL2_G tables).
 *   Full-scale selection via selectAccelFS_Bits / selectGyroFS_Bits.
 * TARGET_MAG_RATE_HZ = 1000 → selectMAG_CTRL_REG1_Val() + selectMAG_OMZ_Bits()
 *   (LIS3MDL FAST_ODR path). Full-scale via selectMagFS_Bits.
 * SPI fixed at 8 MHz, MSBFIRST, SPI_MODE3 for maximum margin on long
 * field cables and noise immunity.
 *
 * PACKET STRUCTURE (little-endian, 26 bytes on the wire)
 * ------------------------------------------------------
 * Offset  Size   Field
 * 0       1      uint8_t payload_length;    // always 23
 * 1       2      uint16_t crc16;            // CRC-16/ARC (poly 0xA001, init 0xFFFF)
 *                                           //   computed ONLY over bytes 4..27
 * 3       1      uint8_t  nano_id;          // decimal value of NANO_ID
 * 4       4      uint32_t timestamp_us;     // (esp_timer_get_time() - time_init)
 *                                           //   microseconds since TCP connect epoch
 * 8      18     int16_t  data[9];           // gx, gy, gz, ax, ay, az, mx, my, mz
 *                                           //   raw signed counts (little-endian)
 *                                           //   mx/my/mz = last latched fresh MAG sample
 *
 * CRC is generated once per sample inside queueDataPacket_IMU_MAG using the
 * pre-computed 256-entry table. Header bytes are written after the CRC is
 * known so the receiver can validate the payload immediately.
 *
 * TRANSMISSION BEHAVIOUR
 * ----------------------
 * sendQueuedDataTCP() always advances q_tail after any successful partial or
 * full write. Under overload or WiFi interference some packets may be
 * overwritten before transmission; the host will see CRC failures or gaps.
 * This is the intended policy.
 *
 * RUNTIME
 * -------
 * setup() brings up SPI, CRC table, WiFi STA, both sensor chips (via the
 * exact register sequences and helper functions), creates the mutex and two
 * binary semaphores, starts both periodic esp_timers, then creates the two
 * pinned tasks. The original loop() is left idle (vTaskDelay(portMAX_DELAY)).
 * All design decisions are documented with // Decision: and // Intent:
 * comments at the point of implementation.
 */

// ========== INCLUDES ==========
#include <SPI.h>                          // SPIClass + beginTransaction/endTransaction API used for every 8 MHz MSBFIRST SPI_MODE3 transaction to both sensors (burst 12-byte IMU or 6-byte MAG reads)
#include <WiFi.h>                         // WiFi STA + WiFiClient used by NetworkTask to maintain the single persistent TCP connection that carries every 27-byte binary packet to the host collector
#include "esp_timer.h"                    // esp_timer_create + esp_timer_start_periodic used to create the two hardware-timed sources: samplingTimer_IMU (IMU_INTERVAL_US) and batch_timer (50 ms)
#include "freertos/FreeRTOS.h"            // BaseType_t, pdTRUE, pdFALSE, portMAX_DELAY, configMAX_PRIORITIES — foundational types for all semaphore and task-priority logic in this sketch
#include "freertos/task.h"                // xTaskCreatePinnedToCore, vTaskDelay, portYIELD_FROM_ISR — creates the two pinned tasks (SamplingTask on core 1, NetworkTask on core 0)
#include "freertos/semphr.h"              // xSemaphoreCreateMutex, xSemaphoreCreateBinary, xSemaphoreTake/Give, xSemaphoreGiveFromISR — protects the circular packet buffer and wakes the two tasks from timer ISRs
#include "Protocentral_ADS1220.h"

// ========== REGISTER ADDRESSES ==========
#define IMU_WHO_AM_I          0x0F        // ISM330DHCX WHO_AM_I register (datasheet §6.1); read once in bringup_IMU() and must return IMU_WHO_AM_I_VAL before any configuration proceeds
#define IMU_CTRL1_XL          0x10        // ISM330DHCX CTRL1_XL; written in bringup_IMU() with ODR[7:4] + FS[3:2] bits selected by selectIMU_ODR_Bits() and selectAccelFS_Bits()
#define IMU_CTRL2_G           0x11        // ISM330DHCX CTRL2_G; written with ODR[7:4] + combined FS bits (including FS_125/FS_4000) chosen by selectGyroFS_Bits()
#define IMU_CTRL3_C           0x12        // ISM330DHCX CTRL3_C; written with IMU_CTRL3_C_VAL to enable BDU (coherent multi-byte reads) + auto-increment for single-transaction 12-byte bursts
#define IMU_STATUS_REG        0x1E        // ISM330DHCX STATUS_REG (XLDA/GDA bits); defined for completeness but unused in the timer-driven hot path — sampling is purely periodic
#define IMU_OUTX_L_G          0x22        // First output register of gyro X_L; readIMU() issues SPI command (IMU_OUTX_L_G | 0x80) + auto-increment to fetch all 12 bytes (gx..az) in one transaction
#define IMU_INT1_CTRL         0x0D        // ISM330DHCX INT1_CTRL; written with 0x01 to route data-ready, but the physical INT1 pin is never read — timing source is the esp_timer, not interrupts
#define IMU_WHO_AM_I_VAL      0x6B        // Expected return value from ISM330DHCX WHO_AM_I read (datasheet); used as the sole pass/fail criterion inside bringup_IMU() before ODR/FS configuration
#define IMU_CTRL3_C_VAL       0x44        // Value written to CTRL3_C (BDU=1 + IF_INC=1); guarantees that the 12-byte gyro+accel burst read always returns a coherent snapshot for pose math

#define MAG_WHO_AM_I          0x0F        // LIS3MDL WHO_AM_I register (datasheet §6.1); read in bringup_MAG() and must return MAG_WHO_AM_I_VAL before any MAG configuration writes
#define MAG_CTRL_REG1         0x20        // LIS3MDL CTRL_REG1; written with result of selectMAG_CTRL_REG1_Val() to set FAST_ODR + performance mode that achieves TARGET_MAG_RATE_HZ
#define MAG_CTRL_REG2         0x21        // LIS3MDL CTRL_REG2; written with selectMagFS_Bits() result to set the ±12 gauss (or other) full-scale used for the latched magnetometer values
#define MAG_CTRL_REG3         0x22        // LIS3MDL CTRL_REG3; written with 0x00 to select continuous-conversion mode (lowest latency) and disable I²C so only SPI is active on the shared bus
#define MAG_CTRL_REG4         0x23        // LIS3MDL CTRL_REG4; written with OMZ bits (matching X/Y performance) + BLE=0 (little-endian) so the three int16_t casts in readMAG() are correct
#define MAG_CTRL_REG5         0x24        // LIS3MDL CTRL_REG5; written with 0x00 (BDU=0, FAST_READ=0) so every conversion immediately updates the output registers for fresh data in every packet
#define MAG_STATUS_REG        0x27        // LIS3MDL STATUS_REG (ZYXDA bit); defined for symmetry but unused — MAG freshness is handled by the simple counter inside SamplingTask instead of polling
#define MAG_OUT_X_L           0x28        // First magnetometer output register (X_L); readMAG() issues command (MAG_OUT_X_L | 0xC0) + auto-increment to fetch all 6 bytes in one minimal SPI transaction
#define MAG_WHO_AM_I_VAL      0x3D        // Expected return value from LIS3MDL WHO_AM_I read (datasheet); sole validation that the correct 3-axis magnetometer is present before configuration

#define ICB_WREG              0x40        // Write register
#define ICB_RREG              0x20        // Read register
#define CONFIG_REG0_ADDRESS   0x00
#define REG_CONFIG0_MUX_MASK  0xF0
#define MUX_AIN0_AIN1         0x00
#define MUX_AIN2_AIN3         0x50
#define SPI_MASTER_DUMMY      0xFF

// ========== PIN DEFINITIONS ==========
#define SPI_SCK     13                    // GPIO13 on Nano ESP32 wired to SCK of both ISM330DHCX and LIS3MDL; driven at 8 MHz by the SPI library for maximum noise margin on field cables
#define SPI_MISO    11                    // GPIO11 wired to MISO/SDO of both sensors; receives the 12-byte IMU or 6-byte MAG burst during every sample inside readIMU() / readMAG()
#define SPI_MOSI    12                    // GPIO12 wired to MOSI/SDI of both sensors; carries the register address byte (with R/W bit) plus any write data during configuration and burst commands
#define SPI_SCK_IMU  A0
#define SPI_MISO_IMU A6
#define SPI_MOSI_IMU A7

#define A_CS_PIN   10
#define A_DRDY_PIN 9
#define B_CS_PIN   8
#define B_DRDY_PIN 7
#define C_CS_PIN   6
#define C_DRDY_PIN 5

#define D_CS_PIN   4
#define D_DRDY_PIN 3
#define E_CS_PIN   2
#define E_DRDY_PIN A1
#define F_CS_PIN   A2
#define F_DRDY_PIN A3

// #define A_CS_PIN   4
// #define A_DRDY_PIN 3
// #define B_CS_PIN   2
// #define B_DRDY_PIN A1
// #define C_CS_PIN   A2
// #define C_DRDY_PIN A3

// #define D_CS_PIN   10
// #define D_DRDY_PIN 9
// #define E_CS_PIN   8
// #define E_DRDY_PIN 7
// #define F_CS_PIN   6
// #define F_DRDY_PIN 5

#define IMU_CS_PIN  A4                    // GPIO A4 drives the active-low CS pin of the ISM330DHCX only; held high except during its own SPI transactions so the shared bus remains collision-free
#define MAG_CS_PIN  A5                    // GPIO A5 drives the active-low CS pin of the LIS3MDL only; independent from IMU_CS_PIN so either device can be selected while the other stays deselected

#define LED_PIN     RX                    // GPIO44 (RX pin) drives the on-board LED; which keeps timing to sync with video frames for 3rd party cameras; works because that UART bus is never used in this sketch

// ========== FUNCTION PROTOTYPES ==========
void initializeADCs();
void enableADCInterrupts();
void disableADCInterrupts();

uint8_t readRegister(int csPin, uint8_t regAddr);
void writeRegister(int csPin, uint8_t regAddr, uint8_t value);

// ========== SPI SETTINGS ==========
// Max 8 MHz for IMU/MAG
// Max 6 MHz for ICB (ADS1220)
const SPISettings sensorSPI_ICB(1000000, MSBFIRST, SPI_MODE1);
const SPISettings sensorSPI_IMU_MAG(1000000, MSBFIRST, SPI_MODE3);

extern SPIClass SPI;
SPIClass spiIMU(HSPI);

// ========== USER TUNABLE GLOBAL CONSTANTS ==========
const int MAX_SENSORS = 6;     // Maximum possible sensors (A to E)
const int NUM_SENSORS = 3;     // Set to 1-5 to use the first N sensors from all_configs below.
const uint8_t dr_code = DR_600SPS;  // Data Rate value. In turbo, value is for pairs/sec. In normal, value is for samples/sec
const uint16_t TARGET_ICB_RATE_HZ = 600;

const uint16_t TARGET_IMU_RATE_HZ = 3000;   // IMU (accel + gyro). Supported HP rates: 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
const uint16_t TARGET_MAG_RATE_HZ = 1000;   // MAG. Supported with FAST_ODR: 155 (UHP), 300 (HP), 560 (MP), 1000 (LP) Hz
const uint32_t IMU_INTERVAL_US = 1000000UL / TARGET_IMU_RATE_HZ;
const uint32_t MAG_INTERVAL_US = 1000000UL / TARGET_MAG_RATE_HZ;
const uint32_t MAG_EVERY_N_IMU_SAMPLES = TARGET_IMU_RATE_HZ / TARGET_MAG_RATE_HZ;
const uint8_t  TARGET_ACCEL_FS_G   = 8;     // Options: 2, 4, 8, 16
const uint16_t TARGET_GYRO_FS_DPS  = 2000;  // Options: 125, 250, 500, 1000, 2000, 4000
const uint8_t  TARGET_MAG_FS_GAUSS = 12;    // Options: 4, 8, 12, 16

const uint32_t LED_FLASH_PERIOD_US = 3000000UL;  // 3 seconds
const uint32_t LED_ON_DURATION_US  = 100000UL;    // 100 ms visible flash

volatile bool g_led_on = false;
uint32_t last_led_toggle_us = 0;

// ========== WiFi Configuration and Connection State Machine ==========
const char* ssid = "Hi-STIFFS_Host";                  // Hard-coded SSID of the isolated access point created by the host-side WiFiDataServer; Nano ESP32 joins as station.
const char* password = "BYUCropBio";                  // Pre-shared WPA2 key for the dedicated Hi-STIFFS_Host network.
const char* host_ip = "192.168.137.1";                // Static IPv4 of the Python host running the single shared WiFiDataServer; every Nano targets this endpoint so the server can demux incoming frames by the leading nano_id byte
const int host_port = 8080;                           // TCP listening port of WiFiDataServer; chosen above the privileged range for cross-platform (Win/Linux/RPi5) compatibility while keeping connection setup fast
const uint8_t NANO_ID = 2;                           // ASCII string form of this probe’s unique 2-digit identifier; atoi()’d at packet-build time and placed as the very first payload byte so the host routes data to the correct DataReceiverWriter instance
const unsigned long BATCH_SEND_INTERVAL_MS = 50;      // Period of batch_timer esp_timer; NetworkTask wakes every 50 ms to snapshot and transmit the circular buffer
const unsigned long WiFi_CHECK_INTERVAL_MS = 2000;    // Health-check cadence inside NetworkTask’s CONNECTED state; 5 s is infrequent enough to avoid stealing cycles from the high-priority SamplingTask yet fast enough to detect and recover from transient field WiFi drops before pose data gaps appear
const unsigned long WiFi_RETRY_DELAY_MS = 5000;       // Minimum back-off between connectToHost() attempts; prevents CPU spin during AP association or TCP failures and protects the deterministic IMU sampling path on core 1
enum State_WiFi {                                     // Minimal two-state FSM owned exclusively by NetworkTask; keeps all blocking WiFi/TCP work off the real-time SamplingTask so IMU/MAG timing remains jitter-free
  CONNECTING,                                         // Transitional state entered on boot or after disconnect; SamplingTask sees g_isConnected == false and skips enqueueing packets
  CONNECTED,                                          // Steady-state operating mode; persistent TCP socket is live, batches are sent every 50 ms, and monitorConnection() periodically validates link health
};
State_WiFi WiFiState = CONNECTING;                    // Global FSM variable initialized to CONNECTING so NetworkTask immediately begins association on power-up; updated only inside NetworkTask for thread safety
WiFiClient client;                                    // Persistent TCP client object from the ESP32 WiFi stack; kept open for the entire sketch lifetime with setNoDelay(true)

// ========== ICB Sensor Global Config ==========
struct SensorConfig_ICB {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int cs_pin;                  // Chip Select pin
  int drdy_pin;                // Data Ready pin
};

SensorConfig_ICB all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN},
  {'F', F_CS_PIN, F_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];             // ADC objects from library
// Single contiguous array. 10 bytes per sensor:
//   [0..3] little-endian uint32_t timestamp_us
//   [4..6] raw 3-byte channel 1 (exact bytes from ADS1220)
//   [7..9] raw 3-byte channel 2 (exact bytes from ADS1220)
const size_t ICB_DATA_SIZE = sizeof(uint32_t) + 2*(3*sizeof(uint8_t));
uint8_t ICBs_Data[NUM_SENSORS*ICB_DATA_SIZE];

uint8_t current_channels[MAX_SENSORS] = {0};        // Indicates which MUX channel to read from for an ADS1220 module
volatile uint8_t ready_mask = 0;                    // Bitmask tracking ready sensors (bit i set to (1) when sensor i pair is complete)
static uint8_t config0_shadow[MAX_SENSORS] = {0};

// ========== Binary Packet Queue and Sizing ==========
uint64_t time_init = 0;                                   // µs epoch captured exactly when TCP connects; every packet timestamp is relative to this

const size_t HEADER_SIZE_ICB = 3;                         // Fixed 3-byte prefix: uint8 payload length + uint16 CRC16 (little-endian)
const size_t PAYLOAD_SIZE_ICB = 1 + NUM_SENSORS * ICB_DATA_SIZE; // nano_id (1 B) + NUM_SENSORS*(timestamp_us (4 B) + 2×int32 (raw1, raw2))
const size_t PACKET_SIZE_ICB = HEADER_SIZE_ICB + PAYLOAD_SIZE_ICB;
constexpr size_t MAX_PACKETS_ICB = TARGET_ICB_RATE_HZ * BATCH_SEND_INTERVAL_MS / 1000 * 20;  // Ring depth sized for ~20 batches of headroom at target rate
uint8_t packet_pool_ICB[MAX_PACKETS_ICB][PACKET_SIZE_ICB]; // Zero-heap circular buffer; each slot holds one complete ready-to-send packet
size_t q_ICB_head = 0;
size_t q_ICB_tail = 0;
size_t q_ICB_count = 0;

const size_t HEADER_SIZE_IMU   = 3;                       // Fixed 3-byte prefix: uint8 payload length + uint16 CRC16 (little-endian)
const size_t PAYLOAD_SIZE_IMU  = 1 + sizeof(uint32_t) + 9 * sizeof(int16_t);  // nano_id (1 B) + timestamp_us (4 B) + 9×int16 (gx gy gz ax ay az mx my mz)
const size_t PACKET_SIZE_IMU = HEADER_SIZE_IMU + PAYLOAD_SIZE_IMU;            // Total wire size = 26 bytes per high-rate IMU/MAG sample
constexpr size_t MAX_PACKETS_IMU = TARGET_IMU_RATE_HZ * BATCH_SEND_INTERVAL_MS / 1000 * 20;  // Ring depth sized for ~20 batches of headroom at target rate
uint8_t  packet_pool_IMU[MAX_PACKETS_IMU][PACKET_SIZE_IMU];   // Zero-heap circular buffer; each slot holds one complete ready-to-send 27-byte packet
size_t   q_IMU_head  = 0;                                     // Producer write index (SamplingTask enqueues here)
size_t   q_IMU_tail  = 0;                                     // Consumer read index (NetworkTask dequeues here)
size_t   q_IMU_count = 0;                                     // Current number of valid packets in the ring (mutex-protected)

static uint16_t crc_table[256];                           // Table of all possible crc values, so packet writing can look it up rather than computing each time

// ========== RTOS Objects and Shared State ==========
// Decision: mutex for buffer + lightweight flag to avoid lock in hottest path (priority inheritance, negligible overhead)
SemaphoreHandle_t bufferMutex_ICB = NULL;
SemaphoreHandle_t bufferMutex_IMU = NULL;             // Protects circular buffer indices during enqueue (SamplingTask) and snapshot/commit (NetworkTask)
SemaphoreHandle_t batchSemaphore = NULL;              // Binary semaphore woken by batch_timer every 50 ms to trigger NetworkTask packet transmission
esp_timer_handle_t batch_timer = NULL;                // Periodic esp_timer providing the batch rhythm
SemaphoreHandle_t samplingSemaphore_IMU = NULL;       // Binary semaphore driven by samplingTimer_IMU at precise IMU rate (fewer wake-ups than software timers)
esp_timer_handle_t samplingTimer_IMU = NULL;          // High-resolution esp_timer waking SamplingTask with deterministic hardware timing
volatile bool g_isConnected = false;                  // Lock-free flag read by SamplingTask; written only by NetworkTask on connect/disconnect transitions
bool g_use_icb_sensors = false;
bool hasSerial = false;                               // Set after Serial.begin succeeds; used to safely gate all debug prints
bool imu_mag_present = false;

// ========== WIFI FUNCTIONS ANG HELPERS ==========

void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();

  // Rate-limit attempts with static backoff to avoid spinning CPU
  // or hammering the AP when it is temporarily unavailable.
  if (now - lastRetry < WiFi_RETRY_DELAY_MS) return;

  lastRetry = now;

  if (hasSerial) Serial.print("\nConnecting to "); Serial.println(ssid);

  // Only start a new WiFi association if we are not already connected.
  // Calling WiFi.begin() while already connected is what was causing
  // long stalls and watchdog resets.
  if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);

      unsigned long startAttempt = millis();
      // Poll with short yields instead of one long 8 s block.
      // This keeps NetworkTask responsive and feeds the watchdog.
      while (WiFi.status() != WL_CONNECTED &&
              (millis() - startAttempt < WiFi_RETRY_DELAY_MS)) {
          vTaskDelay(pdMS_TO_TICKS(100));          // better than delay() in a FreeRTOS task
          // esp_task_wdt_reset();                    // explicitly feed watchdog
          if (hasSerial) Serial.print(".");
      }
  }

  if (WiFi.status() == WL_CONNECTED) {
      if (hasSerial) {
          Serial.println("\nWiFi connected");
          Serial.print("IP: "); Serial.println(WiFi.localIP());
          Serial.println("Trying persistent TCP to host...");
      }

      // Try TCP
      if (client.connect(host_ip, host_port)) {
          client.setNoDelay(true);
          WiFiState = CONNECTED;
          g_isConnected = true;

          // We are now safely connected again → re-initialise ADCs + timestamps
          // and re-enable the ICB interrupts we disabled while reconnecting.
          if (g_use_icb_sensors){
            initializeADCs();
            enableADCInterrupts();
            ready_mask = 0;
            q_ICB_head = q_ICB_tail = q_ICB_count = 0;
          }
          q_IMU_head = q_IMU_tail = q_IMU_count = 0;
          time_init = esp_timer_get_time();

          

          if (hasSerial) Serial.println("TCP connected. Entering CONNECTED state.");
      } else {
          if (hasSerial) Serial.println("TCP connect failed, will retry...");
          // stay in CONNECTING
      }
  } else {
      if (hasSerial) Serial.println("\nWiFi association failed, backing off...");
  }
}

void monitorConnection() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();

  if (now - lastCheck < WiFi_CHECK_INTERVAL_MS) return;
  lastCheck = now;

  if (WiFi.status() != WL_CONNECTED || !client.connected()) {
    client.stop();
    disableADCInterrupts();
    WiFiState = CONNECTING;
    g_isConnected = false;
    if (hasSerial) Serial.println("WiFi or TCP disconnected. Returning to CONNECTING state.");
  }
}

void init_crc_table() {
  for (int i = 0; i < 256; i++) {
    uint16_t crc = i;
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
    crc_table[i] = crc;
  }
}

uint16_t compute_crc_fast(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

// ========== ICB FUNCTIONS ==========

// === Interrupt Helpers ===
// Fast read ADS1220 chips and switch multiplexer
void IRAM_ATTR readADS1220(int idx, int cs_pin, uint8_t* buffer) {
  // Serial.println("Read ICB");
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(sensorSPI_ICB);       // Get SPI open with settings
  digitalWrite(cs_pin, LOW);              // Select particular sensor (cs='chip select')
  // delayMicroseconds(10);

  // Retrieve 3 byte (24-bit) ADC result    
  // SPI.transfer(buffer, 3); is not stable for reading from this chip        
  buffer[0] = SPI.transfer(0);
  buffer[1] = SPI.transfer(0);
  buffer[2] = SPI.transfer(0); 

  // Change multiplexer channels
  SPI.transfer(ICB_WREG | (CONFIG_REG0_ADDRESS << 2));                 // WREG command to address 0 (CONFIG_REG0)
  if (current_channels[idx] == 0) {
    SPI.transfer((config0_shadow[0] & ~REG_CONFIG0_MUX_MASK) | MUX_AIN2_AIN3);
  } 
  else {
    SPI.transfer((config0_shadow[0] & ~REG_CONFIG0_MUX_MASK) | MUX_AIN0_AIN1);
  }

  // Close SPI
  digitalWrite(cs_pin, HIGH);             // Release sensor from SPI
  // delayMicroseconds(10);
  SPI.endTransaction();                     // Release Nano's SPI bus
}

// Fast direct data write to sensor's individual data array
void IRAM_ATTR storeData(int idx, uint32_t time, uint8_t* buffer) {
  const size_t byte_offset = (size_t)idx * ICB_DATA_SIZE; // move forward in the binary data array by this number of bytes

  // ADS1220 can only store one value, so manually switch and track between channels
  if (current_channels[idx] == 0) {
    ICBs_Data[byte_offset+4] = buffer[0];
    ICBs_Data[byte_offset+5] = buffer[1];
    ICBs_Data[byte_offset+6] = buffer[2];

    current_channels[idx] = 1;
  } 
  else {
    ICBs_Data[byte_offset+7] = buffer[0];
    ICBs_Data[byte_offset+8] = buffer[1];
    ICBs_Data[byte_offset+9] = buffer[2];

    ICBs_Data[byte_offset + 0] = (uint8_t)(time & 0xFF);
    ICBs_Data[byte_offset + 1] = (uint8_t)((time >>  8) & 0xFF);
    ICBs_Data[byte_offset + 2] = (uint8_t)((time >> 16) & 0xFF);
    ICBs_Data[byte_offset + 3] = (uint8_t)((time >> 24) & 0xFF);

    current_channels[idx] = 0;

    ready_mask |= (1 << idx);
  }
}
// === Interrupt Service Routines ===

void IRAM_ATTR handleDrdyA() {
  const int i = 0;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, A_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

void IRAM_ATTR handleDrdyB() {
  const int i = 1;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, B_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

void IRAM_ATTR handleDrdyC() {
  const int i = 2;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, C_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

void IRAM_ATTR handleDrdyD() {
  const int i = 3;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, D_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

void IRAM_ATTR handleDrdyE() {
  const int i = 4;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, E_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

void IRAM_ATTR handleDrdyF() {
  const int i = 5;                                                      // Array index of sensor
  uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init); // Record the time the ADC value was reported by DRDY interrupt pin. 
  uint8_t SPI_Buf[3];                                                   // temporary local buffer to work with raw result before storing integer value

  readADS1220(i, F_CS_PIN, SPI_Buf);    // Get data off the chip and switch multiplexer channel in one SPI transaction
  storeData(i, timestamp_us, SPI_Buf);  // Store data to memory for later queuing
}

// Broadcast a command to all active sensors
void broadcast_command(uint8_t cmd) {
  // Lower all CS pins for the active sensors to broadcast the command to all chips.
  // We loop only over NUM_SENSORS to avoid affecting unused pins.
  SPI.beginTransaction(sensorSPI_ICB);
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, LOW);
  }
  SPI.transfer(cmd);  // Send the command to all sensors at once
  // Raise all CS pins for active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, HIGH);
  }
  SPI.endTransaction();
}

// Disable interrupts for sensor DRDY pins
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
  }
}

// Enable interrupts for sensor DRDY pinsready_mask
void enableADCInterrupts() {
  void (*isrHandlers[MAX_SENSORS])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE, handleDrdyF};
  for (int i = 0; i < NUM_SENSORS; i++) {
    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isrHandlers[i], FALLING);
  }
}

// Reset and reconfigure all ADCs, reset time_init
void initializeADCs() {
  // Broadcast RESET to all active sensors
  broadcast_command(RESET);
  delay(10);  // Short delay for reset to take effect (per datasheet)

  // Detach interrupts before re-attaching
  disableADCInterrupts();

  // ADS1220 initialization
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
    size_t byte_offset = (size_t)i*ICB_DATA_SIZE;
    memset(&ICBs_Data[byte_offset], 0, ICB_DATA_SIZE);
    // Read back Config Reg 0 that the library just wrote during setup
    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(all_configs[i].cs_pin, LOW);
    delayMicroseconds(1);
    SPI.transfer(ICB_RREG | (CONFIG_REG0_ADDRESS << 2));           // RREG to address 0
    config0_shadow[i] = SPI.transfer(SPI_MASTER_DUMMY);
    delayMicroseconds(1);
    digitalWrite(all_configs[i].cs_pin, HIGH);
    SPI.endTransaction();

    if (hasSerial) {
      Serial.print("Setup complete for Sensor [enumerated as: "); Serial.print(i); Serial.println("]");
      Serial.print("CS: "); Serial.print(all_configs[i].cs_pin); 
      Serial.print(" DRDY: "); Serial.println(all_configs[i].drdy_pin);
    }
    delay(100);
  }
  if (hasSerial) Serial.println("Starting datastream from sensors");

  // Allow settling
  delay(100);               

  // Staggered START: Evenly space within one conversion cycle
  uint16_t turbo_sps = 0;
  switch (dr_code) {
    case DR_20SPS: turbo_sps = 40; break;
    case DR_45SPS: turbo_sps = 90; break;
    case DR_90SPS: turbo_sps = 180; break;
    case DR_175SPS: turbo_sps = 350; break;
    case DR_330SPS: turbo_sps = 660; break;
    case DR_600SPS: turbo_sps = 1200; break;
    case DR_1000SPS: turbo_sps = 2000; break;
    default: 
      if (hasSerial) Serial.println("Invalid DR (data rate) code; using default 180 SPS");
      turbo_sps = 180;
  }
  unsigned long period_us = 1000000UL / turbo_sps;  // Conversion period in µs
  unsigned long stagger_us = period_us / NUM_SENSORS;  // time delay for n sensors

  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS - 1) {
      delayMicroseconds(stagger_us);
    }
  }

  // Set time=0 for datastream after letting ADS1220 modules stabilize
  delay(50);
  enableADCInterrupts();
}

// Check if all data pairs are ready for this cycle
bool checkICB_AllDataReady() {
  uint8_t all_ready_mask = (1U << NUM_SENSORS) - 1;  // Local compile-time constant. Stored in CPU stack for compare, not created in and read from RAM.
  if (ready_mask == all_ready_mask) {
    ready_mask = 0;   // reset all bits in the mask to 0
    return true;
  }
  return false;
}

// ====================== queueDataPacket_ICB  ======================
// Initializes single packet array, builds payload, computes CRC over payload, builds header
// Then loads completed single packet into data queue (global circular buffer array) under mutex lock
// TODO - Finish adjusting packet construstion to match ICB payload given NUM_SENSORS. Also use correct bufferMutex
void queueDataPacket_ICB() {
    if (!g_isConnected) return;

    uint8_t packet[PACKET_SIZE_ICB];
    uint8_t* p = packet + HEADER_SIZE_ICB;  // point past the 3-byte header we will fill later

    // Nano ID (uint8) — host uses this to route to the correct DataReceiverWriter
    *p++ = NANO_ID;

    // Per-sensor: uint32_t timestamp_us (relative), int32_t raw_ch1, int32_t raw_ch2
    // We write the already sign-extended int32_t values that the ISRs put into raw_values[][].
    // for (int i=0; i<NUM_SENSORS; i++) {
    //   // Timestamp captured in handleDrdy* when the second channel of the pair became ready
    //   *(uint32_t*)p = (uint32_t)timestamps[i];
    //   p += sizeof(uint32_t);

    //   // Channel 1 (AIN0-AIN1)
    //   *(uint32_t*)p = (uint32_t)raw_values[i][0];
    //   p += sizeof(uint32_t);

    //   // Channel 2 (AIN2-AIN3)
    //   *(uint32_t*)p = (uint32_t)raw_values[i][1];
    //   p += sizeof(uint32_t);
    // }

    // Copy the entire pre-built binary block for all sensors in one shot.
    // ICBs_Data[] already contains, for every sensor:
    //   4-byte little-endian timestamp + 3-byte ch1 raw + 3-byte ch2 raw
    memcpy(p, ICBs_Data, NUM_SENSORS * ICB_DATA_SIZE);
    p += NUM_SENSORS * ICB_DATA_SIZE;

    // CRC is computed ONLY over the payload (nano_id + all sensor blocks)
    uint16_t crc = compute_crc_fast(packet + HEADER_SIZE_ICB, PAYLOAD_SIZE_ICB);

    // Header (payload length + CRC16 little-endian)
    packet[0] = static_cast<uint8_t>(PAYLOAD_SIZE_ICB);
    packet[1] = crc & 0xFF;
    packet[2] = (crc >> 8) & 0xFF;

    // Enqueue under mutex (critical section kept minimal)
    if (xSemaphoreTake(bufferMutex_ICB, portMAX_DELAY) == pdTRUE) {
        if (q_ICB_count >= MAX_PACKETS_ICB) {
            q_ICB_tail = (q_ICB_tail + 1) % MAX_PACKETS_ICB;
            q_ICB_count--;
        }
        memcpy(packet_pool_ICB[q_ICB_head], packet, PACKET_SIZE_ICB);
        q_ICB_head = (q_ICB_head + 1) % MAX_PACKETS_ICB;
        q_ICB_count++;
        xSemaphoreGive(bufferMutex_ICB);
    }
}

// ====================== ADS1220 / ICB PRESENCE DETECTION ======================
// ADS1220 has no WHO_AM_I register. We use the standard robust detection method
// for this chip family:
//   1. Send RESET (0x06) → restores all registers to POR defaults.
//   2. Read CONFIG_REG0 (addr 0). Expected value after reset = 0x00.
//   3. 0xFF on MISO almost always means the slave is not driving the line
//      (no chip present or CS wiring fault).
// The check is completely isolated by the per-sensor CS pins and uses the
// correct sensorSPI_ICB settings so it cannot collide with the 8 MHz MODE3
// IMU/MAG transactions.

bool detectADS1220(int csPin) {
    const uint8_t TEST_REG = 0x01;        // CONFIG_REG1 — safe to temporarily modify
    const uint8_t TEST_PATTERN = 0x55;    // Arbitrary pattern unlikely to be default

    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);
    SPI.transfer(0x06);                   // RESET
    digitalWrite(csPin, HIGH);
    delayMicroseconds(10);
    SPI.endTransaction();

    delay(3000);

    // --- Step 1: Read current value so we can restore it later ---
    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);
    SPI.transfer(ICB_RREG | (TEST_REG << 2) | 0);
    uint8_t original_val = SPI.transfer(SPI_MASTER_DUMMY);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(10);
    SPI.endTransaction();

    // --- Step 2: Write test pattern ---
    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);
    SPI.transfer(ICB_WREG | (TEST_REG << 2) | 0);   // WREG starting at TEST_REG, 1 register
    SPI.transfer(TEST_PATTERN);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(10);
    SPI.endTransaction();

    // --- Step 3: Read it back ---
    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);
    SPI.transfer(ICB_RREG | (TEST_REG << 2) | 0);
    uint8_t readback = SPI.transfer(SPI_MASTER_DUMMY);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(10);
    SPI.endTransaction();

    // --- Step 4: Restore original value (best practice) ---
    SPI.beginTransaction(sensorSPI_ICB);
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);
    SPI.transfer(ICB_WREG | (TEST_REG << 2) | 0);
    SPI.transfer(original_val);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(10);
    SPI.endTransaction();

    // Success only if the chip echoed back exactly what we wrote
    bool detected = (readback == TEST_PATTERN);

    if (!detected) {
        // Optional: also check if we at least got something other than obvious bus noise
        // (helps distinguish "no chip" from "very noisy bus")
    }

    // Serial.println("Forced return true.");
    // return true;
    return detected;
}

// ====================== bringup_ICB_sensors ======================
// Called once in setup(). Returns true only if EVERY configured sensor
// (A.. up to NUM_SENSORS) responds correctly. This keeps packet formatting
// and the ready_mask logic consistent.
bool bringup_ICB_sensors() {
    Serial.print("Detecting ADS1220 ICB sensors (");
    Serial.print(NUM_SENSORS);
    Serial.println(" expected)...");

    bool all_present = true;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int cs = all_configs[i].cs_pin;
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);           // ensure deselected
        delayMicroseconds(10);

        bool present = detectADS1220(cs);
        digitalWrite(cs, HIGH);           // ensure deselected
        delayMicroseconds(10);

        if (present) {
            Serial.printf("  ADS1220 '%c' (CS=%d) : OK\n",
                          all_configs[i].id, cs);
        } else {
            Serial.printf("  ADS1220 '%c' (CS=%d) : NOT DETECTED (MISO returned 0xFF)\n",
                          all_configs[i].id, cs);
            all_present = false;
        }
    }

    if (all_present) {
        Serial.println("All ICB sensors detected — ICB data path ENABLED.");
    } else {
        Serial.println("One or more ADS1220 chips missing — ICB data path DISABLED for safety.");
    }
    return all_present;
}

// ====================== Output.Data.Rate AND Full.Scale.-RANGE SELECTION HELPERS (DATASHEET DRIVEN) ======================

/*
 * Selects the closest supported high-performance ODR for ISM330DHCX (rounded UP).
 * Reference: IMU datasheet Table 43 (CTRL1_XL) and Table 46 (CTRL2_G).
 * We only use the high-performance column here because pose tracking
 * benefits from lowest noise + highest rate.
 */
uint8_t selectIMU_ODR_Bits(uint16_t target_hz) {
  // Supported HP ODRs (ascending): 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
  if (target_hz <=   12) return 0b0001; // 12.5 Hz
  if (target_hz <=   26) return 0b0010; // 26 Hz
  if (target_hz <=   52) return 0b0011; // 52 Hz
  if (target_hz <=  104) return 0b0100; // 104 Hz
  if (target_hz <=  208) return 0b0101; // 208 Hz
  if (target_hz <=  416) return 0b0110; // 416 Hz
  if (target_hz <=  833) return 0b0111; // 833 Hz   ← e.g. 800 Hz target lands here
  if (target_hz <= 1660) return 0b1000; // 1.66 kHz
  if (target_hz <= 3330) return 0b1001; // 3.33 kHz
  return 0b1010;                        // 6.66 kHz (max) — anything higher also uses max
}

/*
 * Selects MAG operating mode + FAST_ODR bits for LIS3MDL (rounded UP).
 * Reference: MAG datasheet Table 19 (when FAST_ODR=1) and Table 21.
 * We prefer the FAST_ODR path for highest possible rates.
 * OM[1:0] controls X/Y performance; we also set matching OMZ in CTRL_REG4.
 */

uint8_t selectMAG_CTRL_REG1_Val(uint16_t target_hz) {
  // Supported rates with FAST_ODR=1 (ascending): 155, 300, 560, 1000 Hz
  if (target_hz <= 155) return 0b01100010; // UHP + FAST_ODR → 155 Hz
  if (target_hz <= 300) return 0b01000010; // HP  + FAST_ODR → 300 Hz
  if (target_hz <= 560) return 0b00100010; // MP  + FAST_ODR → 560 Hz
  return 0b00000010;                       // LP  + FAST_ODR → 1000 Hz (max)
  // Any target > 1000 Hz also safely lands on 1000 Hz (chip maximum)
}

/*
 * Returns the OMZ[1:0] bits for CTRL_REG4 that match the X/Y performance
 * chosen above (for consistent noise/ODR on all three axes).
 */
uint8_t selectMAG_OMZ_Bits(uint16_t target_hz) {
  if (target_hz <= 155) return 0b11; // UHP
  if (target_hz <= 300) return 0b10; // HP
  if (target_hz <= 560) return 0b01; // MP
  return 0b00;                       // LP (for 1000 Hz and above)
}

/*
 * Returns the FS[1:0] bits for CTRL1_XL.
 * Picks the smallest supported accelerometer range >= target (ceiling).
 * Reference: ISM330DHCX datasheet Table 42 (CTRL1_XL register).
 */
uint8_t selectAccelFS_Bits(uint8_t target_g) {
  if (target_g <= 2)  return 0b00; // ±2 g
  if (target_g <= 4)  return 0b10; // ±4 g
  if (target_g <= 8)  return 0b11; // ±8 g
  return 0b01;                     // ±16 g (max)
}

/*
 * Returns the combined FS bits for CTRL2_G (FS[1:0], FS_125, FS_4000).
 * Picks the smallest supported gyroscope range >= target (ceiling).
 * Reference: ISM330DHCX datasheet Table 45 (CTRL2_G register).
 */
uint8_t selectGyroFS_Bits(uint16_t target_dps) {
  if (target_dps <= 125)  return 0b00000001; // ±125 dps  (FS_125 = 1)
  if (target_dps <= 250)  return 0b00000000; // ±250 dps
  if (target_dps <= 500)  return 0b00000100; // ±500 dps
  if (target_dps <= 1000) return 0b00001000; // ±1000 dps
  if (target_dps <= 2000) return 0b00001100; // ±2000 dps
  return 0b00001100 | 0b00000010;            // ±4000 dps (FS_4000 = 1)
}

/*
 * Returns the FS[1:0] bits for LIS3MDL CTRL_REG2 (0x21).
 * Picks the smallest supported magnetic range >= target (ceiling).
 * Reference: LIS3MDL datasheet Table 22 (CTRL_REG2 register).
 */
uint8_t selectMagFS_Bits(uint8_t target_gauss) {
  if (target_gauss <= 4)  return 0b00000000; // ±4 gauss
  if (target_gauss <= 8)  return 0b00100000; // ±8 gauss  (bit 5)
  if (target_gauss <= 12) return 0b01000000; // ±12 gauss (bit 6)
  return 0b01100000;                         // ±16 gauss (bits 6:5)
}

// ====================== CHIP BRING-UP ======================

bool bringup_IMU() {
  Serial.print("Checking ISM330DHCX WHO_AM_I ... ");

  uint8_t whoami_val = 0;
  bool id_ok = false;

  // Up to 11 total reads (1 initial + 10 retries). Delay only between attempts.
  // Single read site + early exit keeps the bringup path simple and reliable.
  for (int attempt = 0; attempt < 11; ++attempt) {
      whoami_val = readRegister(IMU_CS_PIN, IMU_WHO_AM_I);
      if (whoami_val == IMU_WHO_AM_I_VAL) {
          id_ok = true;
          break;
      }
      if (attempt < 10) {
          delay(50);
      }
  }

  if (!id_ok) {
    Serial.println("FAILED (expected 0x6B)");
    Serial.printf("Got: 0b%08b / 0x%02X\n", whoami_val, whoami_val);
    Serial.println("Continuing without IMU/MAG functionality.");
    return false;
  }

  Serial.println("OK (0x6B)");

  // Calculate best ODR bits from user target (rounded up)
  uint8_t odr_bits = selectIMU_ODR_Bits(TARGET_IMU_RATE_HZ);

  // === Full-scale range selection (configurable) ===
  uint8_t accel_fs_bits = selectAccelFS_Bits(TARGET_ACCEL_FS_G);
  uint8_t gyro_fs_bits  = selectGyroFS_Bits(TARGET_GYRO_FS_DPS);

  // Build CTRL1_XL: ODR[7:4] | FS[3:2] | LPF2_EN=0 | bit0=0
  uint8_t ctrl1_val = (odr_bits << 4) | (accel_fs_bits << 2);

  // Build CTRL2_G: ODR[7:4] | FS bits (includes FS_125 / FS_4000)
  uint8_t ctrl2_val = (odr_bits << 4) | gyro_fs_bits;

  // Apply configuration (order matters on some sensors)
  writeRegister(IMU_CS_PIN, IMU_CTRL3_C,   IMU_CTRL3_C_VAL); // BDU + auto-inc
  writeRegister(IMU_CS_PIN, IMU_CTRL1_XL,  ctrl1_val);
  writeRegister(IMU_CS_PIN, IMU_CTRL2_G,   ctrl2_val);
  writeRegister(IMU_CS_PIN, IMU_INT1_CTRL, 0x01); // we set DRDY but do not use interrupt pin

  Serial.printf("IMU configured: target=%u Hz → actual ODR bits=0x%X (see datasheet Table 43/46)\n",
                TARGET_IMU_RATE_HZ, odr_bits);

  return true;
}

bool bringup_MAG() {
  Serial.print("Checking LIS3MDL WHO_AM_I ... ");

  uint8_t whoami_val = 0;
  bool id_ok = false;

  // Up to 11 total reads (1 initial + 10 retries). Delay only between attempts.
  for (int attempt = 0; attempt < 11; ++attempt) {
      whoami_val = readRegister(MAG_CS_PIN, MAG_WHO_AM_I);
      if (whoami_val == MAG_WHO_AM_I_VAL) {
          id_ok = true;
          break;
      }
      if (attempt < 10) {
          delay(50);
      }
  }

  if (!id_ok) {
    Serial.println("FAILED (expected 0x3D)");
    Serial.printf("Got: 0b%08b / 0x%02X\n", whoami_val, whoami_val);
    Serial.println("Continuing without IMU/MAG functionality.");
    return false;
  }

  Serial.println("OK (0x3D)");

  // Calculate best CTRL_REG1 value from user target
  uint8_t ctrl1_val = selectMAG_CTRL_REG1_Val(TARGET_MAG_RATE_HZ);
  uint8_t omz_bits  = selectMAG_OMZ_Bits(TARGET_MAG_RATE_HZ);

  // CTRL_REG4: OMZ[5:4] | BLE=0 (little endian)
  uint8_t ctrl4_val = (omz_bits << 4) | 0b00000000;

  // === Full-scale range selection for magnetometer ===
  uint8_t mag_fs_bits = selectMagFS_Bits(TARGET_MAG_FS_GAUSS);

  // Standard power-on sequence (keep BDU=0 for continuous update, we handle freshness via STATUS)
  writeRegister(MAG_CS_PIN, MAG_CTRL_REG2, mag_fs_bits);
  writeRegister(MAG_CS_PIN, MAG_CTRL_REG4, ctrl4_val);
  writeRegister(MAG_CS_PIN, MAG_CTRL_REG5, 0x00); // BDU=0, FAST_READ=0
  writeRegister(MAG_CS_PIN, MAG_CTRL_REG1, ctrl1_val);
  writeRegister(MAG_CS_PIN, MAG_CTRL_REG3, 0x00); // continuous conversion

  Serial.printf("MAG configured: target=%u Hz → CTRL_REG1=0x%02X (see datasheet Table 19)\n",
                TARGET_MAG_RATE_HZ, ctrl1_val);

  return true;
}

// ====================== LOW-LEVEL SPI HELPERS ======================

void writeRegister(int csPin, uint8_t regAddr, uint8_t value) {
  spiIMU.beginTransaction(sensorSPI_IMU_MAG);
  digitalWrite(csPin, LOW);
  delayMicroseconds(10);
  spiIMU.transfer(regAddr & 0x7F);   // write
  spiIMU.transfer(value);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(10);
  spiIMU.endTransaction();
}

uint8_t readRegister(int csPin, uint8_t regAddr) {
  spiIMU.beginTransaction(sensorSPI_IMU_MAG);
  digitalWrite(csPin, LOW);
  delay(100);
  spiIMU.transfer(regAddr | 0x80);   // read
  uint8_t value = spiIMU.transfer(0x00);
  digitalWrite(csPin, HIGH);
  spiIMU.endTransaction();
  return value;
}

// === Declarations with attributes (must come before any calls) ===
void IRAM_ATTR readIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                       int16_t &ax, int16_t &ay, int16_t &az) __attribute__((always_inline));

void IRAM_ATTR readMAG(int16_t &mx, int16_t &my, int16_t &mz) __attribute__((always_inline));

// Burst read 12 bytes from IMU (gyro X/Y/Z + accel X/Y/Z)
void IRAM_ATTR readIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                       int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t raw[12];

  spiIMU.beginTransaction(sensorSPI_IMU_MAG);
  digitalWrite(IMU_CS_PIN, LOW);

  spiIMU.transfer(IMU_OUTX_L_G | 0x80);   // read + auto-increment command
  spiIMU.transfer(raw, 12);               // bulk transfer

  digitalWrite(IMU_CS_PIN, HIGH);
  spiIMU.endTransaction();

  // Little-endian assembly
  gx = (int16_t)(raw[0] | (raw[1] << 8));
  gy = (int16_t)(raw[2] | (raw[3] << 8));
  gz = (int16_t)(raw[4] | (raw[5] << 8));
  ax = (int16_t)(raw[6] | (raw[7] << 8));
  ay = (int16_t)(raw[8] | (raw[9] << 8));
  az = (int16_t)(raw[10] | (raw[11] << 8));
}

// Burst read 6 bytes from magnetometer (mag X/Y/Z)
void IRAM_ATTR readMAG(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t raw[6];

  spiIMU.beginTransaction(sensorSPI_IMU_MAG);
  digitalWrite(MAG_CS_PIN, LOW);

  spiIMU.transfer(MAG_OUT_X_L | 0xC0);    // read + auto-increment (LIS3MDL)
  spiIMU.transfer(raw, 6);                // bulk transfer replaces the per-byte loop

  digitalWrite(MAG_CS_PIN, HIGH);
  spiIMU.endTransaction();

  // Little-endian assembly (unchanged)
  mx = (int16_t)(raw[0] | (raw[1] << 8));
  my = (int16_t)(raw[2] | (raw[3] << 8));
  mz = (int16_t)(raw[4] | (raw[5] << 8));
}

// ====================== queueDataPacket_IMU_MAG  ======================
// Initializes single packet array, builds payload, computes CRC over payload, builds header
// Then loads completed single packet into data queue (global circular buffer array) under mutex lock
void queueDataPacket_IMU_MAG(uint32_t timestamp_us,
                             int16_t gx, int16_t gy, int16_t gz,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t mx, int16_t my, int16_t mz) {
    if (!g_isConnected) return;

    uint8_t packet[PACKET_SIZE_IMU];
    uint8_t* p = packet + HEADER_SIZE_IMU;

    // Nano ID
    *p++ = NANO_ID;

    // Timestamp (direct 32-bit write)
    *(uint32_t*)p = timestamp_us;
    p += sizeof(uint32_t);

    // 9 sensor values — direct uint16_t stores
    *(uint16_t*)p = (uint16_t)gx;  p += 2;
    *(uint16_t*)p = (uint16_t)gy;  p += 2;
    *(uint16_t*)p = (uint16_t)gz;  p += 2;
    *(uint16_t*)p = (uint16_t)ax;  p += 2;
    *(uint16_t*)p = (uint16_t)ay;  p += 2;
    *(uint16_t*)p = (uint16_t)az;  p += 2;
    *(uint16_t*)p = (uint16_t)mx;  p += 2;
    *(uint16_t*)p = (uint16_t)my;  p += 2;
    *(uint16_t*)p = (uint16_t)mz;  p += 2;

    // CRC over payload
    uint16_t crc = compute_crc_fast(packet + HEADER_SIZE_IMU, PAYLOAD_SIZE_IMU);

    // Header
    packet[0] = PAYLOAD_SIZE_IMU;
    packet[1] = crc & 0xFF;
    packet[2] = (crc >> 8) & 0xFF;

    // Enqueue under mutex (critical section kept minimal)
    if (xSemaphoreTake(bufferMutex_IMU, portMAX_DELAY) == pdTRUE) {
        if (q_IMU_count >= MAX_PACKETS_IMU) {
            q_IMU_tail = (q_IMU_tail + 1) % MAX_PACKETS_IMU;
            q_IMU_count--;
        }
        memcpy(packet_pool_IMU[q_IMU_head], packet, PACKET_SIZE_IMU);
        q_IMU_head = (q_IMU_head + 1) % MAX_PACKETS_IMU;
        q_IMU_count++;
        xSemaphoreGive(bufferMutex_IMU);
    }
}

// ====================== sendQueuedDataTCP ======================
// Sends as many queued packets as possible over TCP.
// Optimized for minimum execution time while preserving correctness under producer interference.
bool sendQueuedDataTCP() {
    if (!client.connected()) return false;
    // if (q_IMU_count == 0) return true;

    // === 1. Snapshot current buffer indices and optimistic advance indices under lock ===
    size_t snapshot_tail_imu = 0;
    size_t total_packets_imu = 0;
    size_t total_bytes_imu = 0;
    if (xSemaphoreTake(bufferMutex_IMU, portMAX_DELAY) == pdTRUE) {
      // Capture the work we are about to do
      snapshot_tail_imu = q_IMU_tail;
      total_packets_imu = q_IMU_count;
      total_bytes_imu = total_packets_imu * PACKET_SIZE_IMU;

      // Optimistic commit — advance the consumer pointers NOW
      q_IMU_tail = (snapshot_tail_imu + total_packets_imu) % MAX_PACKETS_IMU;
      q_IMU_count -= total_packets_imu;

      xSemaphoreGive(bufferMutex_IMU);
    }

    // Same for ICB sensors
    size_t snapshot_tail_icb = 0;
    size_t total_packets_icb = 0;
    size_t total_bytes_icb = 0;
    if (xSemaphoreTake(bufferMutex_ICB, portMAX_DELAY) == pdTRUE) {
      snapshot_tail_icb = q_ICB_tail;
      total_packets_icb = q_ICB_count;
      total_bytes_icb = total_packets_icb * PACKET_SIZE_ICB;

      q_ICB_tail = (snapshot_tail_icb + total_packets_icb) % MAX_PACKETS_ICB;
      q_ICB_count -= total_packets_icb;

      xSemaphoreGive(bufferMutex_ICB);
    }

    // === 2. Perform TCP write(s) OUTSIDE the mutex ===
    // We may need to split across the ring buffer wrap point.
    size_t first_chunk_imu = min(total_packets_imu, MAX_PACKETS_IMU - snapshot_tail_imu);
    size_t bytes_first_imu = first_chunk_imu * PACKET_SIZE_IMU;
    size_t bytes_second_imu = total_bytes_imu - bytes_first_imu;

    size_t written_imu = client.write(packet_pool_IMU[snapshot_tail_imu], bytes_first_imu);
    if (written_imu != bytes_first_imu) {
      if (hasSerial) Serial.println("Partial IMU write on first segment");
      return false;
    }
    if (bytes_second_imu > 0) {
      written_imu = client.write(packet_pool_IMU[0], bytes_second_imu);   // wrap: start from beginning of pool
      if (written_imu != bytes_second_imu) {
        if (hasSerial) Serial.println("Partial IMU write on wrap segment");
          return false;
      }
    }

    // same for ICB sensors
    size_t first_chunk_icb = min(total_packets_icb, MAX_PACKETS_ICB - snapshot_tail_icb);
    size_t bytes_first_icb = first_chunk_icb * PACKET_SIZE_ICB;
    size_t bytes_second_icb = total_bytes_icb - bytes_first_icb;

    size_t written_icb = client.write(packet_pool_ICB[snapshot_tail_icb], bytes_first_icb);
    if (written_icb != bytes_first_icb) {
      if (hasSerial) Serial.println("Partial ICB write on first segment");
      return false;
    }
    if (bytes_second_icb > 0) {
      written_icb = client.write(packet_pool_ICB[0], bytes_second_icb);   // wrap: start from beginning of pool
      if (written_icb != bytes_second_icb) {
        if (hasSerial) Serial.println("Partial ICB write on wrap segment");
          return false;
      }
    }

    // Occasional flush for very large batches (reduces latency on the wire)
    if (total_packets_imu > 200) {
        client.flush();
    }

    return true;
}

// ====================== BATCH TIMER CALLBACK ) ======================
void IRAM_ATTR batch_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(batchSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ====================== SAMPLING TIMER CALLBACK ======================
void IRAM_ATTR samplingTimer_IMU_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(samplingSemaphore_IMU, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ====================== SAMPLING TASK ======================

void SamplingTask(void *pvParameters) {
    static int16_t latched_mx = 0, latched_my = 0, latched_mz = 0;

    // Simple counter for MAG sub-sampling.
    // We wake at IMU rate (e.g. 3000 Hz). We only read fresh MAG every Nth wake.
    // 3000 / 1000 = 3 → MAG runs at its target rate with almost zero overhead.
    static uint32_t mag_sample_count = 0;

    // Decision: Block on samplingSemaphore_IMU (given by esp_timer ISR).
    // Intent: The hardware timer now *is* the rate source. Task only runs when
    // a real sample is due. Timeout (pdMS) is just a safety net (rarely triggers).
    for (;;) {
        if (xSemaphoreTake(samplingSemaphore_IMU, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (g_isConnected) {
              uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init);   // cast is safe; we only ever send the low 32 bits
              if (imu_mag_present) {
                portDISABLE_INTERRUPTS(); // Prevent ICB DRDY interrupts during this read
                // === IMU sample (always happens at hardware-timed rate) ===

                int16_t gx, gy, gz, ax, ay, az;
                readIMU(gx, gy, gz, ax, ay, az);

                // === MAG handling (simple counter, no software timer) ===
                // Increment first, then check. On the Nth IMU wake we read fresh MAG
                // and reset the counter. All other wakes just reuse the latched values.
                // This completely replaces the old IMU_MAG_timerExpired(last_mag_us, ...) logic.
                mag_sample_count++;
                if (mag_sample_count >= MAG_EVERY_N_IMU_SAMPLES) {
                    readMAG(latched_mx, latched_my, latched_mz);
                    mag_sample_count = 0;
                }
                portENABLE_INTERRUPTS();  // Allow ICB DRDY interrupts as soon as read is over

                // Queue exactly as before (timestamp comes from the IMU sample instant)
                queueDataPacket_IMU_MAG(timestamp_us,
                                        gx, gy, gz, ax, ay, az,
                                        latched_mx, latched_my, latched_mz);
              }

              // === ICB sensors sample
              if (checkICB_AllDataReady() && g_use_icb_sensors) {
                queueDataPacket_ICB();
              }

              // === LED flash logic (runs at IMU rate, extremely cheap) ===

              if (!g_led_on) {
                  if (timestamp_us - last_led_toggle_us >= LED_FLASH_PERIOD_US) {
                      digitalWrite(LED_PIN, HIGH);
                      g_led_on = true;
                      last_led_toggle_us = timestamp_us;
                      // Optional: you can also set a flag that gets packed into the next packet
                      }
                  } else {
                      if (timestamp_us - last_led_toggle_us >= LED_ON_DURATION_US) {
                          digitalWrite(LED_PIN, LOW);
                          g_led_on = false;
                        }
                }
            }
        }
    }
}

// ====================== NETWORK TASK ======================
void NetworkTask(void *pvParameters) {
    unsigned long lastCheck = 0;

    for (;;) {
        switch (WiFiState) {
            case CONNECTING: {
                connectToHost();
                vTaskDelay(pdMS_TO_TICKS(10));   // Only delay in CONNECTING
                break;
            }

            case CONNECTED: {
                // Wait for batch trigger or timeout for monitoring
                if (xSemaphoreTake(batchSemaphore, pdMS_TO_TICKS(200)) == pdTRUE) {
                    sendQueuedDataTCP();
                }

                // Periodic connection health check (non-blocking)
                unsigned long now = millis();
                if (now - lastCheck >= WiFi_CHECK_INTERVAL_MS) {
                    lastCheck = now;
                    monitorConnection();
                }
                break;
            }
        }
    }
}

// ====================== SETUP (creates tasks and supporting objects) ======================
void setup() {
  Serial.begin(1000000);
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 5000)) {}
  hasSerial = Serial;
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  if (hasSerial) {
      Serial.println("\n\nSerial connected for debugging");
      Serial.print("Nano_ID: "); Serial.println(NANO_ID);
      Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
      Serial.println("║   9-DOF High-Rate Binary Polling  —  MULTI-TASK REFACTORED            ║");
      Serial.println("╚═══════════════════════════════════════════════════════════════════════╝");
  }

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  spiIMU.begin(SPI_SCK_IMU, SPI_MISO_IMU, SPI_MOSI_IMU);
  pinMode(IMU_CS_PIN, OUTPUT); digitalWrite(IMU_CS_PIN, HIGH);
  pinMode(MAG_CS_PIN, OUTPUT); digitalWrite(MAG_CS_PIN, HIGH);
  for (int i = 0; i < MAX_SENSORS; i++) {
    pinMode(all_configs[i].cs_pin, OUTPUT);
    digitalWrite(all_configs[i].cs_pin, HIGH);
  }
  init_crc_table();
  delay(100);

  // WiFi Station mode setup
  if (hasSerial) {
      Serial.println("Starting station mode. Arduino is WiFi client looking for following network...");
      Serial.print("SSID: "); Serial.print(ssid);
      Serial.print(". Password: "); Serial.println(password);
      Serial.println("Scanning for available networks...");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  int n = WiFi.scanNetworks();
  Serial.println("WiFi network scan done. Listing networks...");
  if (n == 0) {
      Serial.println("No networks found");
  } else {
      for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      Serial.print(" (");
      Serial.print(WiFi.encryptionType(i), HEX);
      Serial.print(")");
      Serial.println();
      }
  }

  // Initialize various chips
  initializeADCs();
  disableADCInterrupts();
  delay(1000);
  bool icb_ok = bringup_ICB_sensors();
  g_use_icb_sensors = icb_ok;
  delay(1000);
  bool imu_ok = bringup_IMU();
  bool mag_ok = bringup_MAG();
  imu_mag_present = imu_ok && mag_ok;
  // imu_mag_present = true;

  // ====================== CREATE RTOS OBJECTS ======================
  if (hasSerial) Serial.println("\nStarting RTOS tasks...");

  bufferMutex_IMU = xSemaphoreCreateMutex();
  bufferMutex_ICB = xSemaphoreCreateMutex();
  batchSemaphore = xSemaphoreCreateBinary();
  if (hasSerial) Serial.println("\tFinished MUTEX and Semaphore.");

  // Decision: Dedicated periodic esp_timer for batch rhythm (user choice).
  // Intent: Clean separation of timing source from task scheduling.
  esp_timer_create_args_t timer_args = {
      .callback = &batch_timer_callback,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "batch_timer"
  };
  esp_timer_create(&timer_args, &batch_timer);
  esp_timer_start_periodic(batch_timer, BATCH_SEND_INTERVAL_MS * 1000ULL);
  if (hasSerial) Serial.println("\tFinished WiFi batch timer interrupt.");

  // Create high-resolution sampling timer (different approach from vTaskDelayUntil)
  samplingSemaphore_IMU = xSemaphoreCreateBinary();
  esp_timer_create_args_t samplingTimer_IMU_args = {
      .callback = &samplingTimer_IMU_callback,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "samplingTimer_IMU"
  };
  esp_timer_create(&samplingTimer_IMU_args, &samplingTimer_IMU);
  esp_timer_start_periodic(samplingTimer_IMU, IMU_INTERVAL_US);
  if (hasSerial) Serial.println("\tFinished high-resolution sampling timer.");

  // ====================== CREATE TASKS (Decision: pinned + priorities) ======================
  // Decision: SamplingTask pinned to core 1 (high prio), NetworkTask to core 0.
  // Intent: Better WiFi stack affinity on core 0; SamplingTask isolated on core 1.
  
  // SamplingTask — high priority, pinned to core 1 (away from WiFi)
  if (xTaskCreatePinnedToCore(
          SamplingTask,
          "SamplingTask",
          6144,
          NULL,
          configMAX_PRIORITIES - 3,   // Slightly higher priority than before
          NULL,
          1                    ) != pdPASS)
  {
      Serial.println("ERROR: Failed to create SamplingTask");
      while (true) delay(100);
  }

  // NetworkTask — lower priority, pinned to core 0 (WiFi affinity)
  if (xTaskCreatePinnedToCore(
          NetworkTask,
          "NetworkTask",
          6144,
          NULL,
          configMAX_PRIORITIES - 5,
          NULL,
          0                   ) != pdPASS)
  {
      Serial.println("ERROR: Failed to create NetworkTask");
      while (true) delay(100);
  }

  if (hasSerial) Serial.println("Both tasks created. SamplingTask running at target rate.");

  // Original loop() is intentionally left to idle (user decision 1.4)
}

// ====================== LOOP (now idle – replaced by dedicated tasks) ======================
void loop() {
    // Decision: Leave default Arduino loopTask idle.
    // Intent: All real work moved to explicit, priority-controlled FreeRTOS tasks.
    vTaskDelay(portMAX_DELAY);
}