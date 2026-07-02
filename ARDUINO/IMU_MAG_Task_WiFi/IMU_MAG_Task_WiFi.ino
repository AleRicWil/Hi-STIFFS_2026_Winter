/*
 * 9-DOF IMU + MAG High-Rate Binary Streaming Sketch — MULTI-TASK REFACTORED
 * Arduino Nano ESP32 (WiFi client) — ISM330DHCX + LIS3MDL
 * =======================================================================
 *
 * PURPOSE
 * -------
 * Achieve the highest sustainable unique sample rate for 3D pose tracking
 * over rolling 1-2 s windows on the stalk probe. The ISM330DHCX (accel +
 * gyro) drives every packet at TARGET_IMU_RATE_HZ. The LIS3MDL magnetometer
 * is read only at its target rate and the three int16 values are simply
 * latched and reused in all intervening packets. This keeps SPI traffic and
 * CPU load minimal while still supplying fresh 9-DOF vectors at the IMU rate.
 *
 * ARCHITECTURE (FreeRTOS + dedicated esp_timers)
 * ----------------------------------------------
 * SamplingTask (configMAX_PRIORITIES-4, pinned to core 1)
 *   - Woken by hardware sampling_timer ISR at exactly IMU_INTERVAL_US.
 *   - Burst-reads 12 bytes from IMU (always fresh).
 *   - Simple uint32 counter; every Nth wake (TARGET_IMU_RATE_HZ / TARGET_MAG_RATE_HZ)
 *     performs a fresh 6-byte MAG read and updates the three latched int16 values.
 *   - Builds a 28-byte packet and enqueues it under bufferMutex ONLY when
 *     the lightweight flag g_isConnected is true.
 *   - Never blocks on WiFi/TCP — deterministic hot path, zero network jitter.
 *
 * NetworkTask (priority 5, pinned to core 0)
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
 *   - sampling_timer: period = IMU_INTERVAL_US → hardware-precise wake-ups.
 *   - batch_timer: period = BATCH_SEND_INTERVAL_MS * 1000 µs (50 ms) →
 *     wakes NetworkTask to transmit the accumulated batch.
 *
 * Buffer & synchronization
 *   - Zero-heap circular buffer: static packet_pool[MAX_PACKETS_IMU][28].
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
 * PACKET STRUCTURE (little-endian, 27 bytes on the wire)
 * ------------------------------------------------------
 * Offset  Size   Field
 * 0       2      uint16_t payload_length;   // always 24 (0x0018)
 * 2       2      uint16_t crc16;            // CRC-16/ARC (poly 0xA001, init 0xFFFF)
 *                                           //   computed ONLY over bytes 4..27
 * 4       1      uint8_t  nano_id;          // decimal value of NANO_ID
 * 5       4      uint32_t timestamp_us;     // (esp_timer_get_time() - time_init)
 *                                           //   microseconds since TCP connect epoch
 * 9      18     int16_t  data[9];           // gx, gy, gz, ax, ay, az, mx, my, mz
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
 * This is the intended policy: the 1-2 s pose window cares only about the
 * freshest data, not about retransmitting stale samples.
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
#include "esp_timer.h"                    // esp_timer_create + esp_timer_start_periodic used to create the two hardware-timed sources: sampling_timer (IMU_INTERVAL_US) and batch_timer (50 ms)
#include "freertos/FreeRTOS.h"            // BaseType_t, pdTRUE, pdFALSE, portMAX_DELAY, configMAX_PRIORITIES — foundational types for all semaphore and task-priority logic in this sketch
#include "freertos/task.h"                // xTaskCreatePinnedToCore, vTaskDelay, portYIELD_FROM_ISR — creates the two pinned tasks (SamplingTask on core 1, NetworkTask on core 0)
#include "freertos/semphr.h"              // xSemaphoreCreateMutex, xSemaphoreCreateBinary, xSemaphoreTake/Give, xSemaphoreGiveFromISR — protects the circular packet buffer and wakes the two tasks from timer ISRs

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

// ========== PIN DEFINITIONS ==========
#define SPI_SCK     13                    // GPIO13 on Nano ESP32 wired to SCK of both ISM330DHCX and LIS3MDL; driven at 8 MHz by the SPI library for maximum noise margin on field cables
#define SPI_MISO    11                    // GPIO11 wired to MISO/SDO of both sensors; receives the 12-byte IMU or 6-byte MAG burst during every sample inside readIMU() / readMAG()
#define SPI_MOSI    12                    // GPIO12 wired to MOSI/SDI of both sensors; carries the register address byte (with R/W bit) plus any write data during configuration and burst commands
#define IMU_CS_PIN  A4                    // GPIO A4 drives the active-low CS pin of the ISM330DHCX only; held high except during its own SPI transactions so the shared bus remains collision-free
#define MAG_CS_PIN  A5                    // GPIO A5 drives the active-low CS pin of the LIS3MDL only; independent from IMU_CS_PIN so either device can be selected while the other stays deselected

// ========== SPI SETTINGS ==========
// Kept at 8 MHz for maximum long-term stability in field deployments
// (both chips support 10 MHz, but 8 MHz has more margin on long cables / noise).
SPISettings sensorSPI(8000000, MSBFIRST, SPI_MODE3);

// ========== USER TUNABLE GLOBAL CONSTANTS ==========
// Change these lines to control the target output data rates and ranges.
// The bringup functions will automatically select the closest supported
// hardware ODR that is >= the target (rounded up from datasheet tables).
const uint16_t TARGET_IMU_RATE_HZ = 3000;   // IMU (accel + gyro). Supported HP rates: 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
const uint16_t TARGET_MAG_RATE_HZ = 1000;   // MAG. Supported with FAST_ODR: 155 (UHP), 300 (HP), 560 (MP), 1000 (LP) Hz
const uint32_t IMU_INTERVAL_US = 1000000UL / TARGET_IMU_RATE_HZ;
const uint32_t MAG_INTERVAL_US = 1000000UL / TARGET_MAG_RATE_HZ;
const uint8_t  TARGET_ACCEL_FS_G   = 8;     // Options: 2, 4, 8, 16
const uint16_t TARGET_GYRO_FS_DPS  = 2000;  // Options: 125, 250, 500, 1000, 2000, 4000
const uint8_t  TARGET_MAG_FS_GAUSS = 12;    // Options: 4, 8, 12, 16

// ========== WiFi Configuration and Connection State Machine ==========
const char* ssid = "Hi-STIFFS_Host";                  // Hard-coded SSID of the isolated access point created by the host-side WiFiDataServer; Nano ESP32 joins as station.
const char* password = "BYUCropBio";                  // Pre-shared WPA2 key for the dedicated Hi-STIFFS_Host network.
const char* host_ip = "192.168.137.1";                // Static IPv4 of the Python host running the single shared WiFiDataServer; every Nano targets this endpoint so the server can demux incoming frames by the leading nano_id byte
const int host_port = 8080;                           // TCP listening port of WiFiDataServer; chosen above the privileged range for cross-platform (Win/Linux/RPi5) compatibility while keeping connection setup fast
const char* NANO_ID = "01";                           // ASCII string form of this probe’s unique 2-digit identifier; atoi()’d at packet-build time and placed as the very first payload byte so the host routes data to the correct DataReceiverWriter instance
const unsigned long BATCH_SEND_INTERVAL_MS = 50;      // Period of batch_timer esp_timer; NetworkTask wakes every 50 ms to snapshot and transmit the circular buffer
const unsigned long WiFi_CHECK_INTERVAL_MS = 5000;    // Health-check cadence inside NetworkTask’s CONNECTED state; 5 s is infrequent enough to avoid stealing cycles from the high-priority SamplingTask yet fast enough to detect and recover from transient field WiFi drops before pose data gaps appear
const unsigned long WiFi_RETRY_DELAY_MS = 5000;       // Minimum back-off between connectToHost() attempts; prevents CPU spin during AP association or TCP failures and protects the deterministic IMU sampling path on core 1
enum State_WiFi {                                     // Minimal two-state FSM owned exclusively by NetworkTask; keeps all blocking WiFi/TCP work off the real-time SamplingTask so 3 kHz IMU/MAG timing remains jitter-free
  CONNECTING,                                         // Transitional state entered on boot or after disconnect; SamplingTask sees g_isConnected == false and skips enqueueing packets
  CONNECTED,                                          // Steady-state operating mode; persistent TCP socket is live, batches are sent every 50 ms, and monitorConnection() periodically validates link health
};
State_WiFi WiFiState = CONNECTING;                    // Global FSM variable initialized to CONNECTING so NetworkTask immediately begins association on power-up; updated only inside NetworkTask for thread safety
WiFiClient client;                                    // Persistent TCP client object from the ESP32 WiFi stack; kept open for the entire sketch lifetime with setNoDelay(true)

// ========== Binary Packet Queue and Sizing ==========
uint64_t time_init = 0;                                   // µs epoch captured exactly when TCP connects; every packet timestamp is relative to this for clean 1-2 s pose window alignment on the host
const size_t HEADER_SIZE_IMU   = 4;                       // Fixed 4-byte prefix: uint16 payload length + uint16 CRC16 (little-endian)
const size_t PAYLOAD_SIZE_IMU  = 1 + sizeof(uint32_t) + 9 * sizeof(int16_t);  // nano_id (1 B) + timestamp_us (4 B) + 9×int16 (gx gy gz ax ay az mx my mz)
const size_t PACKET_SIZE_IMU = HEADER_SIZE_IMU + PAYLOAD_SIZE_IMU;            // Total wire size = 27 bytes per high-rate IMU/MAG sample
constexpr size_t MAX_PACKETS_IMU = TARGET_IMU_RATE_HZ * BATCH_SEND_INTERVAL_MS / 1000 * 20;  // Ring depth sized for ~20 batches of headroom at 3 kHz target rate
uint8_t  packet_pool[MAX_PACKETS_IMU][PACKET_SIZE_IMU];   // Zero-heap circular buffer; each slot holds one complete ready-to-send 27-byte packet
size_t   q_head  = 0;                                     // Producer write index (SamplingTask enqueues here)
size_t   q_tail  = 0;                                     // Consumer read index (NetworkTask dequeues here)
size_t   q_count = 0;                                     // Current number of valid packets in the ring (mutex-protected)
bool hasSerial = false;                                   // Set after Serial.begin succeeds; used to safely gate all debug prints in field deployments

// ========== RTOS Objects and Shared State ==========
// Decision: mutex for buffer + lightweight flag to avoid lock in hottest path (priority inheritance, negligible overhead)
SemaphoreHandle_t bufferMutex = NULL;                 // Protects circular buffer indices during enqueue (SamplingTask) and snapshot/commit (NetworkTask)
SemaphoreHandle_t batchSemaphore = NULL;              // Binary semaphore woken by batch_timer every 50 ms to trigger NetworkTask packet transmission
esp_timer_handle_t batch_timer = NULL;                // Periodic esp_timer providing the batch rhythm for low-latency 3 kHz streaming
const uint32_t SAMPLING_TIMER_PERIOD_US = IMU_INTERVAL_US;  // Hardware timer period = exact IMU target interval for jitter-free sampling
SemaphoreHandle_t samplingSemaphore = NULL;           // Binary semaphore driven by sampling_timer at precise IMU rate (fewer wake-ups than software timers)
esp_timer_handle_t sampling_timer = NULL;             // High-resolution esp_timer waking SamplingTask with deterministic hardware timing
volatile bool g_isConnected = false;                  // Lock-free flag read by SamplingTask; written only by NetworkTask on connect/disconnect transitions

// ========== WIFI FUNCTIONS ANG HELPERS ==========

// Attempt to connect to host AP
void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();
  if (now - lastRetry >= WiFi_RETRY_DELAY_MS) {
    lastRetry = now;
    if (hasSerial) Serial.println(" "); Serial.print("Connecting to "); Serial.println(ssid);

    WiFi.begin(ssid, password);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
      delay(500);
      if (hasSerial) Serial.print("."); Serial.print(WiFi.status());
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (hasSerial) {
        Serial.println("\nWiFi connected"); 
        Serial.print("IP address: "); 
        Serial.println(WiFi.localIP());
        Serial.println("Attemping TCP server connection...");
      }

      // Establish persistent TCP connection
      if (client.connect(host_ip, host_port)) {
        client.setNoDelay(true);           // disable Nagle – send small bursts immediately
        if (hasSerial) Serial.println("Persistent TCP connected to host");
        WiFiState = CONNECTED;
        q_head = q_tail = q_count = 0;   // Clear circular buffer (zero-allocation version)
        time_init = esp_timer_get_time();
        g_isConnected = true;
        if (hasSerial) Serial.println("Entering CONNECTED state.");
      } 
      else {
        if (hasSerial) Serial.println("TCP server connection failed");
      }
    } 
    else {
      if (hasSerial) Serial.println("\nWiFi Connection failed, retrying...");
    }
  }
}

// Monitor connection to host
void monitorConnection() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck >= WiFi_CHECK_INTERVAL_MS) {
    lastCheck = now;
    if (WiFi.status() != WL_CONNECTED || !client.connected()) {
      if (hasSerial) Serial.println("WiFi or TCP disconnected.");
      client.stop();
      WiFiState = CONNECTING;
      if (hasSerial) Serial.println("Returning to CONNECTING state.");
    }
  }
}

static uint16_t crc_table[256];

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

// Fast CRC over a buffer (called once per IMU/MAG sample)
uint16_t compute_crc_fast(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
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
  if (target_hz <=  833) return 0b0111; // 833 Hz   ← 800 Hz target lands here
  if (target_hz <= 1660) return 0b1000; // 1.66 kHz
  if (target_hz <= 3330) return 0b1001; // 3.33 kHz
  return 0b1010;                         // 6.66 kHz (max) — anything higher also uses max
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
 * selectAccelFS_Bits()
 * --------------------
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
 * selectGyroFS_Bits()
 * -------------------
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
 * selectMagFS_Bits()
 * ------------------
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

void bringup_IMU() {
  Serial.print("Checking ISM330DHCX WHO_AM_I ... ");
  uint8_t whoami_val = readRegister(IMU_CS_PIN, IMU_WHO_AM_I);
  if (whoami_val != IMU_WHO_AM_I_VAL) {
    for (int i = 0; i < 10; ++i) {
      delay(5);
      whoami_val = readRegister(IMU_CS_PIN, IMU_WHO_AM_I);
      if (whoami_val == IMU_WHO_AM_I_VAL) break;
    }
  }
  if (whoami_val != IMU_WHO_AM_I_VAL) {
    Serial.println("FAILED (expected 0x6B)");
    Serial.printf("Got: 0b%08b / 0x%02X\n", whoami_val, whoami_val);
    while (true) delay(100);
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
  Serial.println("IMU high-performance mode, BDU+auto-increment enabled.");
}

void bringup_MAG() {
  Serial.print("Checking LIS3MDL WHO_AM_I ... ");
  uint8_t whoami_val = readRegister(MAG_CS_PIN, MAG_WHO_AM_I);
  if (whoami_val != MAG_WHO_AM_I_VAL) {
    for (int i = 0; i < 10; ++i) {
      delay(5);
      whoami_val = readRegister(MAG_CS_PIN, MAG_WHO_AM_I);
      if (whoami_val == MAG_WHO_AM_I_VAL) break;
    }
  }
  if (whoami_val != MAG_WHO_AM_I_VAL) {
    Serial.println("FAILED (expected 0x3D)");
    Serial.printf("Got: 0b%08b / 0x%02X\n", whoami_val, whoami_val);
    while (true) delay(100);
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
}

// ====================== LOW-LEVEL SPI HELPERS ======================

void writeRegister(int csPin, uint8_t regAddr, uint8_t value) {
  SPI.beginTransaction(sensorSPI);
  digitalWrite(csPin, LOW);
  SPI.transfer(regAddr & 0x7F);   // write
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(int csPin, uint8_t regAddr) {
  SPI.beginTransaction(sensorSPI);
  digitalWrite(csPin, LOW);
  SPI.transfer(regAddr | 0x80);   // read
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

// Burst read 12 bytes from IMU (gyro X/Y/Z + accel X/Y/Z)
void readIMU(int16_t &gx, int16_t &gy, int16_t &gz,
             int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t raw[12];
  SPI.beginTransaction(sensorSPI);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(IMU_OUTX_L_G | 0x80);   // read + auto-increment
  for (int i = 0; i < 12; i++) {
    raw[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();

  gx = (int16_t)(raw[0] | (raw[1] << 8));
  gy = (int16_t)(raw[2] | (raw[3] << 8));
  gz = (int16_t)(raw[4] | (raw[5] << 8));
  ax = (int16_t)(raw[6] | (raw[7] << 8));
  ay = (int16_t)(raw[8] | (raw[9] << 8));
  az = (int16_t)(raw[10] | (raw[11] << 8));
}

// Burst read 6 bytes from magnetometer
void readMAG(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t raw[6];
  SPI.beginTransaction(sensorSPI);
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(MAG_OUT_X_L | 0xC0);    // read + auto-increment (LIS3MDL)
  for (int i = 0; i < 6; i++) {
    raw[i] = SPI.transfer(0x00);
  }
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();

  mx = (int16_t)(raw[0] | (raw[1] << 8));
  my = (int16_t)(raw[2] | (raw[3] << 8));
  mz = (int16_t)(raw[4] | (raw[5] << 8));
}


// ====================== queueDataPacket_IMU_MAG  ======================
void queueDataPacket_IMU_MAG(uint32_t timestamp_us,
                             int16_t gx, int16_t gy, int16_t gz,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t mx, int16_t my, int16_t mz) {

    if (!g_isConnected) return;

    uint8_t packet[PACKET_SIZE_IMU];
    size_t offset = HEADER_SIZE_IMU;

    // Payload
    packet[offset++] = static_cast<uint8_t>(atoi(NANO_ID));

    memcpy(&packet[offset], &timestamp_us, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    int16_t imu_mag_vals[9] = {gx, gy, gz, ax, ay, az, mx, my, mz};
    memcpy(&packet[offset], imu_mag_vals, sizeof(imu_mag_vals));
    offset += sizeof(imu_mag_vals);

    // CRC over payload only (fast table version)
    uint16_t crc = compute_crc_fast(&packet[HEADER_SIZE_IMU], PAYLOAD_SIZE_IMU);

    // Header (written after CRC is known)
    offset = 0;
    packet[offset++] = static_cast<uint8_t>(PAYLOAD_SIZE_IMU & 0xFF);
    packet[offset++] = static_cast<uint8_t>((PAYLOAD_SIZE_IMU >> 8) & 0xFF);

    packet[offset++] = static_cast<uint8_t>(crc & 0xFF);
    packet[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    // Enqueue under mutex
    if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
        if (q_count >= MAX_PACKETS_IMU) {
            q_tail = (q_tail + 1) % MAX_PACKETS_IMU;
            q_count--;
        }
        memcpy(packet_pool[q_head], packet, PACKET_SIZE_IMU);
        q_head = (q_head + 1) % MAX_PACKETS_IMU;
        q_count++;
        xSemaphoreGive(bufferMutex);
    }
}

// ====================== sendQueuedDataTCP  ======================
bool sendQueuedDataTCP() {
    if (!client.connected()) return false;
    if (q_count == 0) return true;

    size_t first_chunk = 0;
    size_t bytes_first = 0;
    size_t second_chunk = 0;
    size_t bytes_second = 0;
    size_t snapshot_tail = 0;   // captured under lock for interference detection

    // Snapshot under lock (Decision: keep critical section as short as possible)
    if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
        if (q_count == 0) {
            xSemaphoreGive(bufferMutex);
            return true;
        }
        snapshot_tail = q_tail;
        first_chunk = min(q_count, MAX_PACKETS_IMU - q_tail);
        bytes_first = first_chunk * PACKET_SIZE_IMU;

        if (q_count > first_chunk) {
            second_chunk = q_count - first_chunk;
            bytes_second = second_chunk * PACKET_SIZE_IMU;
        }
        xSemaphoreGive(bufferMutex);
    }

    // Perform actual writes OUTSIDE the mutex (this is the long operation we wanted to decouple)
    size_t written = client.write(packet_pool[q_tail], bytes_first);
    if (written != bytes_first) {
        if (hasSerial) Serial.println("Partial write on first segment");
        return false;
    }

    // Commit first chunk under lock — always advance after successful write
    if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
        if (q_tail == snapshot_tail) {
            q_tail = (q_tail + first_chunk) % MAX_PACKETS_IMU;
            q_count -= first_chunk;
        } else {
            // Interference occurred (producer dropped samples while we were writing).
            // We still advance by what we sent so we do not re-transmit stale data.
            // A few packets may have been overwritten → host will see CRC fails/gaps.
            // This is the desired behaviour for high-rate fresh-data streaming.
            if (hasSerial) {
                Serial.println("WARNING: Producer drop interfered during TCP send — advancing anyway (stale re-send prevented)");
            }
            q_tail = (q_tail + first_chunk) % MAX_PACKETS_IMU;
            q_count -= first_chunk;   // may go slightly negative in extreme cases; clamp below
            if ((int)q_count < 0) q_count = 0;
        }
        xSemaphoreGive(bufferMutex);
    }

    if (second_chunk > 0) {
        written = client.write(packet_pool[q_tail], bytes_second);
        if (written != bytes_second) {
            if (hasSerial) Serial.println("Partial write on wrap segment");
            return false;
        }
        if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
            if (q_tail == ((snapshot_tail + first_chunk) % MAX_PACKETS_IMU) || q_count == 0) {
                q_tail = (q_tail + second_chunk) % MAX_PACKETS_IMU;
                q_count = 0;
            } else {
                if (hasSerial) {
                    Serial.println("WARNING: Producer drop interfered on wrap segment — advancing anyway (stale re-send prevented)");
                }
                q_tail = (q_tail + second_chunk) % MAX_PACKETS_IMU;
                q_count = 0;
            }
            xSemaphoreGive(bufferMutex);
        }
    }

    if (first_chunk + second_chunk > 200) {
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
void IRAM_ATTR sampling_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(samplingSemaphore, &xHigherPriorityTaskWoken);
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
    const uint32_t MAG_EVERY_N_IMU_SAMPLES = TARGET_IMU_RATE_HZ / TARGET_MAG_RATE_HZ;  // integer result = 3

    // Decision: Block on samplingSemaphore (given by esp_timer ISR).
    // Intent: The hardware timer now *is* the rate source. Task only runs when
    // a real sample is due. Timeout is just a safety net (rarely triggers).
    for (;;) {
        if (xSemaphoreTake(samplingSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (g_isConnected) {
                // === IMU sample (always happens at hardware-timed rate) ===
                uint32_t timestamp_us = (uint32_t)(esp_timer_get_time() - time_init);   // cast is safe; we only ever send the low 32 bits

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

                // Queue exactly as before (timestamp comes from the IMU sample instant)
                queueDataPacket_IMU_MAG(timestamp_us,
                                        gx, gy, gz, ax, ay, az,
                                        latched_mx, latched_my, latched_mz);
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
                connectToHost();           // blocks up to ~8 s inside, acceptable here
            } break;

            case CONNECTED: {
                if (!client.connected()) {
                    client.stop();
                    WiFiState = CONNECTING;
                    g_isConnected = false;
                    continue;
                }

                // Wait for next batch trigger from esp_timer (or timeout for monitoring)
                if (xSemaphoreTake(batchSemaphore, pdMS_TO_TICKS(200)) == pdTRUE) {
                    unsigned long send_start = micros();
                    bool sent = sendQueuedDataTCP();
                    unsigned long send_end = micros();

                    if (hasSerial && sent) {
                        Serial.print("Send took (us):");
                        Serial.println(send_end - send_start);
                    }
                }

                // Periodic connection health check (unchanged logic)
                unsigned long now = millis();
                if (now - lastCheck >= WiFi_CHECK_INTERVAL_MS) {
                    lastCheck = now;
                    monitorConnection();
                    if (WiFiState != CONNECTED) {
                        g_isConnected = false;
                    }
                }
            } break;
        }

        // Small yield so other tasks (including SamplingTask) get CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ====================== SETUP (creates tasks and supporting objects) ======================
void setup() {
    Serial.begin(1000000);
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {}
    hasSerial = Serial;

    if (hasSerial) {
        Serial.println("\n\nSerial connected for debugging");
        Serial.print("Nano_ID: "); Serial.println(NANO_ID);
        Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
        Serial.println("║   9-DOF High-Rate Binary Polling  —  MULTI-TASK REFACTORED            ║");
        Serial.println("╚═══════════════════════════════════════════════════════════════════════╝");
    }

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    init_crc_table();
    pinMode(IMU_CS_PIN, OUTPUT);
    pinMode(MAG_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    digitalWrite(MAG_CS_PIN, HIGH);
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

    bringup_IMU();
    bringup_MAG();

    // ====================== CREATE RTOS OBJECTS ======================
    if (hasSerial) Serial.println("\nStarting RTOS tasks...");

    bufferMutex = xSemaphoreCreateMutex();
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
    if (hasSerial) Serial.println("\tFinished batch timer interrupt.");

    // Create high-resolution sampling timer (different approach from vTaskDelayUntil)
    samplingSemaphore = xSemaphoreCreateBinary();
    esp_timer_create_args_t sampling_timer_args = {
        .callback = &sampling_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sampling_timer"
    };
    esp_timer_create(&sampling_timer_args, &sampling_timer);
    esp_timer_start_periodic(sampling_timer, SAMPLING_TIMER_PERIOD_US);
    if (hasSerial) Serial.println("\tFinished high-resolution sampling timer.");

    // ====================== CREATE TASKS (Decision: pinned + priorities) ======================
    // Decision: SamplingTask pinned to core 1 (high prio), NetworkTask to core 0.
    // Intent: Better WiFi stack affinity on core 0; SamplingTask isolated on core 1.
    xTaskCreatePinnedToCore(
        SamplingTask,
        "SamplingTask",
        4096,                    // user-specified stack size
        NULL,
        configMAX_PRIORITIES - 4, // high priority, but less that setup()
        NULL,
        1                        // core 1
    );
    if (hasSerial) Serial.println("\tFinished sensor sampling task.");

    xTaskCreatePinnedToCore(
        NetworkTask,
        "NetworkTask",
        6144,                    // generous for WiFi/TCP
        NULL,
        configMAX_PRIORITIES - 5, // lower priority
        NULL,
        0                        // core 0
    );
    if (hasSerial) Serial.println("\tFinished networking task.");

    if (hasSerial) {
        Serial.println("Both tasks created. SamplingTask running at target rate.");
        Serial.printf("Streaming binary packets at IMU rate (target %u Hz).\n", TARGET_IMU_RATE_HZ);
    }

    // Original loop() is intentionally left to idle (user decision 1.4)
}

// ====================== LOOP (now idle – replaced by dedicated tasks) ======================
void loop() {
    // Decision: Leave default Arduino loopTask idle.
    // Intent: All real work moved to explicit, priority-controlled FreeRTOS tasks.
    vTaskDelay(portMAX_DELAY);
}