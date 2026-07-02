/*
 * 9-DOF IMU + MAG High-Rate Polling Sketch w/WiFi 
 * =========================================================
 * 
 * Purpose: Maximum sustainable unique sample rate for Hi-STIFFS stalk probe
 *          3D pose tracking over 1-2 second rolling windows.
 *          IMU (ISM330DHCX) drives the packet rate; MAG (LIS3MDL) is
 *          latched only on fresh data to minimize redundant bytes before
 *          WiFi burst offloads.
 *
 * Packet format [33 bytes]:
 *   --- OVERHEAD ---
 *   [0..1]   uint16_t length
 *   [2..3]   unit16_t sequence number
 *   [4..5]   unit16_t crc, (cyclic redundancy check code)
 *   --- PAYLOAD ---
 *   [6]      unit8_t  nano_ID
 *   [7]      unit8_t  sensor_type (0 for ICB sensors, 1 for IMU/MAG)
 *   [8..15]  uint64_t timestamp_us  (from esp_timer_get_time())
 *   [16..27] int16_t  gx,gy,gz,ax,ay,az   (little-endian)
 *   [28..33] int16_t  mx,my,mz           (latest latched MAG)
 *
 * Host payload parsing (Python example):
 *   import struct
 *   nano_ID,sensor_type, ts_us, gx,gy,gz,ax,ay,az, mx,my,mz = struct.unpack("<BB Q 9h", packet)
 *
 * Pinout (Arduino Nano ESP32):
 *   IMU (ISM330DHCX) : CS = A4
 *   MAG (LIS3MDL)    : CS = A5
 *   SPI              : SCK=13, MISO=11, MOSI=12
 *
 * Data rate control:
 *   Change TARGET_IMU_RATE_HZ and TARGET_MAG_RATE_HZ below.
 *   The configuration functions will pick the nearest supported rate
 *   (rounded UP) that the chip can actually deliver in high-performance mode.
 */

// ====================== INCLUDES ======================
#include <SPI.h>
#include <WiFi.h>       // For WiFi Station mode
#include <deque>        // For queuing packets to batch sends
#include <vector>       // For storing binary packet data
#include "esp_timer.h"  // high-resolution monotonic timestamps (µs)

// ====================== REGISTER ADDRESSES ======================
// ISM330DHCX (IMU)
#define IMU_WHO_AM_I          0x0F
#define IMU_CTRL1_XL          0x10
#define IMU_CTRL2_G           0x11
#define IMU_CTRL3_C           0x12
#define IMU_STATUS_REG        0x1E   // Bit0=XLDA, Bit1=GDA (we poll these)
#define IMU_OUTX_L_G          0x22
#define IMU_INT1_CTRL         0x0D

#define IMU_WHO_AM_I_VAL      0x6B
#define IMU_CTRL3_C_VAL       0x44   // BDU + address auto-increment + 4-wire SPI (keep this)

// LIS3MDL (MAG)
#define MAG_WHO_AM_I          0x0F
#define MAG_CTRL_REG1         0x20
#define MAG_CTRL_REG2         0x21
#define MAG_CTRL_REG3         0x22
#define MAG_CTRL_REG4         0x23
#define MAG_CTRL_REG5         0x24
#define MAG_STATUS_REG        0x27   // Bit0=DRDY
#define MAG_OUT_X_L           0x28

#define MAG_WHO_AM_I_VAL      0x3D

// ====================== PIN DEFINITIONS ======================
#define SPI_SCK     13
#define SPI_MISO    11
#define SPI_MOSI    12
#define IMU_CS_PIN  A4
#define MAG_CS_PIN  A5

// ====================== SPI SETTINGS ======================
// Kept at 8 MHz for maximum long-term stability in field deployments
// (both chips support 10 MHz, but 8 MHz has more margin on long cables / noise).
SPISettings sensorSPI(8000000, MSBFIRST, SPI_MODE3);

// ====================== USER TUNABLE GLOBAL CONSTANTS ======================
// Change these lines to control the target output data rates and ranges.
// The bringup functions will automatically select the closest supported
// hardware ODR that is >= the target (rounded up from datasheet tables).
const uint16_t TARGET_IMU_RATE_HZ = 1000;   // IMU (accel + gyro). Supported HP rates: 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
const uint16_t TARGET_MAG_RATE_HZ = 1000;   // MAG. Supported with FAST_ODR: 155 (UHP), 300 (HP), 560 (MP), 1000 (LP) Hz
const uint32_t IMU_INTERVAL_US = 1000000UL / TARGET_IMU_RATE_HZ;   // e.g. 10000 µs for 100 Hz
const uint32_t MAG_INTERVAL_US = 1000000UL / TARGET_MAG_RATE_HZ;
const uint8_t  TARGET_ACCEL_FS_G   = 8;     // Options: 2, 4, 8, 16
const uint16_t TARGET_GYRO_FS_DPS  = 2000;  // Options: 125, 250, 500, 1000, 2000, 4000
const uint8_t  TARGET_MAG_FS_GAUSS = 12;    // Options: 4, 8, 12, 16

// ========== WiFi access point to connect to host device ==========
const char* ssid = "Hi-STIFFS_Host";       // Host WiFi network name (SSID) - choose something unique
const char* password = "BYUCropBio";       // Host WiFi password (minimum 8 characters)
const char* ota_password = "BYUCropBio";   // Optional OTA password for security (change this)
// Host server details
const char* host_ip = "192.168.137.1";       // IPv4 of the host server (e.g., Raspberry Pi or laptop), from device settings, not Python code
// const char* host_ip = "192.168.137.1";    // For Pi_001
const int host_port = 8080;                  // Port on host for data streaming
// Unique Nano ID in network of multiple Arduinos (2-digit, e.g., "01" to "99")
const char* NANO_ID = "01";                // Assign unique per Nano
// Batching configuration: Send queued packets every this interval (ms)
const unsigned long BATCH_SEND_INTERVAL_MS = 100;  // Default 0.1s; adjust for desired batch frequency
const unsigned long WiFi_CHECK_INTERVAL_MS = 1000;  // 1 Hz
const unsigned long WiFi_RETRY_DELAY_MS = 5000;     // Retry connection every 5s if failed
enum class SensorDataType : uint8_t {
    TYPE_ICB   = 0,   // Added to packet payload to indicate it contains ICB sensor data
    TYPE_IMU_MAG = 1  // Added to packet payload to indicate it contains IMU/MAG sensor data
};
enum State_WiFi {
  CONNECTING,   // Attempting to connect to host AP
  CONNECTED,    // Connected to host, sending data
};
State_WiFi WiFiState = CONNECTING;

WiFiClient client;  // Global client for sending data to host

// ====================== BINARY PACKET and WiFi Batching Queue ======================
struct __attribute__((packed)) IMUPacket {
  uint8_t  sync;           // 0xAA for easy framing / resync on host or WiFi
  uint64_t timestamp_us;   // monotonic from esp_timer_get_time()
  int16_t  gx, gy, gz;     // gyro  (dps raw)
  int16_t  ax, ay, az;     // accel (g raw)
  int16_t  mx, my, mz;     // mag   (gauss raw, latched latest)
};

// Packet queue for batching: Stores binary packets ready to send
uint16_t seq_num = 0;                               // Sequence number for packets, increments per packet
unsigned long last_batch_send = 0;                  // Timestamp of last batch send
uint64_t time_init = esp_timer_get_time();                            // t=0 of datastream. Reset each time WiFi connection initiates datastream

const size_t HEADER_SIZE_IMU   = 6;
const size_t PAYLOAD_SIZE_IMU  = 2 + sizeof(uint64_t) + 9 * sizeof(int16_t); // 28
const size_t PACKET_SIZE_IMU = HEADER_SIZE_IMU + PAYLOAD_SIZE_IMU;     // 6-byte header + 28-byte payload (must match queueDataPacket_IMU_MAG)
constexpr size_t MAX_PACKETS_IMU = TARGET_IMU_RATE_HZ * BATCH_SEND_INTERVAL_MS / 1000 * 2;    // 2 batch send periods in case regaular interval is missed

uint8_t  packet_pool[MAX_PACKETS_IMU][PACKET_SIZE_IMU];
size_t   q_head  = 0;   // producer write index
size_t   q_tail  = 0;   // sender read index
size_t   q_count = 0;   // current depth (for fast empty/full checks)

// Global flag for Serial presence, set in setup
bool hasSerial = false;

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

// Compute WiFi cyclic redundancy check code for packet integrity check
uint16_t compute_crc(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;  // CRC-CCITT initial value
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// ============================================================
// Fast table-driven CRC16-CCITT (reflected, poly 0xA001, init 0xFFFF)
// ------------------------------------------------------------
// This is the EXACT same algorithm used in collect_data.py
// (WiFiDataServer._crc16_ccitt + _generate_crc16_table).
// It produces identical CRC values, so the wire protocol and
// host-side validation are unchanged.
//
// Why we do this:
//   - The original bit-by-bit version was correct but performed
//     ~200 conditional branches + shifts per sample at 1000 Hz.
//   - Table-driven version = one 256-entry table (built once) +
//     28 cheap lookups per sample → ~8-10× faster in the hot path.
//   - Table is small (512 bytes) and lives in RAM; on ESP32 this
//     is negligible and gives deterministic timing.
//
// Decision: build the table at runtime in setup() so it is
// guaranteed identical to the Python implementation even if
// someone later changes the polynomial.
// ============================================================

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
  Serial.println("MAG continuous mode, DRDY polling will be used.");
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

// ====================== DATA PACKET COLLECT, QUEUE, AND SEND MANGAGEMENT ======================


// ------------------------------------------------------------
// Builds and enqueues one 34-byte binary-framed IMU/MAG packet
// at the full target rate with ZERO heap
// allocations in the hot path.
//
// Implementation now uses the static circular buffer declared above.
// Only memcpy + modular index arithmetic remain.
//
// Wire layout (34 bytes total, little-endian, unchanged):
//   [0..1]   uint16_t payload_length = 28
//   [2..3]   uint16_t seq_num
//   [4..5]   uint16_t CRC16 (over payload only)
//   [6]      uint8_t  nano_id
//   [7]      uint8_t  sensor_type = 1 (IMU_MAG)
//   [8..15]  uint64_t timestamp_us
//   [16..33] int16_t  gx,gy,gz,ax,ay,az,mx,my,mz   (9 values)
//
// Intent: make the path cheap and deterministic enough that
// the ESP32 can comfortably sustain it while WiFi/TCP is also running.
// ============================================================

void queueDataPacket_IMU_MAG(uint64_t timestamp_us,
                             int16_t gx, int16_t gy, int16_t gz,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t mx, int16_t my, int16_t mz) {

    uint8_t packet[PACKET_SIZE_IMU];   // fixed stack buffer – zero heap cost

    // --- Payload first (we need it for CRC) ---
    size_t offset = HEADER_SIZE_IMU;

    // nano_id (from flashed const char* NANO_ID)
    packet[offset++] = static_cast<uint8_t>(atoi(NANO_ID));

    // sensor_type
    packet[offset++] = static_cast<uint8_t>(SensorDataType::TYPE_IMU_MAG);

    // timestamp_us (uint64_t, little-endian) – single memcpy
    memcpy(&packet[offset], &timestamp_us, sizeof(uint64_t));
    offset += sizeof(uint64_t);

    // 9 × int16_t in host-expected order: gx gy gz ax ay az mx my mz
    int16_t imu_mag_vals[9] = {gx, gy, gz, ax, ay, az, mx, my, mz};
    memcpy(&packet[offset], imu_mag_vals, sizeof(imu_mag_vals));
    offset += sizeof(imu_mag_vals);

    // --- CRC over payload only (fast table-driven version) ---
    uint16_t crc = compute_crc_fast(&packet[HEADER_SIZE_IMU], PAYLOAD_SIZE_IMU);

    // --- Header (written after CRC is known) ---
    offset = 0;

    // Length (uint16_t LE)
    packet[offset++] = static_cast<uint8_t>(PAYLOAD_SIZE_IMU & 0xFF);
    packet[offset++] = static_cast<uint8_t>((PAYLOAD_SIZE_IMU >> 8) & 0xFF);

    // Sequence number (uint16_t LE) + post-increment
    packet[offset++] = static_cast<uint8_t>(seq_num & 0xFF);
    packet[offset++] = static_cast<uint8_t>((seq_num >> 8) & 0xFF);
    seq_num++;

    // CRC (uint16_t LE)
    packet[offset++] = static_cast<uint8_t>(crc & 0xFF);
    packet[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    // --- Enqueue into the zero-allocation circular buffer ---
    if (q_count >= MAX_PACKETS_IMU) {
        // Buffer full — drop oldest sample (keeps queue "fresh" for pose estimation).
        // This is a robustness improvement over the previous unbounded deque.
        q_tail = (q_tail + 1) % MAX_PACKETS_IMU;
        q_count--;
    }

    memcpy(packet_pool[q_head], packet, PACKET_SIZE_IMU);
    q_head = (q_head + 1) % MAX_PACKETS_IMU;
    q_count++;
}


// sendQueuedDataTCP  (coalesced writer using circular buffer)
// ------------------------------------------------------------
// Highest-impact optimization for sustained high-rate streaming.
// Now operates on the static circular buffer instead of std::deque.
//
// Behavior preserved exactly:
//   - Only drains on *complete* successful write(s)
//   - On any partial write we leave tail untouched and return false
//     so the next 100 ms interval will retry the same data
//   - One flush after full success
//   - Wire format, CRC, seq_num unchanged
//
// Uses at most two client.write() calls (handles wrap-around).
// Still far fewer syscalls than the original per-packet writes.
// ============================================================
bool sendQueuedDataTCP() {
    if (!client.connected()) {
        if (hasSerial) Serial.println("TCP not connected; skipping send");
        return false;
    }

    if (q_count == 0) {   // There is no data ready to be sent
        return true;
    }

    // First contiguous segment (may be the entire queue or up to the wrap point)
    size_t first_chunk = min(q_count, MAX_PACKETS_IMU - q_tail);
    size_t bytes_first = first_chunk * PACKET_SIZE_IMU;

    size_t written = client.write(packet_pool[q_tail], bytes_first);

    if (written != bytes_first) {
        if (hasSerial) Serial.println("Partial write on first segment — will retry");
        return false;
    }

    // Advance past first segment
    q_tail = (q_tail + first_chunk) % MAX_PACKETS_IMU;
    q_count -= first_chunk;

    // Second segment if we wrapped
    if (q_count > 0) {
        size_t bytes_second = q_count * PACKET_SIZE_IMU;
        written = client.write(packet_pool[q_tail], bytes_second);

        if (written != bytes_second) {
            if (hasSerial) Serial.println("Partial write on wrap segment — will retry");
            return false;
        }

        q_tail = (q_tail + q_count) % MAX_PACKETS_IMU;
        q_count = 0;
    }

    client.flush();
    return true;
}

// -----------------------------------------------------------------------------
// Minimal timer helper — returns true and updates last_us only when the
// interval has elapsed. Uses monotonic esp_timer (µs) so it is unaffected by
// millis() rollover or task switching. Call it for each IMU or MAG independently.
// -----------------------------------------------------------------------------
inline bool IMU_MAG_timerExpired(uint64_t &last_us, uint32_t interval_us) {
  uint64_t now = esp_timer_get_time();
  if (now - last_us >= interval_us) {
    last_us = now;           // reset to actual trigger time (low jitter, no drift accumulation)
    return true;
  }
  return false;
}

// ====================== SETUP ======================
void setup() {
  // Serial detection and init
  Serial.begin(1000000);
  unsigned long startTime = millis();
  while (!Serial) {
    if (millis() - startTime > 5000) {
      // Continuing without serial
      break;
    }
  }
  hasSerial = Serial;  // Set flag based on connection
  if (hasSerial) {
    Serial.println(" "); Serial.println(" "); Serial.println(" "); 
    Serial.println("Serial connected for debugging");
    Serial.print("Nano_ID: ");
    Serial.println(NANO_ID);
  }

  Serial.println("╔═══════════════════════════════════════════════════════════════════════╗");
  Serial.println("║   9-DOF High-Rate Binary Polling  —  ISM330DHCX + LIS3MDL             ║");
  Serial.println("║   No interrupts  |  Binary packets for WiFi batching                  ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════╝");

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

  Serial.println();
  Serial.printf("Streaming binary packets at IMU rate (target %u Hz). MAG latched on its own DRDY.\n", TARGET_IMU_RATE_HZ);

  last_batch_send = millis();
}

// ====================== LOOP (throttled, zero STATUS polling) ======================
void loop() {
  // ---------------------------------------------------------------------------
  // Independent software timers for IMU and MAG.
  // - Each sensor is read ONLY when its own timer has expired.
  // - No STATUS_REG reads at all in the hot path → far fewer SPI transactions
  //   than the original "poll every iteration" design.
  // - IMU timer drives the binary packet rate (exactly as original intent).
  // - MAG timer controls how often we refresh the latched values.
  //   We latch the values so every IMU packet always carries the *latest known* MAG.
  // - On first entry the statics start at 0, so both sensors read immediately
  //   (desirable for startup streaming).
  // - Uses integer µs arithmetic only — extremely fast on ESP32.
  // ---------------------------------------------------------------------------
  static uint64_t last_imu_us = 0;
  static uint64_t last_mag_us = 0;
  static int16_t latched_mx = 0, latched_my = 0, latched_mz = 0;

  switch (WiFiState) {
    case CONNECTED: {
      if (!client.connected()) {
        client.stop();
        WiFiState = CONNECTING;
      }
      if (IMU_MAG_timerExpired(last_mag_us, MAG_INTERVAL_US)) {
        // Fresh MAG data is due at the target MAG rate.
        // We still use the existing readMAG() helper (burst 6 bytes over SPI).
        // This replaces the old "read MAG_STATUS every single IMU sample" pattern.
        readMAG(latched_mx, latched_my, latched_mz);
      }
      if (IMU_MAG_timerExpired(last_imu_us, IMU_INTERVAL_US)) {
        // IMU data is due at the target IMU rate → this is what produces packets.
        uint64_t timestamp_us = esp_timer_get_time() - time_init;   // fresh timestamp for this sample

        int16_t gx, gy, gz, ax, ay, az;
        readIMU(gx, gy, gz, ax, ay, az);

        // Queue binary packet
        // It always carries the most recent latched MAG (which may have been
        // updated on this iteration or on a previous MAG timer tick).
        // TODO - update this function to accept IMU/MAG data instead of working with global ICB data
        queueDataPacket_IMU_MAG(timestamp_us,
                                  gx, gy, gz, ax, ay, az,
                                  latched_mx, latched_my, latched_mz);
      }

      // Check if time to send batch of packets
      unsigned long now = millis();
      if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
        unsigned long send_start = micros();
        sendQueuedDataTCP();
        unsigned long send_end = micros();
        last_batch_send = now;

        if (hasSerial) {
          Serial.print("Send took (us):");
          Serial.println(send_end - send_start);
        }
      }
      monitorConnection();
    } break;
    
    case CONNECTING: {
      connectToHost();
    } break;

  }
}