/*
 * 9-DOF IMU + MAG High-Rate Polling Sketch (Binary Output)
 * =========================================================
 * 
 * Purpose: Maximum sustainable unique sample rate for Hi-STIFFS stalk probe
 *          3D pose tracking over 1-2 second rolling windows.
 *          IMU (ISM330DHCX) drives the packet rate; MAG (LIS3MDL) is
 *          latched only on fresh data to minimize redundant bytes before
 *          future WiFi burst offloads.
 *
 * Key improvements implemented:
 *   - Global target rate constants (easy one-line change for testing).
 *   - bringup_* functions now automatically select the closest supported
 *     ODR (rounded up) from the datasheet tables and configure the chips.
 *   - DRDY status polling (pure software poll, NO interrupts) so we only
 *     read and transmit when new data actually exists.
 *   - Compact binary packet output (fixed 27 bytes) for maximum throughput
 *     and easy host parsing / future WiFi buffering.
 *   - MAG data is only refreshed when its own DRDY asserts → no duplicate
 *     MAG transmissions in consecutive high-rate IMU packets.
 *   - SPI left at 8 MHz for maximum stability (as requested).
 *   - All comments are extremely detailed for direct copy-paste use and
 *     future maintenance.
 *
 * Packet format (sent on every new IMU sample):
 *   [0]    uint8_t  sync = 0xAA
 *   [1..8] uint64_t timestamp_us  (from esp_timer_get_time())
 *   [9..20] int16_t gx,gy,gz,ax,ay,az   (little-endian)
 *   [21..26] int16_t mx,my,mz           (latest latched MAG)
 *
 * Host parsing (Python example):
 *   import struct
 *   sync, ts, gx,gy,gz,ax,ay,az,mx,my,mz = struct.unpack("<B Q 9h", packet)
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
#include "esp_timer.h"   // high-resolution monotonic timestamps (µs)

// ====================== USER TUNABLE GLOBAL CONSTANTS ======================
// Change these lines to control the target output data rates and ranges.
// The bringup functions will automatically select the closest supported
// hardware ODR that is >= the target (rounded up from datasheet tables).
const uint16_t TARGET_IMU_RATE_HZ = 6660;   // IMU (accel + gyro). Supported HP rates: 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
const uint16_t TARGET_MAG_RATE_HZ = 1000;   // MAG. Supported with FAST_ODR: 155 (UHP), 300 (HP), 560 (MP), 1000 (LP) Hz
const uint32_t IMU_INTERVAL_US = 1000000UL / TARGET_IMU_RATE_HZ;   // e.g. 10000 µs for 100 Hz
const uint32_t MAG_INTERVAL_US = 1000000UL / TARGET_MAG_RATE_HZ;
const uint8_t  TARGET_ACCEL_FS_G   = 8;     // Options: 2, 4, 8, 16
const uint16_t TARGET_GYRO_FS_DPS  = 2000;  // Options: 125, 250, 500, 1000, 2000, 4000
const uint8_t  TARGET_MAG_FS_GAUSS = 12;    // Options: 4, 8, 12, 16

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

// ====================== BINARY PACKET DEFINITION ======================
struct __attribute__((packed)) IMUPacket {
  uint8_t  sync;           // 0xAA for easy framing / resync on host or WiFi
  uint64_t timestamp_us;   // monotonic from esp_timer_get_time()
  int16_t  gx, gy, gz;     // gyro  (dps raw)
  int16_t  ax, ay, az;     // accel (g raw)
  int16_t  mx, my, mz;     // mag   (gauss raw, latched latest)
};

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

// ====================== BINARY PACKET SENDER AND TIMER HELPER ======================

void sendBinaryPacket(uint64_t ts_us,
                      int16_t gx, int16_t gy, int16_t gz,
                      int16_t ax, int16_t ay, int16_t az,
                      int16_t mx, int16_t my, int16_t mz) {
  IMUPacket pkt;
  pkt.sync = 0xAA;
  pkt.timestamp_us = ts_us;
  pkt.gx = gx; pkt.gy = gy; pkt.gz = gz;
  pkt.ax = ax; pkt.ay = ay; pkt.az = az;
  pkt.mx = mx; pkt.my = my; pkt.mz = mz;

  // Single efficient write (ESP32 USB CDC handles this well)
  Serial.write((uint8_t *)&pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Minimal timer helper — returns true and updates last_us only when the
// interval has elapsed. Uses monotonic esp_timer (µs) so it is unaffected by
// millis() rollover or task switching. Call it for each sensor independently.
// -----------------------------------------------------------------------------
inline bool timerExpired(uint64_t &last_us, uint32_t interval_us) {
  uint64_t now = esp_timer_get_time();
  if (now - last_us >= interval_us) {
    last_us = now;           // reset to actual trigger time (low jitter, no drift accumulation)
    return true;
  }
  return false;
}
// ====================== SETUP ======================
void setup() {
  Serial.begin(1000000);   // high speed native USB
  while (!Serial) {}

  Serial.println("╔════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║   9-DOF High-Rate Binary Polling  —  ISM330DHCX + LIS3MDL                  ║");
  Serial.println("║   DRDY status polling (no interrupts)  |  Binary packets for WiFi bursts   ║");
  Serial.println("╚════════════════════════════════════════════════════════════════════════════╝");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  pinMode(IMU_CS_PIN, OUTPUT);
  pinMode(MAG_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);
  digitalWrite(MAG_CS_PIN, HIGH);
  delay(100);

  bringup_IMU();
  bringup_MAG();

  Serial.println();
  Serial.printf("Streaming binary packets at IMU rate (target %u Hz). MAG latched on its own DRDY.\n", TARGET_IMU_RATE_HZ);
  Serial.println("Packet: sync(0xAA) + uint64_t ts_us + 6xint16 IMU + 3xint16 MAG (27 bytes total)");
  Serial.println("────────────────────────────────────────────────────────────────────────────");
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

  // Check MAG first (cheap) so its latest value is available if IMU also fires
  // on this same iteration. Order does not matter for correctness.
  if (timerExpired(last_mag_us, MAG_INTERVAL_US)) {
    // Fresh MAG data is due at the target MAG rate.
    // We still use the existing readMAG() helper (burst 6 bytes over SPI).
    // This replaces the old "read MAG_STATUS every single IMU sample" pattern.
    readMAG(latched_mx, latched_my, latched_mz);
  }

  if (timerExpired(last_imu_us, IMU_INTERVAL_US)) {
    // IMU data is due at the target IMU rate → this is what produces packets.
    uint64_t timestamp_us = esp_timer_get_time();   // fresh timestamp for this sample

    int16_t gx, gy, gz, ax, ay, az;
    readIMU(gx, gy, gz, ax, ay, az);

    // Send the compact 27-byte binary packet exactly as before.
    // It always carries the most recent latched MAG (which may have been
    // updated on this iteration or on a previous MAG timer tick).
    sendBinaryPacket(timestamp_us,
                     gx, gy, gz, ax, ay, az,
                     latched_mx, latched_my, latched_mz);
  }

  // No delay(), no yield(), no blocking. The loop spins as fast as possible
  // but performs real SPI work only when a timer says a sensor is due.
  // This gives excellent efficiency for the current two-chip case and will
  // scale cleanly when more sensors (each with their own timer check) are added.
}