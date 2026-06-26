//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino code for reading from up to 5 ADS1220 chips on an Arduino Nano ESP32.
//    Each ADS1220 reads 2 differential channels (AIN0-AIN1 and AIN2-AIN3),
//    labeled as sensors A-E with channels 1-2 (e.g., A1, A2, B1, B2, etc.).
//    Total possible channels: 10.
//
//    This code is based on the provided Protocentral_ADS1220 library and the example
//    new_DAQ_2.ino. It uses interrupts on DRDY pins for efficient data reading.
//
//    Key Decisions and Justifications (Updated for 2026 Wireless Integration):
//    - Modular Design: 
//      - Use a configurable number of sensors (NUM_SENSORS) and an array
//        of configurations. To use fewer sensors (e.g., only A-C), set NUM_SENSORS=3. 
//      - The ADS_sensors array lists all possible configurations in order (A to E). 
//      - The code will only use the first NUM_SENSORS entries from this array. This allows 
//        easy enabling/disabling by just changing NUM_SENSORS without commenting out lines 
//        in the array initializer. If you need to change pin assignments or IDs, edit the 
//        ADS_sensors array directly.
//    - Pin Assignments: 
//      - SPI pins: Fixed on Nano ESP32 as D13 (SCK), D11 (MOSI/COPI), D12 (MISO/CIPO).
//      - CS pins: D10, D9, D8, D7, D6 (outputs, chosen to avoid SPI pins and common serial pins).
//      - DRDY pins: D5, D4, D3, D2, A6 (inputs with interrupt support). A6 is used as digital input
//        (valid on ESP32). Avoided D0/D1 (potential UART) and analog-only pins without interrupt needs.
//        These pins are plentiful on Nano ESP32 (14 digital + 8 analog-as-digital).
//      - Justification: Ensures no conflicts with SPI bus or USB Serial. All DRDY pins support
//        attachInterrupt on ESP32.
//    - Operating Mode and Data Rate:
//      - Set to Turbo mode (MODE_TURBO) as requested.
//      - Data rate set to DR_90SPS, which becomes ~180 SPS in turbo mode for ~90 SPS per channel
//        (per ADS1220 datasheet, Table 11). Adjusted from DR_175SPS for stability with WiFi.
//    - Ratiometric Measurement: 
//      - Configured for ratiometric bridge readings as recommended
//        in the ADS1220 datasheet (section 9.2.3). The bridge excitation voltage (5.1V) is
//        used as the external reference by setting VREF to AVDD (ANALOG). This cancels excitation voltage
//        variations. 
//      - Note: Since we now send raw ADC integers instead of mV, the voltage 
//        calculation (which previously used VREF=5.1V) is no longer performed here. Any 
//        conversion to physical units should be done on the receiving side if needed.
//      - Justification: Datasheet recommends using excitation as reference for ratiometric
//        operation to improve accuracy in bridge sensors like load cells.
//    - Synchronization Within Each Sensor:
//      - Each ADS1220 has one ADC, so channels are multiplexed and converted sequentially.
//      - Use continuous mode with mux switching on DRDY interrupt (as in example). After reading channel 1,
//        switch to channel 2 (restarts conversion). Next DRDY interupt reads channel 2.
//      - The two channels per chip are ~1/data_rate apart (~11ms at 90Hz), which is "reasonably synchronized".
//      - Single timestamp (micros()) taken after reading the second channel, applied to both. This approximates
//        the pair as synchronized.
//      - Justification: Simultaneous conversion impossible with single ADC. This method minimizes delay.
//    - Synchronization Between Sensors: 
//      - Not implemented. Sensors run independently (internal oscillators may drift slightly over time, ~2% per datasheet). 
//      - If needed, add periodic START/SYNC commands to all chips (requires careful handling to avoid MISO conflicts).
//    - Output Format (Wireless Update): 
//      - Send data only when all sensors are ready (a full cycle), in a
//        single efficient CSV line: tsA,A1_raw,A2_raw,tsB,B1_raw,B2_raw,... Each sensor has its
//        own timestamp (taken after reading its second channel). Timestamps are always per-sensor to account for any minor delays.
//      - The _raw values are the 24-bit signed integers directly from the ADC (sign-extended to 32-bit for printing).
//        Previously, these were converted to mV using the formula: (float(val) * VREF / PGA_GAIN / (1LL<<23)) * 1000.0f,
//        but now we send the raw integers for flexibility (conversion can be done later if needed).
//      - Now outputs over WiFi using persistent TCP connection to a host server (port 80) for streaming.
//        The Nano connects as a WiFi client to the host's AP.
//      - Serial output retained for debugging (with timeout to avoid blocking if no USB connected).
//      - Justification: Enables untethered field operation; persistent TCP provides efficient, reliable streaming compatible with Python servers.
//        High baud rate serial fallback for wired debugging.
//    - Wireless Features:
//      - Station mode: Nano connects to host's WiFi network (configurable SSID/password).
//      - OTA: Supports over-the-air sketch uploads via WiFi for wireless updates.
//      - Power Impact: Station mode adds ~80-150mA draw; monitor for battery-powered setups (see Nano ESP32 manual, Section 11).
//    - Other Settings: 
//      - PGA=128 (Programmable Gain Amplifier), internal VREF=2.048V (from datasheet). Continuous mode for steady sampling.
//    - Interrupt Handling: 
//      - Use lambdas to pass sensor index to handler (C++11, supported in Arduino).
//      - Note: In the code, we use a fixed array of 5 ISR handlers (handleDrdy0 to handleDrdy4) because 
//        attaching interrupts with lambdas that capture variables can be tricky on ESP32 (especially with IRAM_ATTR).
//        We define all 5 handlers even if NUM_SENSORS < 5, but only attach the first NUM_SENSORS. This is simple 
//        and avoids issues with captured lambdas in interrupts.
//    - Error Handling: 
//      - Basic; assumes DRDY triggers reliably. Added serial timeout for non-blocking boot.
//
//    Dependencies: Protocentral_ADS1220 library (include .h and .cpp as provided).
//
//    For information on ADS1220, see datasheet. For Nano ESP32 pins and WiFi, see provided PDF.
//
//    This software is licensed under the MIT License (as in original library).
//
//    Updated Architecture (2026):
//    - Finite State Machine (FSM): Manages connection states to attempt connection when idle,
//      send data when connected, and handle Serial presence determined at setup. States: CONNECTING,
//      CONNECTED, SERIAL_ONLY, NO_SERIAL. This ensures efficient resource use on Nano ESP32.
//    - Modular Functions: Broke out key operations (e.g., OTA handling, DRDY processing, data sending)
//      for tight, efficient code. Prepares for higher data rates by minimizing loop overhead.
//    - OTA Restriction: Disabled during active connections to prevent disruptions.
//    - ADC Reset: On new connection, reset ADCs and timestamps for session synchronization.
//    - Error Gracefulness: Added checks for missed DRDY (e.g., skip if flag not set), connection errors
//      (retry), and suppressed Serial if absent to avoid blocking.
//    - Efficiency for Nano ESP32: Used IRAM_ATTR for ISRs, non-blocking loops, conditional debug
//      prints. Aligned with PDF specs (e.g., low power notes for future optimizations, pin capabilities).
//    - Future-Proofing: Code structure allows easy data rate increases (e.g., to DR_1000SPS in turbo,
//      ~2000 SPS total) once tested; focus on tight execution to handle higher interrupt rates.
//    - Multi-Nano Compatibility: Each Nano has a unique ID (NANO_ID) sent in data packets.
//      Host handles aggregation; no changes here for multi-Nano beyond ID inclusion.
//    - Persistent TCP: Replaced per-packet HTTP POST with a single persistent TCP connection
//      established once in CONNECTED state. Data is streamed as length-prefixed binary frames
//      for reliable, low-overhead transmission. This avoids repeated handshakes and headers.
//    - Queued Batching: Added a queue (std::deque<std::vector<uint8_t>>) to store complete
//      binary packets when data is ready. Every BATCH_SEND_INTERVAL_MS (configurable, default 100ms),
//      the queue is drained and all packets are sent over the persistent connection. This batches
//      sends to amortize network overhead while maintaining low latency.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>       // For WiFi Station mode
#include <ArduinoOTA.h> // For Over-The-Air updates
#include <deque>        // For queuing packets to batch sends
#include <vector>       // For storing binary packet data

#define FULL_SCALE (1LL << 23) // 2^23 for 24-bit signed scaling - retained for reference, though not used in raw output
#define A_CS_PIN 10
#define A_DRDY_PIN 9
#define B_CS_PIN 8
#define B_DRDY_PIN 7
#define C_CS_PIN 6
#define C_DRDY_PIN 5
#define D_CS_PIN 4
#define D_DRDY_PIN 3
#define E_CS_PIN 2
#define E_DRDY_PIN A1
#define F_CS_PIN A2
#define F_DRDY_PIN A3

const int SPI_SCK  = 13;   // default D13
const int SPI_MISO = 11;   // default D12 (CIPO)
const int SPI_MOSI = 12;   // default D11 (COPI)

const int MAX_SENSORS = 6;     // Maximum possible sensors (A to F)
const int NUM_SENSORS = 3;     // Set to 1-5 to use the first N sensors from ADS_sensors below.
const uint8_t dr_code = DR_330SPS;  // Data Rate value. In turbo, value is for pairs/sec. In normal, value is for samples/sec
const SPISettings spi_settings(2000000, MSBFIRST, SPI_MODE1);

// WiFi access point to connect to (host's AP)
const char* ssid = "Hi-STIFFS_Host";       // Host WiFi network name (SSID) - choose something unique
const char* password = "BYUCropBio";       // Host WiFi password (minimum 8 characters)
const char* ota_password = "BYUCropBio";   // Optional OTA password for security (change this)
// Host server details
// const char* host_ip = "192.168.137.1";       // IPv4 of the host server (e.g., Raspberry Pi or laptop), from device settings, not Python code
const char* host_ip = "192.168.137.1";    // For Pi_001
const int host_port = 8080;                  // Port on host for data streaming
WiFiClient client;                         // Global client for sending data to host

// Unique Nano ID (2-digit, e.g., "01" to "99")
const char* NANO_ID = "01";                // Assign per Nano; for now, fixed to "01"

// Batching configuration: Send queued packets every this interval (ms)
const unsigned long BATCH_SEND_INTERVAL_MS = 10;  // Default 0.1s; adjust for desired batch frequency

struct SensorConfig {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int cs_pin;                  // Chip Select pin
  int drdy_pin;                // Data Ready pin
};

// Full list of configurations for sensors A to E. The code uses only the first NUM_SENSORS.
// To change pins or IDs, edit this array. No commenting out needed - just set NUM_SENSORS.
SensorConfig ADS_sensors[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN},
  {'F', F_CS_PIN, F_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];             // ADC objects from library
int32_t raw_values[MAX_SENSORS][2];                 // [sensor][channel]: raw 24-bit signed ADC values; 0=ch1 (AIN0-1), 1=ch2 (AIN2-3)
uint32_t timestamps[MAX_SENSORS];                    // Per-chip timestamps for each pair
uint8_t current_channels[MAX_SENSORS] = {0};        // Indicates which MUX channel to read from for an ADS1220 module
volatile uint8_t ready_mask = 0;                    // Bitmask tracking ready sensors (bit i set to (1) when sensor i pair is complete)
uint32_t time_init;                            // t=0 of datastream. Set each time connection initiates datastream
const unsigned long WiFi_CHECK_INTERVAL_MS = 1000;  // 1 Hz
const unsigned long WiFi_RETRY_DELAY_MS = 5000;     // Retry connection every 5s if failed

// Packet queue for batching: Stores binary packets ready to send
std::deque<std::vector<uint8_t>> packet_queue;      // Queue of binary packets (each vector is one full sensor set)
uint16_t seq_num = 0;                               // Sequence number for packets, increments per packet
unsigned long last_batch_send = 0;                  // Timestamp of last batch send

// FSM States
enum State_WiFi {
  CONNECTING,   // Attempting to connect to host AP
  CONNECTED,    // Connected to host, sending data
};
State_WiFi WiFiState = CONNECTING;

// Global flag for Serial presence, set in setup
bool hasSerial = false;

// Packet types
enum PacketType : uint8_t
{
  PKT_IMU = 1,
  PKT_MAG = 2,
  PKT_ADS = 3
};
struct AdsPacket {
  uint64_t ts_us;
  char adc_id;
  int32_t raw1, raw2;
};
struct MagPacket {
  uint64_t ts_us;
  int16_t mx, my, mz;
};
struct ImuPacket {
  uint64_t ts_us;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

void adsService(char ads_id) {
  const int i = ads_id - 'A';                     // Array index of sensor (A=0, B=1...)
  uint32_t service_time = micros();    // Record the time the ADC value was reported by DRDY interrupt pin. 
                                                  // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);             // Get SPI open with settings
  digitalWrite(ADS_sensors[i].cs_pin, LOW);       // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                           // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                                // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);                   // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                           // Allow communication to finish
  // Close SPI
  digitalWrite(ADS_sensors[i].cs_pin, HIGH);      // Release sensor from SPI
  SPI.endTransaction();                           // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // ADS1220 can only store one value, so manually switch and track between channels
  if (current_channels[i] == 0) {
    raw_values[i][0] = val;
    adcs[i].select_mux_channels(MUX_AIN2_AIN3);
    current_channels[i] = 1;
  } 
  else {
    raw_values[i][1] = val;
    adcs[i].select_mux_channels(MUX_AIN0_AIN1);
    current_channels[i] = 0;

    // When second channel is read, record time and mark sensor's bit in bitmask as ready (1)
    timestamps[i] = service_time - time_init;
    ready_mask |= (1 << i);
  }
}

void IRAM_ATTR handleDrdyA() { adsService('A'); }
void IRAM_ATTR handleDrdyB() { adsService('B'); }
void IRAM_ATTR handleDrdyC() { adsService('C'); }
void IRAM_ATTR handleDrdyD() { adsService('D'); }
void IRAM_ATTR handleDrdyE() { adsService('E'); }
void IRAM_ATTR handleDrdyF() { adsService('F'); }

// Broadcast a command to all active sensors
void broadcast_command(uint8_t cmd) {
  // Lower all CS pins for the active sensors to broadcast the command to all chips.
  // We loop only over NUM_SENSORS to avoid affecting unused pins.
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(ADS_sensors[i].cs_pin, LOW);
  }
  SPI.transfer(cmd);  // Send the command to all sensors at once
  // Raise all CS pins for active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(ADS_sensors[i].cs_pin, HIGH);
  }
}

// Disable interrupts for sensor DRDY pins
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    detachInterrupt(digitalPinToInterrupt(ADS_sensors[i].drdy_pin));
  }
}

// Enable interrupts for sensor DRDY pins
void enableADCInterrupts() {
  void (*isrHandlers[MAX_SENSORS])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE, handleDrdyF};
  for (int i = 0; i < NUM_SENSORS; i++) {
    attachInterrupt(digitalPinToInterrupt(ADS_sensors[i].drdy_pin), isrHandlers[i], FALLING);
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
    adcs[i].begin(ADS_sensors[i].cs_pin, ADS_sensors[i].drdy_pin);
    adcs[i].set_OperationMode(MODE_TURBO);
    adcs[i].set_data_rate(dr_code);
    adcs[i].PGA_ON();
    adcs[i].set_pga_gain(PGA_GAIN_128);
    adcs[i].set_VREF(VREF_ANALOG);
    adcs[i].set_conv_mode_continuous();
    adcs[i].select_mux_channels(MUX_AIN0_AIN1);
    current_channels[i] = 0;

    if (hasSerial) {
      Serial.print("Setup complete for Sensor [enumerated as: "); Serial.print(i); Serial.println("]");
      Serial.print("CS: "); Serial.print(ADS_sensors[i].cs_pin); 
      Serial.print(" DRDY: "); Serial.println(ADS_sensors[i].drdy_pin);
    }
    delay(100);
  }
  if (hasSerial) Serial.println("Starting datastream from sensors");

  // Allow settling
  delay(100);               

  // Staggered START: Evenly space within one conversion cycle, assuming 5 sensors max
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
      if (hasSerial) Serial.println("Invalid DR code; using default 180 SPS");
      turbo_sps = 180;
  }
  unsigned long period_us = 1000000UL / turbo_sps;  // Conversion period in µs
  unsigned long stagger_us = period_us / 5;  // Step for 5 sensors

  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS - 1) {
      delayMicroseconds(stagger_us);
    }
  }

  // Set time=0 for datastream after letting ADS1220 modules stabilize
  enableADCInterrupts();
  delay(50);
  time_init = micros();
}

// Handle OTA updates (called only in non-CONNECTED states)
void handleOTA() {
  ArduinoOTA.handle();
}

// Check if all data pairs are ready for this cycle
bool checkDataReady() {
  uint8_t all_ready_mask = (1U << NUM_SENSORS) - 1;  // Local compile-time constant. Stored in CPU stack for compare, not created in and read from RAM.
  if (ready_mask == all_ready_mask) {
    ready_mask = 0;   // reset all bits in the mask to 0
    return true;
  }
  return false;
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

// Build a binary packet from current data and add to queue
void queueDataPacket() {
  // Calculate packet payload length: 1 byte ID + sensors * (4 ts + 4 raw1 + 4 raw2)
  size_t payload_len = 1 + NUM_SENSORS * 12;

  // Build payload separately
  std::vector<uint8_t> payload;
  payload.reserve(payload_len);

  // Add nano_id as uint8_t
  uint8_t nano_id = atoi(NANO_ID);
  payload.push_back(nano_id);

  // Per sensor: uint32_t ts_us, int32_t raw1, int32_t raw2
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint32_t ts_us = (uint32_t)timestamps[i];  // Cast to 32-bit (matches ESP32 unsigned long)
    const uint8_t* ts_ptr = reinterpret_cast<const uint8_t*>(&ts_us);
    payload.insert(payload.end(), ts_ptr, ts_ptr + sizeof(uint32_t));

    const uint8_t* raw1_ptr = reinterpret_cast<const uint8_t*>(&raw_values[i][0]);
    payload.insert(payload.end(), raw1_ptr, raw1_ptr + sizeof(int32_t));

    const uint8_t* raw2_ptr = reinterpret_cast<const uint8_t*>(&raw_values[i][1]);
    payload.insert(payload.end(), raw2_ptr, raw2_ptr + sizeof(int32_t));
  }

  // Compute CRC over payload
  uint16_t crc = compute_crc(payload.data(), payload.size());

  // Build full packet: length (2 bytes) + seq (2) + crc (2) + payload
  std::vector<uint8_t> packet;
  packet.reserve(6 + payload_len);

  // Length prefix (uint16_t, little-endian)
  uint16_t length = static_cast<uint16_t>(payload_len);
  packet.push_back(static_cast<uint8_t>(length & 0xFF));
  packet.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));

  // Sequence number (uint16_t, little-endian)
  packet.push_back(static_cast<uint8_t>(seq_num & 0xFF));
  packet.push_back(static_cast<uint8_t>((seq_num >> 8) & 0xFF));
  seq_num++;  // Increment for next packet

  // CRC (uint16_t, little-endian)
  packet.push_back(static_cast<uint8_t>(crc & 0xFF));
  packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  // Append payload
  packet.insert(packet.end(), payload.begin(), payload.end());

  // Add to queue
  packet_queue.push_back(std::move(packet));
}

// Send queued packets over persistent TCP
bool sendQueuedDataTCP() {
  if (!client.connected()) {
    if (hasSerial) Serial.println("TCP not connected; skipping send");
    return false;
  }

  bool sent_any = false;
  while (!packet_queue.empty()) {
    const auto& packet = packet_queue.front();
    if (client.write(packet.data(), packet.size()) != packet.size()) {
      if (hasSerial) Serial.println("Partial or failed packet send");
      return false;  // Stop on error to avoid out-of-order
    }
    packet_queue.pop_front();
    sent_any = true;
  }

  if (sent_any) {
    client.flush();  // Ensure data is sent
  }
  return true;
}

// Send individual packet over wired serial, if connected
void sendDataSerial() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print(timestamps[i] / 1000000.0, 6);
    Serial.print(",");
    Serial.print(raw_values[i][0]);
    Serial.print(",");
    Serial.print(raw_values[i][1]);
  }
  Serial.println();
}

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
        if (hasSerial) Serial.println("Persistent TCP connected to host");
        WiFiState = CONNECTED;
        initializeADCs();  // Reset ADCs and time for new session
        ready_mask = 0;
        packet_queue.clear();  // Clear any stale packets
        ArduinoOTA.end();
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
      ArduinoOTA.begin();
      if (hasSerial) Serial.println("Returning to CONNECTING state with OTA re-enabled.");
    }
  }
}

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
    Serial.println(" "); 
    Serial.println("Serial connected for debugging");
    Serial.print("Nano_ID: ");
    Serial.println(NANO_ID);
  }

  // SPI init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // WiFi Station mode setup
  if (hasSerial) {
    Serial.println("Starting station mode. Arduino is WiFi client looking for following network...");
    Serial.print("SSID: "); Serial.print(ssid);
    Serial.print(". Password: "); Serial.println(password);
    Serial.println("Scanning for available networks...");
  }

  WiFi.mode(WIFI_STA);
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

  // OTA setup
  // ArduinoOTA.setHostname("Hi-STIFFS_Nano");
  // ArduinoOTA.setPassword(ota_password);
  // ArduinoOTA.begin();
  // if (hasSerial) Serial.println("OTA ready. Connect to WiFi and use OTA for updates.");

  // ADS1220 initialization (but interrupts disabled until connected)
  initializeADCs();
  disableADCInterrupts();

  // Set initial state based on Serial
  if (hasSerial) { 
    Serial.print("Initial state: ");
    Serial.println(hasSerial ? "HAS_SERIAL" : "NO_SERIAL");
  }

  last_batch_send = millis();
}

void loop() {
  // State-specific actions
  switch (WiFiState) {
    case CONNECTED: { // Connected, do not allow OTA updates and send datastream to host 
      if (checkDataReady()) {
        queueDataPacket();  // Build and queue binary packet
        if (hasSerial) sendDataSerial();   // Also send over serial for debugging
      }
      // Check if time to send batch
      unsigned long now = millis();
      if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
        sendQueuedDataTCP();
        last_batch_send = now;
      }
      monitorConnection();
    } break;        
    case CONNECTING: { // Not connected, but attempt connection and allow OTA updates
      disableADCInterrupts();
      handleOTA();
      connectToHost();
    } break;
  }
}