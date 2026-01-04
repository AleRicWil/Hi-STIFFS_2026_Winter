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
//      - The all_configs array lists all possible configurations in order (A to E). 
//      - The code will only use the first NUM_SENSORS entries from this array. This allows 
//        easy enabling/disabling by just changing NUM_SENSORS without commenting out lines 
//        in the array initializer. If you need to change pin assignments or IDs, edit the 
//        all_configs array directly.
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
//      - Now outputs over WiFi using Server-Sent Events (SSE) on a simple HTTP server (port 80) for continuous streaming.
//        Connect via browser or client to http://<SoftAP_IP> (default 192.168.4.1) to receive real-time updates.
//      - Serial output retained for debugging (with timeout to avoid blocking if no USB connected).
//      - Justification: Enables untethered field operation; SSE provides efficient, non-blocking streaming compatible with Python clients.
//        High baud rate serial fallback for wired debugging.
//    - Wireless Features:
//      - SoftAP mode: Nano creates its own WiFi network (configurable SSID/password) for direct connection without router.
//      - OTA: Supports over-the-air sketch uploads via WiFi for wireless updates.
//      - Power Impact: SoftAP adds ~100-200mA draw; monitor for battery-powered setups (see Nano ESP32 manual, Section 11).
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
//    - Finite State Machine (FSM): Manages connection states to listen for clients only when idle,
//      send data when connected, and handle Serial presence determined at setup. States: LISTENING,
//      CONNECTED, SERIAL_ONLY, NO_SERIAL. This ensures efficient resource use on Nano ESP32.
//    - Modular Functions: Broke out key operations (e.g., OTA handling, DRDY processing, data sending)
//      for tight, efficient code. Prepares for higher data rates by minimizing loop overhead.
//    - OTA Restriction: Disabled during active connections to prevent disruptions.
//    - ADC Reset: On new connection, reset ADCs and timestamps for session synchronization.
//    - Error Gracefulness: Added checks for missed DRDY (e.g., skip if flag not set), client errors
//      (stay in state), and suppressed Serial if absent to avoid blocking.
//    - Efficiency for Nano ESP32: Used IRAM_ATTR for ISRs, non-blocking loops, conditional debug
//      prints. Aligned with PDF specs (e.g., low power notes for future optimizations, pin capabilities).
//    - Future-Proofing: Code structure allows easy data rate increases (e.g., to DR_1000SPS in turbo,
//      ~2000 SPS total) once tested; focus on tight execution to handle higher interrupt rates.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <SPI.h>
#include <WiFi.h>       // For WiFi SoftAP
#include <ArduinoOTA.h> // For Over-The-Air updates

#define FULL_SCALE (1LL << 23) // 2^23 for 24-bit signed scaling - retained for reference, though not used in raw output
#define A_CS_PIN 10
#define B_CS_PIN 9
#define C_CS_PIN 8
#define D_CS_PIN 7
#define E_CS_PIN 6
#define A_DRDY_PIN 5
#define B_DRDY_PIN 4
#define C_DRDY_PIN 3
#define D_DRDY_PIN 2
#define E_DRDY_PIN A6

const int MAX_SENSORS = 5;     // Maximum possible sensors (A to E)
const int NUM_SENSORS = 1;     // Set to 1-5 to use the first N sensors from all_configs below.
const uint8_t dr_code = DR_90SPS;  // Data Rate value. In turbo, value is for pairs/sec. In normal, value is for samples/sec
const SPISettings spi_settings(2000000, MSBFIRST, SPI_MODE1);

// WiFi access point
const char* ssid = "Hi-STIFFS_Nano";       // WiFi network name (SSID) - choose something unique
const char* password = "BYUCropBio";       // WiFi password (minimum 8 characters)
const char* ota_password = "BYUCropBio";   // Optional OTA password for security (change this)
// WiFi server for SSE streaming
WiFiServer server(80);
WiFiClient streamingClient;  // Global client for ongoing SSE connection

struct SensorConfig {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int cs_pin;                  // Chip Select pin
  int drdy_pin;                // Data Ready pin
};

// Full list of configurations for sensors A to E. The code uses only the first NUM_SENSORS.
// To change pins or IDs, edit this array. No commenting out needed - just set NUM_SENSORS.
SensorConfig all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];             // ADC objects from library
int32_t raw_values[MAX_SENSORS][2];                 // [sensor][channel]: raw 24-bit signed ADC values; 0=ch1 (AIN0-1), 1=ch2 (AIN2-3)
unsigned long timestamps[MAX_SENSORS];              // Per-chip timestamps for each pair
uint8_t current_channels[MAX_SENSORS] = {0};        // Indicates which MUX channel to read from for an ADS1220 module
volatile uint8_t ready_mask = 0;                    // Bitmask tracking ready sensors (bit i set to (1) when sensor i pair is complete)
unsigned long time_init;                            // t=0 of datastream. Set each time Wi-Fi client initiates datastream
const unsigned long WiFi_CHECK_INTERVAL_MS = 1000;  // 1 Hz

// FSM States
enum State_Serial {
  NO_SERIAL,    // No Serial connection
  HAS_SERIAL,   // Serial connection active
};
State_Serial SerialState;

enum State_WiFi {
  LISTENING,    // No active client, listening for connections
  CONNECTED,    // Active client, sending data, no listening/OTA
};
State_WiFi WiFiState;

// Global flag for Serial presence, set in setup
bool hasSerial = false;

// Interrupt handlers for each possible sensor (0 to 4, corresponding to A to E).
// These are defined as separate functions to ensure compatibility with attachInterrupt on ESP32.
// We define all 5 even if NUM_SENSORS < 5, but only attach the ones we use.
// The IRAM_ATTR attribute ensures they can be called from interrupt context efficiently.
// All read functionality must be internal to each interrupt (no external SPI-reading functions or global reads).
// Write functionality can be to external and using external functions.
void IRAM_ATTR handleDrdyA() {
  const int i = 0;                          // Array index of sensor
  unsigned long interrupt_time = micros();  // Record the time the ADC value was reported by DRDY interrupt pin. 
                                            // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);       // Get SPI open with settings
  digitalWrite(A_CS_PIN, LOW);                // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                     // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                          // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);             // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                     // Allow communication to finish
  // Close SPI
  digitalWrite(A_CS_PIN, HIGH);               // Release sensor from SPI
  SPI.endTransaction();                     // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // // Corruption detection (expanded per datasheet valid range: -2^23 to 2^23-1)
  // if (val == -1 || val == 0 || abs(val) > 8388607LL) {
  //   adcs[i].select_mux_channels(MUX_AIN0_AIN1);  // Reset to ch1
  //   current_channels[i] = 0;
  //   ready_mask |= (0 << i);  // mark sensor's bit in bitmask as not-ready (0)
  //   return;  // Skip this read (creates data hole)
  // }

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
    timestamps[i] = interrupt_time - time_init;
    ready_mask |= (1 << i);
  }
}
void IRAM_ATTR handleDrdyB() {
  const int i = 1;                          // Array index of sensor
  unsigned long interrupt_time = micros();  // Record the time the ADC value was reported by DRDY interrupt pin. 
                                            // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);       // Get SPI open with settings
  digitalWrite(B_CS_PIN, LOW);                // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                     // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                          // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);             // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                     // Allow communication to finish
  // Close SPI
  digitalWrite(B_CS_PIN, HIGH);               // Release sensor from SPI
  SPI.endTransaction();                     // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // // Corruption detection (expanded per datasheet valid range: -2^23 to 2^23-1)
  // if (val == -1 || val == 0 || abs(val) > 8388607LL) {
  //   adcs[i].select_mux_channels(MUX_AIN0_AIN1);  // Reset to ch1
  //   current_channels[i] = 0;
  //   ready_mask |= (0 << i);  // mark sensor's bit in bitmask as not-ready (0)
  //   return;  // Skip this read (creates data hole)
  // }

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
    timestamps[i] = interrupt_time - time_init;
    ready_mask |= (1 << i);
  }
}
void IRAM_ATTR handleDrdyC() {
  const int i = 2;                          // Array index of sensor
  unsigned long interrupt_time = micros();  // Record the time the ADC value was reported by DRDY interrupt pin. 
                                            // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);       // Get SPI open with settings
  digitalWrite(C_CS_PIN, LOW);                // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                     // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                          // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);             // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                     // Allow communication to finish
  // Close SPI
  digitalWrite(C_CS_PIN, HIGH);               // Release sensor from SPI
  SPI.endTransaction();                     // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // // Corruption detection (expanded per datasheet valid range: -2^23 to 2^23-1)
  // if (val == -1 || val == 0 || abs(val) > 8388607LL) {
  //   adcs[i].select_mux_channels(MUX_AIN0_AIN1);  // Reset to ch1
  //   current_channels[i] = 0;
  //   ready_mask |= (0 << i);  // mark sensor's bit in bitmask as not-ready (0)
  //   return;  // Skip this read (creates data hole)
  // }

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
    timestamps[i] = interrupt_time - time_init;
    ready_mask |= (1 << i);
  }
}
void IRAM_ATTR handleDrdyD() {
  const int i = 3;                          // Array index of sensor
  unsigned long interrupt_time = micros();  // Record the time the ADC value was reported by DRDY interrupt pin. 
                                            // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);       // Get SPI open with settings
  digitalWrite(D_CS_PIN, LOW);                // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                     // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                          // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);             // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                     // Allow communication to finish
  // Close SPI
  digitalWrite(D_CS_PIN, HIGH);               // Release sensor from SPI
  SPI.endTransaction();                     // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // // Corruption detection (expanded per datasheet valid range: -2^23 to 2^23-1)
  // if (val == -1 || val == 0 || abs(val) > 8388607LL) {
  //   adcs[i].select_mux_channels(MUX_AIN0_AIN1);  // Reset to ch1
  //   current_channels[i] = 0;
  //   ready_mask |= (0 << i);  // mark sensor's bit in bitmask as not-ready (0)
  //   return;  // Skip this read (creates data hole)
  // }

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
    timestamps[i] = interrupt_time - time_init;
    ready_mask |= (1 << i);
  }
}
void IRAM_ATTR handleDrdyE() {
  const int i = 4;                          // Array index of sensor
  unsigned long interrupt_time = micros();  // Record the time the ADC value was reported by DRDY interrupt pin. 
                                            // Technically the time when the interrupt is processed, as interrupts on same core of same priority form queue
  // Open SPI with sensor's ADS1220 chip/module
  SPI.beginTransaction(spi_settings);       // Get SPI open with settings
  digitalWrite(E_CS_PIN, LOW);                // Select particular sensor (cs='chip select')
  delayMicroseconds(1);                     // Allow ADS1220 module to accept connection (50ns on datasheet)
  // Retrieve 3 byte (24-bit) ADC result
  byte SPI_Buf[3];                          // temporary local buffer to work with raw result before storing integer value
  SPI_Buf[0] = SPI.transfer(0);             // Send dummy 0x00 byte on MOSI and receive one conversion byte on MISO
  SPI_Buf[1] = SPI.transfer(0);
  SPI_Buf[2] = SPI.transfer(0);
  delayMicroseconds(1);                     // Allow communication to finish
  // Close SPI
  digitalWrite(E_CS_PIN, HIGH);               // Release sensor from SPI
  SPI.endTransaction();                     // Release Nano's SPI bus

  long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2]; // Assemble 3 bytes into single 24-bit object (using 32-bit datatype)
  int32_t val = (bits24 << 8) >> 8;                                          // Sign-extend 24-bit to 32-bit. Copy sign bit 23 to bits 31-24 

  // // Corruption detection (expanded per datasheet valid range: -2^23 to 2^23-1)
  // if (val == -1 || val == 0 || abs(val) > 8388607LL) {
  //   adcs[i].select_mux_channels(MUX_AIN0_AIN1);  // Reset to ch1
  //   current_channels[i] = 0;
  //   ready_mask |= (0 << i);  // mark sensor's bit in bitmask as not-ready (0)
  //   return;  // Skip this read (creates data hole)
  // }

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
    timestamps[i] = interrupt_time - time_init;
    ready_mask |= (1 << i);
  }
}

// Broadcast a command to all active sensors
void broadcast_command(uint8_t cmd) {
  // Lower all CS pins for the active sensors to broadcast the command to all chips.
  // We loop only over NUM_SENSORS to avoid affecting unused pins.
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, LOW);
  }
  SPI.transfer(cmd);  // Send the command to all sensors at once
  // Raise all CS pins for active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, HIGH);
  }
}

// Disable interrupts for sensor CS pins
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
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
  void (*isrHandlers[5])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE};
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

    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isrHandlers[i], FALLING);

    if (hasSerial) {
      Serial.print("Setup complete for Sensor [enumerated as: ");
      Serial.print(i);
      Serial.println("]");
    }
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
  delay(100);
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
    ready_mask = 0;   // resest all bits in the mask to 0
    return true;
  }
  return false;
}

// Send data over WiFi to single client
void sendDataWiFi() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) streamingClient.print(",");
    streamingClient.print(timestamps[i] / 1000000.0, 6);
    streamingClient.print(",");
    streamingClient.print(raw_values[i][0]);
    streamingClient.print(",");
    streamingClient.print(raw_values[i][1]);
  }
  streamingClient.println();
  streamingClient.println();  // Double newline for SSE event
  streamingClient.flush();
}

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

// Listen for new WiFi client connections. No w/Serial version because no datastream when listening
void listenForClient() {
  WiFiClient client = server.available();
  if (client) {
    if (hasSerial) Serial.println("New Client.");
    String currentLine = "";
    bool headersComplete = false;
    unsigned long timeout = millis() + 5000;  // 5s timeout for header read
    while (client.connected() && millis() < timeout && !headersComplete) {
      if (client.available()) {
        char c = client.read();
        if (hasSerial) Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Headers complete; send SSE response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/event-stream");
            client.println("Cache-Control: no-cache");
            client.println("Connection: keep-alive");
            client.println();
            headersComplete = true;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    if (headersComplete) {
      streamingClient = client;
      initializeADCs();  // Reset ADCs and time for new session
      // Reset ready mask so old data doesn't get sent
      ready_mask = 0;
      WiFiState = CONNECTED;
      ArduinoOTA.end();
      if (hasSerial) Serial.println("Entering CONNECTED state.");
    } 
    else {
      // Error: Timeout or incomplete headers; close and stay in state
      client.stop();
      if (hasSerial) Serial.println("Client connection failed.");
    }
  }
}

// Monitor active client for disconnection
void monitorClient() {
  static unsigned long lastConnectionCheck = 0;
  unsigned long now = millis();
  if (now - lastConnectionCheck >= WiFi_CHECK_INTERVAL_MS) {
    lastConnectionCheck = now;

    if (!streamingClient.connected()) {
      streamingClient.stop();
      WiFiState = LISTENING;  // Return to listening
      ArduinoOTA.begin();     // Re-enable OTA (safe to call repeatedly)
    }
  }
  // No else needed — check skipped otherwise
}

// Monitor active client for disconnection, send debug over serial port
void monitorClient_wSerial() {
  static unsigned long lastConnectionCheck = 0;
  unsigned long now = millis();
  if (now - lastConnectionCheck >= WiFi_CHECK_INTERVAL_MS) {
    lastConnectionCheck = now;

    if (!streamingClient.connected()) {
      if (hasSerial) Serial.println("Streaming client disconnected.");
      streamingClient.stop();
      WiFiState = LISTENING;  // Return to listening
      ArduinoOTA.begin();     // Re-enable OTA (safe to call repeatedly)
      if (hasSerial) Serial.println("Returning to LISTENING state with OTA re-enabled.");
    }
  }
  // No else needed — check skipped otherwise
}

void setup() {
  // Serial detection and init
  Serial.begin(2000000);
  unsigned long startTime = millis();
  while (!Serial) {
    if (millis() - startTime > 5000) {
      // Continuing without serial
      break;
    }
  }
  hasSerial = Serial;  // Set flag based on connection
  if (hasSerial) Serial.println(" "); Serial.println("Serial connected for debugging");

  // SPI init
  SPI.begin();

  // WiFi SoftAP setup
  WiFi.softAP(ssid, password);
  if (hasSerial) {
    Serial.println("Starting SoftAP (wireless access point)...");
    Serial.print("SSID: ");
    Serial.print(ssid);
    Serial.print(". Password: ");
    Serial.print(password);
    Serial.print(". SoftAP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  // Start web server
  server.begin();
  if (hasSerial) Serial.println("Web server started for data streaming");

  // OTA setup
  ArduinoOTA.setHostname(ssid);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.begin();
  if (hasSerial) Serial.println("OTA ready. Connect to WiFi and use OTA for updates.");

  // ADS1220 initialization
  initializeADCs();
  disableADCInterrupts();

  // Set initial state based on Serial
  SerialState = hasSerial ? HAS_SERIAL : NO_SERIAL;
  if (hasSerial) { 
    Serial.print("Initial state: ");
    Serial.println(SerialState == HAS_SERIAL ? "HAS_SERIAL" : "NO_SERIAL");
  }
}

void loop() {

  // State-specific actions
  switch (SerialState) {
    case NO_SERIAL: // Do nothing with the serial port
      
      switch (WiFiState) {
        case CONNECTED:
          if (checkDataReady()) sendDataWiFi();
          monitorClient();
          break;
        case LISTENING:
          disableADCInterrupts();
          handleOTA();
          listenForClient();
          break;

      }
      break;
    
    case HAS_SERIAL:  // Send data over serial port at all times
      
      if (checkDataReady()) sendDataSerial();
      switch (WiFiState) {
        case CONNECTED: // Connected, do not allow OTA updates or new clients and send datastream to client 
          if (checkDataReady()) sendDataWiFi();
          monitorClient_wSerial();
          break;        
        case LISTENING: // No connection, but listen for OTA updates and datastream client
          disableADCInterrupts();
          handleOTA();
          listenForClient();
          break;
      }
      break;
  }

}