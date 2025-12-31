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
//    Key Decisions and Justifications:
//    - Modular Design: Use a configurable number of sensors (NUM_SENSORS) and an array
//      of configurations. To use fewer sensors (e.g., only A-C), set NUM_SENSORS=3. 
//      The all_configs array lists all possible configurations in order (A to E). 
//      The code will only use the first NUM_SENSORS entries from this array. This allows 
//      easy enabling/disabling by just changing NUM_SENSORS without commenting out lines 
//      in the array initializer. If you need to change pin assignments or IDs, edit the 
//      all_configs array directly.
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
//      - Data rate set to DR_175SPS, which becomes 350 SPS in turbo mode for 175sps per channel
//        (per ADS1220 datasheet, Table 11).
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
//      - The two channels per chip are ~1/data_rate apart (~5.5ms at 180Hz), which is "reasonably synchronized".
//      - Single timestamp (micros()) taken after reading the second channel, applied to both. This approximates
//        the pair as synchronized.
//      - Justification: Simultaneous conversion impossible with single ADC. This method minimizes delay.
//    - Synchronization Between Sensors: 
//      - Not implemented. Sensors run independently (internal oscillators may drift slightly over time, ~2% per datasheet). 
//      - If needed, add periodic START/SYNC commands to all chips (requires careful handling to avoid MISO conflicts).
//    - Output Format: 
//      - Send data only when all sensors are ready (a full cycle), in a
//        single efficient CSV line: tsA,A1_raw,A2_raw,tsB,B1_raw,B2_raw,... Each sensor has its
//        own timestamp (taken after reading its second channel). Timestamps are always per-sensor to account for any minor delays.
//      - The _raw values are the 24-bit signed integers directly from the ADC (sign-extended to 32-bit for printing).
//        Previously, these were converted to mV using the formula: (float(val) * VREF / PGA_GAIN / (1LL<<23)) * 1000.0f,
//        but now we send the raw integers for flexibility (conversion can be done later if needed).
//      - Justification: Ensures efficient serial transfer (one packet per cycle) while providing
//        unique timestamps per sensor as specified. High baud rate (2M) minimizes overhead.
//    - Other Settings: 
//      - PGA=128 (Programmable Gain Amplifier), internal VREF=2.048V (from datasheet). Continuous mode for steady sampling.
//    - Interrupt Handling: 
//      - Use lambdas to pass sensor index to handler (C++11, supported in Arduino).
//      - Note: In the code, we use a fixed array of 5 ISR handlers (handleDrdy0 to handleDrdy4) because 
//        attaching interrupts with lambdas that capture variables can be tricky on ESP32 (especially with IRAM_ATTR).
//        We define all 5 handlers even if NUM_SENSORS < 5, but only attach the first NUM_SENSORS. This is simple 
//        and avoids issues with captured lambdas in interrupts.
//    - Error Handling: 
//      - Basic; assumes DRDY triggers reliably.
//
//    Dependencies: Protocentral_ADS1220 library (include .h and .cpp as provided).
//
//    For information on ADS1220, see datasheet. For Nano ESP32 pins, see provided PDF.
//
//    This software is licensed under the MIT License (as in original library).
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <SPI.h>

#define FULL_SCALE (1LL << 23) // 2^23 for 24-bit signed scaling - retained for reference, though not used in raw output

const int MAX_SENSORS = 5;     // Maximum possible sensors (A to E)
const int NUM_SENSORS = 2;     // Set to 1-5 to use the first N sensors from all_configs below.
// Justification: This makes it easy to change the number of sensors - just update this constant,
// and the code will initialize and use only the first NUM_SENSORS configurations.

struct SensorConfig {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int cs_pin;                  // Chip Select pin
  int drdy_pin;                // Data Ready pin
};

// Full list of configurations for sensors A to E. The code uses only the first NUM_SENSORS.
// To change pins or IDs, edit this array. No commenting out needed - just set NUM_SENSORS.
SensorConfig all_configs[MAX_SENSORS] = {
  {'A', 10, 5},                // A: CS=D10, DRDY=D5
  {'B', 9, 4},                 // B: CS=D9, DRDY=D4
  {'C', 8, 3},                 // C: CS=D8, DRDY=D3
  {'D', 7, 2},                 // D: CS=D7, DRDY=D2
  {'E', 6, A6}                 // E: CS=D6, DRDY=A6 (A6 as digital input)
};

Protocentral_ADS1220 adcs[NUM_SENSORS];
volatile bool drdy_flags[NUM_SENSORS] = {false};
int32_t raw_values[NUM_SENSORS][2];    // [sensor][channel]: raw 24-bit signed ADC values; 0=ch1 (AIN0-1), 1=ch2 (AIN2-3)
unsigned long timestamps[NUM_SENSORS]; // Per-chip timestamps for each pair
uint8_t current_channels[NUM_SENSORS] = {0};
volatile bool pair_ready[NUM_SENSORS] = {false};
volatile unsigned long interrupt_times[NUM_SENSORS]; // Captures micros() at ISR entry for each DRDY interrupt
unsigned long time_init;

void broadcast_command(uint8_t cmd) {
  // Lower all CS pins for the active sensors to broadcast the command to all chips.
  // We loop only over NUM_SENSORS to avoid affecting unused pins.
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, LOW);
  }
  SPI.transfer(cmd);  // Send the command (e.g., START)
  // Raise all CS pins for active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, HIGH);
  }
}

// Interrupt handlers for each possible sensor (0 to 4, corresponding to A to E).
// These are defined as separate functions to ensure compatibility with attachInterrupt on ESP32.
// We define all 5 even if NUM_SENSORS < 5, but only attach the ones we use.
// The IRAM_ATTR attribute ensures they can be called from interrupt context efficiently.
void IRAM_ATTR handleDrdy0() { drdy_flags[0] = true; interrupt_times[0] = micros();}
void IRAM_ATTR handleDrdy1() { drdy_flags[1] = true; interrupt_times[1] = micros();}
void IRAM_ATTR handleDrdy2() { drdy_flags[2] = true; interrupt_times[2] = micros();}
void IRAM_ATTR handleDrdy3() { drdy_flags[3] = true; interrupt_times[3] = micros();}
void IRAM_ATTR handleDrdy4() { drdy_flags[4] = true; interrupt_times[4] = micros();}

void setup() {
  Serial.begin(2000000);  // High baud rate for efficient serial transfer
  while (!Serial);        // Wait for Serial (USB) to initialize

  SPI.begin();            // Initialize SPI bus (D13=SCK, D11=MOSI, D12=MISO)

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Initialize each ADS1220 instance using the config from all_configs
    adcs[i].begin(all_configs[i].cs_pin, all_configs[i].drdy_pin);

    // Set turbo mode for higher data rates
    adcs[i].set_OperationMode(MODE_TURBO);

    // Set data rate to 350 SPS in turbo (175 Hz per channel with muxing)
    adcs[i].set_data_rate(DR_175SPS);

    // Set gain (matches example)
    adcs[i].set_pga_gain(PGA_GAIN_128);

    // Set external reference for ratiometric measurement
    adcs[i].set_VREF(VREF_ANALOG);

    // Set continuous conversion mode
    adcs[i].set_conv_mode_continuous();

    // Start with this sensor's channel 1 (AIN0-AIN1)
    adcs[i].select_mux_channels(MUX_AIN0_AIN1);

    // Attach interrupt for DRDY. We use a fixed array of handler functions.
    // The array has 5 entries (for max sensors), but we only attach up to NUM_SENSORS.
    // This syntax creates an array of function pointers: isrHandlers[0] points to handleDrdy0, etc.
    // Justification: Simple way to map index i to the correct handler without complex lambdas.
    void (*isrHandlers[5])() = {handleDrdy0, handleDrdy1, handleDrdy2, handleDrdy3, handleDrdy4}; // Fixed at max num_sensors. Extras are created, but unused.
    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isrHandlers[i], FALLING);
  }

  // Broadcast START to all active chips for synchronized start
  broadcast_command(START);

  time_init = micros();   // Reference time for timestamps
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (drdy_flags[i]) {
      drdy_flags[i] = false;  // Clear flag

      // Read the current sample (raw 24-bit signed value)
      // This is channel 1 or channel 2 depending on current multiplexer selection
      // Multiplexer switching is handled below
      int32_t val = adcs[i].Read_Data_Samples();

      if (current_channels[i] == 0) {
        // Just read channel 1; store raw value and switch to channel 2
        // No conversion to mV - we store the raw integer directly.
        raw_values[i][0] = val;
        adcs[i].select_mux_channels(MUX_AIN2_AIN3);
        current_channels[i] = 1;
      } else {
        // Just read channel 2; store raw value, switch back to channel 1, set ready
        raw_values[i][1] = val;
        adcs[i].select_mux_channels(MUX_AIN0_AIN1);
        current_channels[i] = 0;

        // Pair complete; timestamp and mark ready
        timestamps[i] = interrupt_times[i] - time_init; // Use time at DRDY interrupt of second channel
        pair_ready[i] = true;
      }
    }
  }

  // Check if all sensors have a new pair ready
  bool all_ready = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!pair_ready[i]) {
      all_ready = false;
      break;
    }
  }

  if (all_ready) {
    // Send single packet with all data (CSV format for easy parsing)
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i > 0) Serial.print(",");
      Serial.print(timestamps[i] / 1000000.0, 6);  // Convert μs to seconds as float, 6 decimal places
      Serial.print(",");
      Serial.print(raw_values[i][0]);  // Print raw integer (no decimal places needed)
      Serial.print(",");
      Serial.print(raw_values[i][1]);  // Print raw integer
    }
    Serial.println();

    // Reset ready flags for next cycle
    for (int i = 0; i < NUM_SENSORS; i++) {
      pair_ready[i] = false;
    }
  }
}