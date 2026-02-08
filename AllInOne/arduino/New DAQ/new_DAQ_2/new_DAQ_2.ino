//////////////////////////////////////////////////////////////////////////////////////////
//
//    Demo code for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    This example gives differential voltage across the AN0 and AN1 pins in mVolts
//
//    Arduino connections:
//
//  |ADS1220 pin label| Pin Function         |Arduino Connection|
//  |-----------------|:--------------------:|-----------------:|
//  | DRDY            | Data ready Output pin|  D02             |
//  | MISO            | Slave Out            |  D12             |
//  | MOSI            | Slave In             |  D11             |
//  | SCLK            | Serial Clock         |  D13             |
//  | CS              | Chip Select          |  D7              |
//  | DVDD            | Digital VDD          |  +5V             |
//  | DGND            | Digital Gnd          |  Gnd             |
//  | AN0-AN3         | Analog Input         |  Analog Input    |
//  | AVDD            | Analog VDD           |  -               |
//  | AGND            | Analog Gnd           |  -               |
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <SPI.h>

#define PGA 128                // Programmable Gain, confirm that the same as set_pga_gain
#define VREF 5.12            
#define VFSR VREF/PGA
#define FSR (((long int)1<<23)-1)

#define ADS1220_CS_PIN    7
#define ADS1220_DRDY_PIN  2

Protocentral_ADS1220 pc_ads1220;
int32_t adcValue01;  // Channel 1 (AIN0-AIN1)
int32_t adcValue23;  // Channel 2 (AIN2-AIN3)
volatile bool drdyIntrFlag = false;
unsigned long time_init;
uint8_t currentChannel = 0;  // 0 for AIN0-AIN1, 1 for AIN2-AIN3

void drdyInterruptHndlr() {
  drdyIntrFlag = true;
}

void enableInterruptPin() {
  attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), drdyInterruptHndlr, FALLING);
}

void setup() {
    Serial.begin(115200);

    pc_ads1220.begin(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
  
    // Optimized for low noise/drift at ~165 Hz/channel
    pc_ads1220.set_data_rate(DR_175SPS);          // 330 SPS total (normal mode)
    pc_ads1220.set_pga_gain(PGA_GAIN_128);        // High gain for small bridge signals
    pc_ads1220.PGA_ON();                          // Enable PGA
    pc_ads1220.set_VREF(VREF_ANALOG);             // Ratiometric: VREF = AVDD-AVSS (5V)
    pc_ads1220.set_FIR_Filter(FIR_OFF);           // No FIR (max speed; enable FIR_50Hz if line noise)
    pc_ads1220.set_OperationMode(MODE_TURBO);     // Turbo mode (lower noise at same rate, more power)
    pc_ads1220.set_conv_mode_continuous();        // Continuous conversions
    pc_ads1220.CurrentSources_OFF();              // No IDAC needed for voltage excitation
    pc_ads1220.LowSideSwitch_OPEN();              // Default

    pc_ads1220.select_mux_channels(MUX_AIN0_AIN1);  // Start with Channel 1
    pc_ads1220.Start_Conv();                        // Begin conversions

    enableInterruptPin();
    time_init = micros();
}

void loop()
{
    if(drdyIntrFlag){
        drdyIntrFlag = false; 

        if (currentChannel == 0) {
            adcValue01 = pc_ads1220.Read_Data_Samples();
            pc_ads1220.select_mux_channels(MUX_AIN2_AIN3); // Switch to second channel (restarts conversion)
            currentChannel = 1;  
        } else {
            adcValue23 = pc_ads1220.Read_Data_Samples();
            pc_ads1220.select_mux_channels(MUX_AIN0_AIN1); // Switch back to first channel (restarts conversion)
            currentChannel = 0;


        Serial.print(micros() - time_init);
        Serial.print(",");
        Serial.print(adcValue01);
        Serial.print(",");
        Serial.print(adcValue23);
        Serial.print(",");
        Serial.print(adcValue01);
        Serial.print(",");
        Serial.println(adcValue23);   
        }
    }
}