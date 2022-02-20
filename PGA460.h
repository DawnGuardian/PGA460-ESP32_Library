#ifndef PGA_460_H_
#define PGA_460_H_

#include "Arduino.h"

// Pin mapping
/*
  Communication to the PGA460 chip is via UART,
  specifically using UART2 of ESP32
  ESP_RX2 16
  ESP_TX2 17
*/
/*
  The following pins are required when used in conjuction with
  the BOOSTXL-PGA460 daughter card for the MSP-EXP430F5529LP launch pad
*/
#define COM_SEL 32
#define COM_PD 33
#define MEM_HOLD 25
#define MEM_CS 26

struct PGA460Register {
    uint8_t data;
    const uint8_t addr;
};

class PGA460 {
  public:
    // PGA460 registers by name with default settings
    struct PGA460Register USER_DATA1    = {0x00, 0x00};
    struct PGA460Register USER_DATA2    = {0x00, 0x01};
    struct PGA460Register USER_DATA3    = {0x00, 0x02};
    struct PGA460Register USER_DATA4    = {0x00, 0x03};
    struct PGA460Register USER_DATA5    = {0x00, 0x04};
    struct PGA460Register USER_DATA6    = {0x00, 0x05};
    struct PGA460Register USER_DATA7    = {0x00, 0x06};
    struct PGA460Register USER_DATA8    = {0x00, 0x07};
    struct PGA460Register USER_DATA9    = {0x00, 0x08};
    struct PGA460Register USER_DATA10   = {0x00, 0x09};
    struct PGA460Register USER_DATA11   = {0x00, 0x0A};
    struct PGA460Register USER_DATA12   = {0x00, 0x0B};
    struct PGA460Register USER_DATA13   = {0x00, 0x0C};
    struct PGA460Register USER_DATA14   = {0x00, 0x0D};
    struct PGA460Register USER_DATA15   = {0x00, 0x0E};
    struct PGA460Register USER_DATA16   = {0x00, 0x0F};
    struct PGA460Register USER_DATA17   = {0x00, 0x10};
    struct PGA460Register USER_DATA18   = {0x00, 0x11};
    struct PGA460Register USER_DATA19   = {0x00, 0x12};
    struct PGA460Register USER_DATA20   = {0x00, 0x13};
    struct PGA460Register TVGAIN0       = {0xAF, 0x14};
    struct PGA460Register TVGAIN1       = {0xFF, 0x15};
    struct PGA460Register TVGAIN2       = {0xFF, 0x16};
    struct PGA460Register TVGAIN3       = {0x2D, 0x17};
    struct PGA460Register TVGAIN4       = {0x68, 0x18};
    struct PGA460Register TVGAIN5       = {0x36, 0x19};
    struct PGA460Register TVGAIN6       = {0xFC, 0x1A};
    struct PGA460Register INIT_GAIN     = {0xC0, 0x1B};
    struct PGA460Register FREQUENCY     = {0x8C, 0x1C};
    struct PGA460Register DEADTIME      = {0x00, 0x1D};
    struct PGA460Register PULSE_P1      = {0x01, 0x1E};
    struct PGA460Register PULSE_P2      = {0x12, 0x1F};
    struct PGA460Register CURR_LIM_P1   = {0x47, 0x20};
    struct PGA460Register CURR_LIM_P2   = {0xFF, 0x21};
    struct PGA460Register REC_LENGTH    = {0x1C, 0x22};
    struct PGA460Register FREQ_DIAG     = {0x00, 0x23};
    struct PGA460Register SAT_FDIAG_TH  = {0xEE, 0x24};
    struct PGA460Register FVOLT_DEC     = {0x7C, 0x25};
    struct PGA460Register DECPL_TEMP    = {0x0A, 0x26};
    struct PGA460Register DSP_SCALE     = {0x00, 0x27};
    struct PGA460Register TEMP_TRIM     = {0x00, 0x28};
    struct PGA460Register P1_GAIN_CTRL  = {0x00, 0x29};
    struct PGA460Register P2_GAIN_CTRL  = {0x00, 0x2A};
    struct PGA460Register EE_CRC        = {0xFF, 0x2B};
    struct PGA460Register EE_CNTRL      = {0x00, 0x40};
    struct PGA460Register BPF_A2_MSB    = {0x00, 0x41};
    struct PGA460Register BPF_A2_LSB    = {0x00, 0x42};
    struct PGA460Register BPF_A3_MSB    = {0x00, 0x43};
    struct PGA460Register BPF_A3_LSB    = {0x00, 0x44};
    struct PGA460Register BPF_B1_MSB    = {0x00, 0x45};
    struct PGA460Register BPF_B1_LSB    = {0x00, 0x46};
    struct PGA460Register LPF_A2_MSB    = {0x00, 0x47};
    struct PGA460Register LPF_A2_LSB    = {0x00, 0x48};
    struct PGA460Register LPF_B1_MSB    = {0x00, 0x49};
    struct PGA460Register LPF_B1_LSB    = {0x00, 0x4A};
    struct PGA460Register TEST_MUX      = {0x00, 0x4B};
    struct PGA460Register DEV_STAT0     = {0x00, 0x4C};
    struct PGA460Register DEV_STAT1     = {0x00, 0x4D};
    struct PGA460Register P1_THR_0      = {0x00, 0x5F};
    struct PGA460Register P1_THR_1      = {0x00, 0x60};
    struct PGA460Register P1_THR_2      = {0x00, 0x61};
    struct PGA460Register P1_THR_3      = {0x00, 0x62};
    struct PGA460Register P1_THR_4      = {0x00, 0x63};
    struct PGA460Register P1_THR_5      = {0x00, 0x64};
    struct PGA460Register P1_THR_6      = {0x00, 0x65};
    struct PGA460Register P1_THR_7      = {0x00, 0x66};
    struct PGA460Register P1_THR_8      = {0x00, 0x67};
    struct PGA460Register P1_THR_9      = {0x00, 0x68};
    struct PGA460Register P1_THR_10     = {0x00, 0x69};
    struct PGA460Register P1_THR_11     = {0x00, 0x6A};
    struct PGA460Register P1_THR_12     = {0x00, 0x6B};
    struct PGA460Register P1_THR_13     = {0x00, 0x6C};
    struct PGA460Register P1_THR_14     = {0x00, 0x6D};
    struct PGA460Register P1_THR_15     = {0x00, 0x6E};
    struct PGA460Register P2_THR_0      = {0x00, 0x6F};
    struct PGA460Register P2_THR_1      = {0x00, 0x70};
    struct PGA460Register P2_THR_2      = {0x00, 0x71};
    struct PGA460Register P2_THR_3      = {0x00, 0x72};
    struct PGA460Register P2_THR_4      = {0x00, 0x73};
    struct PGA460Register P2_THR_5      = {0x00, 0x74};
    struct PGA460Register P2_THR_6      = {0x00, 0x75};
    struct PGA460Register P2_THR_7      = {0x00, 0x76};
    struct PGA460Register P2_THR_8      = {0x00, 0x77};
    struct PGA460Register P2_THR_9      = {0x00, 0x78};
    struct PGA460Register P2_THR_10     = {0x00, 0x79};
    struct PGA460Register P2_THR_11     = {0x00, 0x7A};
    struct PGA460Register P2_THR_12     = {0x00, 0x7B};
    struct PGA460Register P2_THR_13     = {0x00, 0x7C};
    struct PGA460Register P2_THR_14     = {0x00, 0x7D};
    struct PGA460Register P2_THR_15     = {0x00, 0x7E};
    const struct PGA460Register THR_CRC = {0x00, 0x7F};

    // Buffer variables
    const uint8_t SYNC_FIELD = 0x55;
    uint8_t diagnosticField  = 0x00;
    uint8_t lastMeasurmentResult[32];
    uint8_t temperature;
    uint8_t noise;
    uint8_t echoDataDump[128];
    uint8_t diagnosticResult[2];

    // Constructor
    PGA460();

    // Trigger commands
    void preset1BL(uint8_t numObj);
    void preset2BL(uint8_t numObj);
    void preset1OL(uint8_t numObj);
    void preset2OL(uint8_t numObj);
    void temperatureOrNoise(uint8_t option);

    // Read commands
    void readMeasurementResult(uint8_t numObj);
    void readTemperatureAndNoise();
    void readEchoDataDump();
    void systemDiagnostic();

    // Single register
    void registerRead(PGA460Register reg);
    void registerWrite(PGA460Register reg);

    // EEPROM R/W
    void eepromBulkRead();
    void eepromBulkWrite();
    void eepromBulkWrite(uint8_t data[]);

    // Time Varying Gain R/W
    void tvgBulkRead();
    void tvgBulkWrite();
    void tvgBulkWrite(uint8_t data[]);

    // Threshold values R/W
    void thresholdBulkRead();
    void thresholdBulkWrite();
    void thresholdBulkWrite(uint8_t data[]);

    // PGA460 setup
    void setTransducerSettings(uint8_t settings[]);
    void setTransducerSettings();
    void setThreshold(uint8_t preset, uint8_t settings[]);
    void setThreshold(uint8_t preset, uint8_t option);
    void setTVG(uint8_t settings[]);
    void setTVG(uint8_t option);

    // PGA460 setup for different transducers
    // WARNING: THESE FUNCTIONS WILL AFFECT TVG SETTINGS
    void setTransducerSettings_MA58MF14_7N();
    void setTransducerSettings_MA40H1S_R();
    void setTransducerSettings_MA40S4();
    void setTransducerSettings_UTR_1440K_TT_R();

    // Setup UART communication with external logger and PGA460
    void initialiseUART(uint32_t baudRate);

    // Auto-threshold function
    void autoThreshold(uint8_t windowIndex, uint8_t autoMax, uint8_t noiseMargin);

  private:
    uint8_t calcChecksum(uint8_t command[], uint8_t commandLength);
    void pga460SerialFlush();
};

#endif
