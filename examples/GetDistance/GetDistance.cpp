#include "Arduino.h"
#include "PGA460.h"

PGA460 pga460;

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.printf("UART connection established\nBeginning configuration ...\n");

    pga460.initialiseUART(115200);
    Serial.flush();
    Serial.printf("CHOOSE TRANSDUCER: (TYPE IN INDEX)\n");
    Serial.printf("1 -> Murata MA40H1S-R\t\t(BOOSTXL-PGA460 Daughter Card Direct Drive Transducer\n");
    Serial.printf("2 -> Murata MA58MF14-7N\t\t(BOOSTXL-PGA460 Daughter Card Transformer Drive Transducer\n");
    Serial.printf("3 -> PUI Audio UTR-1440K-TT-R\t(PGA460PSM-EVM Transducer\n");
    while (!Serial.available()) {
    }
    int option = Serial.readString().toInt();
    if (option == 1) {
        pga460.setTransducerSettings_MA40H1S_R();
        Serial.printf("Murata MA40H1S-R transducer settings applied\n");
    } else if (option == 2) {
        pga460.setTransducerSettings_MA58MF14_7N();
        Serial.printf("Murata MA58MF14-7N transducer settings applied\n");
    } else if (option == 3) {
        pga460.setTransducerSettings_UTR_1440K_TT_R();
        Serial.printf("PUI Audio UTR-1440K-TT-R transducer settings applied\n");
    } else {
        pga460.setTransducerSettings_MA40H1S_R();
        Serial.printf("Murata MA40H1S-R transducer settings applied\n");
    }

    pga460.REC_LENGTH.data = 0x30;
    pga460.registerWrite(pga460.REC_LENGTH);
    Serial.printf("Default transducer recording length applied\n");

    pga460.P1_THR_0.data  = 0x26;
    pga460.P1_THR_1.data  = 0x22;
    pga460.P1_THR_2.data  = 0x44;
    pga460.P1_THR_3.data  = 0x44;
    pga460.P1_THR_4.data  = 0x44;
    pga460.P1_THR_5.data  = 0x44;
    pga460.P1_THR_6.data  = 0xFF;
    pga460.P1_THR_7.data  = 0xEC;
    pga460.P1_THR_8.data  = 0x84;
    pga460.P1_THR_9.data  = 0x21;
    pga460.P1_THR_10.data = 0x09;
    pga460.P1_THR_11.data = 0x40;
    pga460.P1_THR_12.data = 0x40;
    pga460.P1_THR_13.data = 0x40;
    pga460.P1_THR_14.data = 0x40;
    pga460.P1_THR_15.data = 0x07;
    pga460.thresholdBulkWrite();
    Serial.printf("Default threshold values applied\n");
    Serial.printf("PGA460 configured\n");
    delay(1000);
}

void loop() {
    Serial.printf("--------------------------------------------------------------------------------------------------------------\n");

    pga460.preset1BL(1);
    pga460.readMeasurementResult(1);
    pga460.temperatureOrNoise(0);
    pga460.temperatureOrNoise(1);
    pga460.readTemperatureAndNoise();

    int timeOfFlight    = (pga460.lastMeasurmentResult[0] << 8) + pga460.lastMeasurmentResult[1]; // in μs
    uint8_t temperature = pga460.temperature;                                                     // needs to be converted

    double distance          = 343.0 * (timeOfFlight / 2.0) / 1000; // in mm
    double parsedTemperature = (temperature - 64.0) / 1.5;          // in °C

    Serial.printf("ToF\t\t: %d μs\n", timeOfFlight);
    Serial.printf("Object detected\t: %f mm\n", distance);
    Serial.printf("Object width\t: %d μs\n", (pga460.lastMeasurmentResult[2] * 8));
    Serial.printf("Object peak amp\t: %d μs\n", (pga460.lastMeasurmentResult[3]));
    Serial.printf("Temperature\t: %f °C\n", parsedTemperature);

    Serial.printf("--------------------------------------------------------------------------------------------------------------\n");
    delay(1000);
}