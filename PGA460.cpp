#include "PGA460.h"
#include "Arduino.h"

// Serial read timeout in milliseconds
#define MAX_MILLIS_TO_WAIT 1000

/*---------------------------------------- pga460 -----------------------------------------------
|  Function:    pga460
|
|  Purpose:     Constructor function
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
PGA460::PGA460() {}

/*---------------------------------------- calcChecksum -----------------------------------------
|  Function:    calcChecksum
|
|  Purpose:     Calculates checksum for the given command package
|
|  Parameters:
|       command[] (IN) -- The entire command package as a uint8_t array. Should include spaces for the
|               sync field, and the checksum field.
|       commandLenght (IN) -- Length of the command[] array passed in
|
|  Returns:     Return the checksum uint8_t value
*--------------------------------------------------------------------------------------------------*/
uint8_t PGA460::calcChecksum(uint8_t command[], uint8_t commandLength) {
    uint16_t sumOverflow = 0;
    uint8_t sum          = 0;
    uint8_t carry        = 0;

    sumOverflow = command[1] + command[2];
    carry       = sumOverflow >> 8;
    sum         = sumOverflow & 0xFF;

    for (uint8_t i = 3; i < (commandLength - 1); i++) {
        sumOverflow = command[i] + carry + sum;
        carry       = sumOverflow >> 8;
        sum         = sumOverflow & 0xFF;
    }

    sum = ~(carry + sumOverflow);

    return sum;
}

/*---------------------------------------- pga460SerialFlush ------------------------------------
|  Function:    pga460SerialFlush
|
|  Purpose:     Flushes UART port connected to PGA460
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::pga460SerialFlush() {
    delay(10);
    diagnosticField = 0x00;
    Serial2.flush();
    while ((Serial2.available() > 0)) {
        Serial2.read();
    }
    Serial2.flush();
    delay(10);
}

/*---------------------------------------- preset1BL --------------------------------------------
|  Function:    preset1BL
|
|  Purpose:     Sends the Burst and Listen command using preset 1
|
|  Parameters:
|       numObj (IN) -- Number of objects to detect (min 1, max 8)
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::preset1BL(uint8_t numObj) {
    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x00, numObj, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);
}

/*---------------------------------------- preset2BL --------------------------------------------
|  Function:    preset2BL
|
|  Purpose:     Sends the Burst and Listen command using preset 2
|
|  Parameters:
|       numObj (IN) -- Number of objects to detect (min 1, max 8)
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::preset2BL(uint8_t numObj) {
    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x01, numObj, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);
}

/*---------------------------------------- preset1OL --------------------------------------------
|  Function:    preset1OL
|
|  Purpose:     Sends the Listen Only command using preset 1
|
|  Parameters:
|       numObj (IN) -- Number of objects to detect (min 1, max 8)
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::preset1OL(uint8_t numObj) {
    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x02, numObj, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);
}

/*---------------------------------------- preset2OL --------------------------------------------
|  Function:    preset2OL
|
|  Purpose:     Sends the Listen Only command using preset 2
|
|  Parameters:
|       numObj (IN) -- Number of objects to detect (min 1, max 8)
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::preset2OL(uint8_t numObj) {
    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x03, numObj, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);
}

/*---------------------------------------- temperatureOrNoise -----------------------------------
|  Function:    temperatureOrNoise
|
|  Purpose:     Sends the Temperature or Noise command to the PGA460
|
|  Parameters:
|       option (IN) -- The option between reading temperature or noise
|               0 -> Temperature
|               1 -> Noise
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::temperatureOrNoise(uint8_t option) {
    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x04, option, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);
}

/*---------------------------------------- readMeasurementResult --------------------------------
|  Function:    readMeasurementResult
|
|  Purpose:     Pulls the ultrasonic measurement data from PGA460. Should be used only after any
|               of the Burst/Listen or Listen Only commands are sent.
|               Data is stored in lastMeasurementResult[] as a uint8_t array.
|
|  Parameters:
|       numObj (IN) -- The number of objects to detect (min 1, max 8)
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::readMeasurementResult(uint8_t numObj) {
    pga460SerialFlush();
    memset(lastMeasurmentResult, 0x00, 32);

    uint8_t buffer[3] = {SYNC_FIELD, 0x05, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 4 * numObj;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            lastMeasurmentResult[i - 1] = Serial2.read();
        }
    }
}

/*---------------------------------------- readTemperatureAndNoise ------------------------------
|  Function:    readTemperatureAndNoise
|
|  Purpose:     Pulls the temperature and noise data from PGA460. Should be used only after any
|               of the Burst/Listen or Listen Only commands are sent.
|               Data is stored in temperature and noise uint8_t objects.
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::readTemperatureAndNoise() {
    pga460SerialFlush();

    uint8_t buffer[3] = {SYNC_FIELD, 0x06, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 2;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else if (i == 1) {
            temperature = Serial2.read();
        } else if (i == 2) {
            noise = Serial2.read();
        }
    }
}

/*---------------------------------------- readEchoDataDump --------------------------------------
|  Function:    readEchoDataDump
|
|  Purpose:     Pulls the echo data dump from PGA460. Should be used only after any of the
|               Burst/Listen or Listen Only commands are sent. Data is stored in echoDataDump[]
|               as a uint8_t array.
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::readEchoDataDump() {
    pga460SerialFlush();
    memset(echoDataDump, 0x00, 128);

    uint8_t buffer[3] = {SYNC_FIELD, 0x07, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 128;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            echoDataDump[i - 1] = Serial2.read();
        }
    }
}

/*---------------------------------------- systemDiagnostic -------------------------------------
|  Function:    systemDiagnostic
|
|  Purpose:     Pulls PGA460  diagnostic information. Stored in diagnosticResult[].
|               diagnosticResult[0] -> Transducer frequency
|               diagnosticResult[1] -> Decay period time
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::systemDiagnostic() {
    pga460SerialFlush();
    memset(diagnosticResult, 0x00, 2);

    uint8_t buffer[3] = {SYNC_FIELD, 0x08, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 2;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            diagnosticResult[i - 1] = Serial2.read();
        }
    }
}

/*---------------------------------------- registerRead -----------------------------------------
|  Function:    registerRead
|
|  Purpose:     Reads the specified register of the PGA460.
|
|  Parameters:
|       addr (IN) -- Address of the register to be read from
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::registerRead(PGA460Register reg) {
    pga460SerialFlush();

    uint8_t buffer[3 + 1] = {SYNC_FIELD, 0x09, reg.addr, 0x00};
    buffer[3]             = calcChecksum(buffer, 4);
    Serial2.write(buffer, 4);
    delay(10);

    uint8_t dataLength      = 2 + 1;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            reg.data = Serial2.read();
        }
    }
}

/*---------------------------------------- registerWrite -----------------------------------------
|  Function:    registerWrite
|
|  Purpose:     Writes to the specified register of the PGA460.
|
|  Parameters:
|       addr (IN) -- Address of the register to be written to
|       data (IN) -- Data to be written to the register
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::registerWrite(PGA460Register reg) {
    uint8_t buffer[3 + 2] = {SYNC_FIELD, 0x0A, reg.addr, reg.data, 0x00};
    buffer[4]             = calcChecksum(buffer, 5);
    Serial2.write(buffer, 5);
    delay(10);
}

/*---------------------------------------- eepromBulkRead ----------------------------------------
|  Function:    eepromBulkRead
|
|  Purpose:     Reads EEPROM data from PGA460. Data is stored in individual variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::eepromBulkRead() {
    pga460SerialFlush();

    uint8_t buffer[3] = {SYNC_FIELD, 0x0B, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 43;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    uint8_t readData[43];
    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            readData[i - 1] = Serial2.read();
        }
    }
    USER_DATA1.data   = readData[0];
    USER_DATA2.data   = readData[1];
    USER_DATA3.data   = readData[2];
    USER_DATA4.data   = readData[3];
    USER_DATA5.data   = readData[4];
    USER_DATA6.data   = readData[5];
    USER_DATA7.data   = readData[6];
    USER_DATA8.data   = readData[7];
    USER_DATA9.data   = readData[8];
    USER_DATA10.data  = readData[9];
    USER_DATA11.data  = readData[10];
    USER_DATA12.data  = readData[11];
    USER_DATA13.data  = readData[12];
    USER_DATA14.data  = readData[13];
    USER_DATA15.data  = readData[14];
    USER_DATA16.data  = readData[15];
    USER_DATA17.data  = readData[16];
    USER_DATA18.data  = readData[17];
    USER_DATA19.data  = readData[18];
    USER_DATA20.data  = readData[19];
    TVGAIN0.data      = readData[20];
    TVGAIN1.data      = readData[21];
    TVGAIN2.data      = readData[22];
    TVGAIN3.data      = readData[23];
    TVGAIN4.data      = readData[24];
    TVGAIN5.data      = readData[25];
    TVGAIN6.data      = readData[26];
    INIT_GAIN.data    = readData[27];
    FREQUENCY.data    = readData[28];
    DEADTIME.data     = readData[29];
    PULSE_P1.data     = readData[30];
    PULSE_P2.data     = readData[31];
    CURR_LIM_P1.data  = readData[32];
    CURR_LIM_P2.data  = readData[33];
    REC_LENGTH.data   = readData[34];
    FREQ_DIAG.data    = readData[35];
    SAT_FDIAG_TH.data = readData[36];
    FVOLT_DEC.data    = readData[37];
    DECPL_TEMP.data   = readData[38];
    DSP_SCALE.data    = readData[39];
    TEMP_TRIM.data    = readData[40];
    P1_GAIN_CTRL.data = readData[41];
    P2_GAIN_CTRL.data = readData[42];
}

/*---------------------------------------- eepromBulkWrite----------------------------------------
|  Function:    eepromBulkWrite
|
|  Purpose:     Writes EEPROM data to PGA460 using current data in variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::eepromBulkWrite() {
    uint8_t buffer[3 + 43] = {SYNC_FIELD, 0x0C,
                              USER_DATA1.data, USER_DATA2.data, USER_DATA3.data, USER_DATA4.data, USER_DATA5.data,
                              USER_DATA6.data, USER_DATA7.data, USER_DATA8.data, USER_DATA9.data, USER_DATA10.data,
                              USER_DATA11.data, USER_DATA12.data, USER_DATA13.data, USER_DATA14.data, USER_DATA15.data,
                              USER_DATA16.data, USER_DATA17.data, USER_DATA18.data, USER_DATA19.data, USER_DATA20.data,
                              TVGAIN0.data, TVGAIN1.data, TVGAIN2.data, TVGAIN3.data, TVGAIN4.data, TVGAIN5.data, TVGAIN6.data, INIT_GAIN.data,
                              FREQUENCY.data, DEADTIME.data, PULSE_P1.data, PULSE_P2.data, CURR_LIM_P1.data, CURR_LIM_P2.data, REC_LENGTH.data,
                              FREQ_DIAG.data, SAT_FDIAG_TH.data, FVOLT_DEC.data, DECPL_TEMP.data, DSP_SCALE.data, TEMP_TRIM.data,
                              P1_GAIN_CTRL.data, P2_GAIN_CTRL.data, 0x00};
    buffer[45]             = calcChecksum(buffer, 46);
    Serial2.write(buffer, 46);
    delay(10);
}

/*---------------------------------------- eepromBulkWrite----------------------------------------
|  Function:    eepromBulkWrite
|
|  Purpose:     Writes EEPROM data to PGA460 after setting variables with supplied data.
|
|  Parameters:
|       data[] (IN) -- Array of 43 bytes with the threshold data
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::eepromBulkWrite(uint8_t data[]) {
    USER_DATA1.data   = data[0];
    USER_DATA2.data   = data[1];
    USER_DATA3.data   = data[2];
    USER_DATA4.data   = data[3];
    USER_DATA5.data   = data[4];
    USER_DATA6.data   = data[5];
    USER_DATA7.data   = data[6];
    USER_DATA8.data   = data[7];
    USER_DATA9.data   = data[8];
    USER_DATA10.data  = data[9];
    USER_DATA11.data  = data[10];
    USER_DATA12.data  = data[11];
    USER_DATA13.data  = data[12];
    USER_DATA14.data  = data[13];
    USER_DATA15.data  = data[14];
    USER_DATA16.data  = data[15];
    USER_DATA17.data  = data[16];
    USER_DATA18.data  = data[17];
    USER_DATA19.data  = data[18];
    USER_DATA20.data  = data[19];
    TVGAIN0.data      = data[20];
    TVGAIN1.data      = data[21];
    TVGAIN2.data      = data[22];
    TVGAIN3.data      = data[23];
    TVGAIN4.data      = data[24];
    TVGAIN5.data      = data[25];
    TVGAIN6.data      = data[26];
    INIT_GAIN.data    = data[27];
    FREQUENCY.data    = data[28];
    DEADTIME.data     = data[29];
    PULSE_P1.data     = data[30];
    PULSE_P2.data     = data[31];
    CURR_LIM_P1.data  = data[32];
    CURR_LIM_P2.data  = data[33];
    REC_LENGTH.data   = data[34];
    FREQ_DIAG.data    = data[35];
    SAT_FDIAG_TH.data = data[36];
    FVOLT_DEC.data    = data[37];
    DECPL_TEMP.data   = data[38];
    DSP_SCALE.data    = data[39];
    TEMP_TRIM.data    = data[40];
    P1_GAIN_CTRL.data = data[41];
    P2_GAIN_CTRL.data = data[42];

    eepromBulkWrite();
}

/*---------------------------------------- tvgBulkRead -------------------------------------------
|  Function:    tvgBulkRead
|
|  Purpose:     Reads Time Varying Gain data from PGA460. Data is stored in individual variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::tvgBulkRead() {
    pga460SerialFlush();

    uint8_t buffer[3] = {SYNC_FIELD, 0x0D, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 7;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    uint8_t readData[7];
    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            readData[i - 1] = Serial2.read();
        }
    }
    TVGAIN0.data = readData[0];
    TVGAIN1.data = readData[1];
    TVGAIN2.data = readData[2];
    TVGAIN3.data = readData[3];
    TVGAIN4.data = readData[4];
    TVGAIN5.data = readData[5];
    TVGAIN6.data = readData[6];
}

/*---------------------------------------- tvgBulkWrite ------------------------------------------
|  Function:    tvgBulkWrite
|
|  Purpose:     Writes Time Varying Gain data to PGA460 using current data in variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::tvgBulkWrite() {
    uint8_t buffer[3 + 7] = {SYNC_FIELD, 0x0E, TVGAIN0.data, TVGAIN1.data, TVGAIN2.data, TVGAIN3.data, TVGAIN4.data, TVGAIN5.data, TVGAIN6.data, 0x00};
    buffer[9]             = calcChecksum(buffer, 10);
    Serial2.write(buffer, 10);
    delay(10);
}

/*---------------------------------------- tvgBulkWrite ------------------------------------------
|  Function:    tvgBulkWrite
|
|  Purpose:     Writes Time Varying Gain data to PGA460 after setting variables with supplied data.
|
|  Parameters:
|       data[] (IN) -- Array of 7 bytes with the time varying gain data
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::tvgBulkWrite(uint8_t data[]) {
    TVGAIN0.data = data[0];
    TVGAIN1.data = data[1];
    TVGAIN2.data = data[2];
    TVGAIN3.data = data[3];
    TVGAIN4.data = data[4];
    TVGAIN5.data = data[5];
    TVGAIN6.data = data[6];

    tvgBulkWrite();
}

/*---------------------------------------- thresholdBulkRead --------------------------------------
|  Function:    thresholdBulkRead
|
|  Purpose:     Reads Threshold data from PGA460. Data is stored in individual variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::thresholdBulkRead() {
    pga460SerialFlush();

    uint8_t buffer[3] = {SYNC_FIELD, 0x0F, 0x00};
    buffer[2]         = calcChecksum(buffer, 3);
    Serial2.write(buffer, 3);
    delay(10);

    uint8_t dataLength      = 2 + 32;
    unsigned long startTime = millis();

    while ((Serial2.available() < dataLength) && ((millis() - startTime) < MAX_MILLIS_TO_WAIT)) {
    }

    if (Serial2.available() < dataLength) {
        return;
    }

    uint8_t readData[32];
    for (uint8_t i = 0; i < dataLength; i++) {
        if (i == 0) {
            diagnosticField = Serial2.read();
        } else if (i == (dataLength - 1)) {
            Serial2.read();
        } else {
            readData[i - 1] = Serial2.read();
        }
    }
    P1_THR_0.data  = readData[0];
    P1_THR_1.data  = readData[1];
    P1_THR_2.data  = readData[2];
    P1_THR_3.data  = readData[3];
    P1_THR_4.data  = readData[4];
    P1_THR_5.data  = readData[5];
    P1_THR_6.data  = readData[6];
    P1_THR_7.data  = readData[7];
    P1_THR_8.data  = readData[8];
    P1_THR_9.data  = readData[9];
    P1_THR_10.data = readData[10];
    P1_THR_11.data = readData[11];
    P1_THR_12.data = readData[12];
    P1_THR_13.data = readData[13];
    P1_THR_14.data = readData[14];
    P1_THR_15.data = readData[15];
    P2_THR_0.data  = readData[16];
    P2_THR_1.data  = readData[17];
    P2_THR_2.data  = readData[18];
    P2_THR_3.data  = readData[19];
    P2_THR_4.data  = readData[20];
    P2_THR_5.data  = readData[21];
    P2_THR_6.data  = readData[22];
    P2_THR_7.data  = readData[23];
    P2_THR_8.data  = readData[24];
    P2_THR_9.data  = readData[25];
    P2_THR_10.data = readData[26];
    P2_THR_11.data = readData[27];
    P2_THR_12.data = readData[28];
    P2_THR_13.data = readData[29];
    P2_THR_14.data = readData[30];
    P2_THR_15.data = readData[31];
}

/*---------------------------------------- thresholdBulkWrite --------------------------------------
|  Function:    thresholdBulkWrite
|
|  Purpose:     Writes Threshold data to PGA460 using current data in variables
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::thresholdBulkWrite() {
    uint8_t buffer[3 + 32] = {SYNC_FIELD, 0x10, P1_THR_0.data, P1_THR_1.data, P1_THR_2.data, P1_THR_3.data, P1_THR_4.data, P1_THR_5.data, P1_THR_6.data, P1_THR_7.data,
                              P1_THR_8.data, P1_THR_9.data, P1_THR_10.data, P1_THR_11.data, P1_THR_12.data, P1_THR_13.data, P1_THR_14.data, P1_THR_15.data,
                              P2_THR_0.data, P2_THR_1.data, P2_THR_2.data, P2_THR_3.data, P2_THR_4.data, P2_THR_5.data, P2_THR_6.data, P2_THR_7.data,
                              P2_THR_8.data, P2_THR_9.data, P2_THR_10.data, P2_THR_11.data, P2_THR_12.data, P2_THR_13.data, P2_THR_14.data, P2_THR_15.data, 0x00};
    buffer[34]             = calcChecksum(buffer, 35);
    Serial2.write(buffer, 35);
    delay(10);
}

/*---------------------------------------- thresholdBulkWrite -------------------------------------
|  Function:    thresholdBulkWrite
|
|  Purpose:     Writes Threshold data to PGA460 after setting variables with supplied data.
|
|  Parameters:
|       data[] (IN) -- Array of 32 bytes with the threshold data
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::thresholdBulkWrite(uint8_t data[]) {
    P1_THR_0.data  = data[0];
    P1_THR_1.data  = data[1];
    P1_THR_2.data  = data[2];
    P1_THR_3.data  = data[3];
    P1_THR_4.data  = data[4];
    P1_THR_5.data  = data[5];
    P1_THR_6.data  = data[6];
    P1_THR_7.data  = data[7];
    P1_THR_8.data  = data[8];
    P1_THR_9.data  = data[9];
    P1_THR_10.data = data[10];
    P1_THR_11.data = data[11];
    P1_THR_12.data = data[12];
    P1_THR_13.data = data[13];
    P1_THR_14.data = data[14];
    P1_THR_15.data = data[15];
    P2_THR_0.data  = data[16];
    P2_THR_1.data  = data[17];
    P2_THR_2.data  = data[18];
    P2_THR_3.data  = data[19];
    P2_THR_4.data  = data[20];
    P2_THR_5.data  = data[21];
    P2_THR_6.data  = data[22];
    P2_THR_7.data  = data[23];
    P2_THR_8.data  = data[24];
    P2_THR_9.data  = data[25];
    P2_THR_10.data = data[26];
    P2_THR_11.data = data[27];
    P2_THR_12.data = data[28];
    P2_THR_13.data = data[29];
    P2_THR_14.data = data[30];
    P2_THR_15.data = data[31];

    thresholdBulkWrite();
}

/*---------------------------------------- setTransducerSettings ------------------------------------
|  Function:    setTransducerSettings
|
|  Purpose:     Sets held variables that hold the transducer settings with supplied data
|
|  Parameters:
|       settings[] (IN) -- Array of 43 bytes with transducer settings
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setTransducerSettings(uint8_t settings[]) {
    USER_DATA1.data   = settings[0];
    USER_DATA2.data   = settings[1];
    USER_DATA3.data   = settings[2];
    USER_DATA4.data   = settings[3];
    USER_DATA5.data   = settings[4];
    USER_DATA6.data   = settings[5];
    USER_DATA7.data   = settings[6];
    USER_DATA8.data   = settings[7];
    USER_DATA9.data   = settings[8];
    USER_DATA10.data  = settings[9];
    USER_DATA11.data  = settings[10];
    USER_DATA12.data  = settings[11];
    USER_DATA13.data  = settings[12];
    USER_DATA14.data  = settings[13];
    USER_DATA15.data  = settings[14];
    USER_DATA16.data  = settings[15];
    USER_DATA17.data  = settings[16];
    USER_DATA18.data  = settings[17];
    USER_DATA19.data  = settings[18];
    USER_DATA20.data  = settings[19];
    TVGAIN0.data      = settings[20];
    TVGAIN1.data      = settings[21];
    TVGAIN2.data      = settings[22];
    TVGAIN3.data      = settings[23];
    TVGAIN4.data      = settings[24];
    TVGAIN5.data      = settings[25];
    TVGAIN6.data      = settings[26];
    INIT_GAIN.data    = settings[27];
    FREQUENCY.data    = settings[28];
    DEADTIME.data     = settings[29];
    PULSE_P1.data     = settings[30];
    PULSE_P2.data     = settings[31];
    CURR_LIM_P1.data  = settings[32];
    CURR_LIM_P2.data  = settings[33];
    REC_LENGTH.data   = settings[34];
    FREQ_DIAG.data    = settings[35];
    SAT_FDIAG_TH.data = settings[36];
    FVOLT_DEC.data    = settings[37];
    DECPL_TEMP.data   = settings[38];
    DSP_SCALE.data    = settings[39];
    TEMP_TRIM.data    = settings[40];
    P1_GAIN_CTRL.data = settings[41];
    P2_GAIN_CTRL.data = settings[42];

    eepromBulkWrite();
}

/*---------------------------------------- setTransducerSettings ------------------------------------
|  Function:    setTransducerSettings
|
|  Purpose:     Sets held variables that hold the transducer settings with default data
|
|  Parameters:  none
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setTransducerSettings() {
    USER_DATA1.data   = 0x00;
    USER_DATA2.data   = 0x00;
    USER_DATA3.data   = 0x00;
    USER_DATA4.data   = 0x00;
    USER_DATA5.data   = 0x00;
    USER_DATA6.data   = 0x00;
    USER_DATA7.data   = 0x00;
    USER_DATA8.data   = 0x00;
    USER_DATA9.data   = 0x00;
    USER_DATA10.data  = 0x00;
    USER_DATA11.data  = 0x00;
    USER_DATA12.data  = 0x00;
    USER_DATA13.data  = 0x00;
    USER_DATA14.data  = 0x00;
    USER_DATA15.data  = 0x00;
    USER_DATA16.data  = 0x00;
    USER_DATA17.data  = 0x00;
    USER_DATA18.data  = 0x00;
    USER_DATA19.data  = 0x00;
    USER_DATA20.data  = 0x00;
    TVGAIN0.data      = 0xAF;
    TVGAIN1.data      = 0xFF;
    TVGAIN2.data      = 0xFF;
    TVGAIN3.data      = 0x2D;
    TVGAIN4.data      = 0x68;
    TVGAIN5.data      = 0x36;
    TVGAIN6.data      = 0xFC;
    INIT_GAIN.data    = 0xC0;
    FREQUENCY.data    = 0x8C;
    DEADTIME.data     = 0x00;
    PULSE_P1.data     = 0x01;
    PULSE_P2.data     = 0x12;
    CURR_LIM_P1.data  = 0x47;
    CURR_LIM_P2.data  = 0xFF;
    REC_LENGTH.data   = 0x1C;
    FREQ_DIAG.data    = 0x00;
    SAT_FDIAG_TH.data = 0xEE;
    FVOLT_DEC.data    = 0x7C;
    DECPL_TEMP.data   = 0x0A;
    DSP_SCALE.data    = 0x00;
    TEMP_TRIM.data    = 0x00;
    P1_GAIN_CTRL.data = 0x00;
    P2_GAIN_CTRL.data = 0x00;

    eepromBulkWrite();
}

/*---------------------------------------- setThreshold --------------------------------------------
|  Function:    setThreshold
|
|  Purpose:     Sets held variables that hold the threshold settings with supplied data
|
|  Parameters:
|       preset -- Specifies which preset to be affected. (1 - Preset 1, 2 - Preset 2)
|       settings[] (IN) -- Array of 16 bytes with transducer settings
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setThreshold(uint8_t preset, uint8_t settings[]) {
    if (preset == 1) {
        P1_THR_0.data  = settings[0];
        P1_THR_1.data  = settings[1];
        P1_THR_2.data  = settings[2];
        P1_THR_3.data  = settings[3];
        P1_THR_4.data  = settings[4];
        P1_THR_5.data  = settings[5];
        P1_THR_6.data  = settings[6];
        P1_THR_7.data  = settings[7];
        P1_THR_8.data  = settings[8];
        P1_THR_9.data  = settings[9];
        P1_THR_10.data = settings[10];
        P1_THR_11.data = settings[11];
        P1_THR_12.data = settings[12];
        P1_THR_13.data = settings[13];
        P1_THR_14.data = settings[14];
        P1_THR_15.data = settings[15];
    } else if (preset == 2) {
        P2_THR_0.data  = settings[0];
        P2_THR_1.data  = settings[1];
        P2_THR_2.data  = settings[2];
        P2_THR_3.data  = settings[3];
        P2_THR_4.data  = settings[4];
        P2_THR_5.data  = settings[5];
        P2_THR_6.data  = settings[6];
        P2_THR_7.data  = settings[7];
        P2_THR_8.data  = settings[8];
        P2_THR_9.data  = settings[9];
        P2_THR_10.data = settings[10];
        P2_THR_11.data = settings[11];
        P2_THR_12.data = settings[12];
        P2_THR_13.data = settings[13];
        P2_THR_14.data = settings[14];
        P2_THR_15.data = settings[15];
    }

    thresholdBulkWrite();
}

/*---------------------------------------- setThreshold --------------------------------------------
|  Function:    setThreshold
|
|  Purpose:     Sets variables that hold the threshold settings with preconfigured settings
|
|  Parameters:
|       preset (IN) -- Option between Preset 1 and Preset 2
|       option (IN) -- Option between 4 predefined sets of data
|           0 -> Short
|           1 -> Long
|           2 -> All L1 and T1
|           3 -> All midcode
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setThreshold(uint8_t preset, uint8_t option) {
    switch (option) {
        case 0: {
            uint8_t thresholdBuffer[] = {0x44, 0x44, 0x44, 0x44, 0x55, 0x55, 0x9C, 0xD0, 0x72, 0x10, 0x63, 0x28, 0x30, 0x34, 0x3C, 0x00};
            setThreshold(preset, thresholdBuffer);
            break;
        }
        case 1: {
            uint8_t thresholdBuffer[] = {0x77, 0x77, 0x77, 0x77, 0x88, 0x88, 0x9C, 0xD0, 0x72, 0x10, 0x63, 0x28, 0x30, 0x34, 0x3C, 0x00};
            setThreshold(preset, thresholdBuffer);
            break;
        }
        case 2: {
            uint8_t thresholdBuffer[] = {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0x98, 0x98, 0x98, 0x98, 0x00};
            setThreshold(preset, thresholdBuffer);
            break;
        }
        case 3: {
            uint8_t thresholdBuffer[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00};
            setThreshold(preset, thresholdBuffer);
            break;
        }
        default:
            break;
    }

    thresholdBulkWrite();
}

/*---------------------------------------- setTVG ----------------------------------------------------
|  Function:    setTVG
|
|  Purpose:     Sets held variables that hold the time varying gain settings with supplied data
|
|  Parameters:
|       settings[] (IN) -- Array of 8 bytes with gain settings
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setTVG(uint8_t settings[]) {
    TVGAIN0.data   = settings[0];
    TVGAIN1.data   = settings[1];
    TVGAIN2.data   = settings[2];
    TVGAIN3.data   = settings[3];
    TVGAIN4.data   = settings[4];
    TVGAIN5.data   = settings[5];
    TVGAIN6.data   = settings[6];
    INIT_GAIN.data = settings[7];

    tvgBulkWrite();
}

/*---------------------------------------- setTVG -------------------------------------------------
|  Function:    setTVG
|
|  Purpose:     Sets variables that hold the TVG settings with preconfigured settings
|
|  Parameters:
|       option (IN) -- Option between 4 predefined sets of data
|           0 -> Short
|           1 -> Long
|           2 -> All L1 and T1
|           3 -> All midcode
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::setTVG(uint8_t option) {
    switch (option) {
        case 0: {
            uint8_t tvgBuffer[] = {0x44, 0x44, 0x44, 0x08, 0x64, 0x9A, 0x4C, 0x40};
            setTVG(tvgBuffer);
            break;
        }
        case 1: {
            uint8_t tvgBuffer[] = {0x8D, 0xEE, 0xEF, 0x10, 0xA5, 0x20, 0x60, 0x40};
            setTVG(tvgBuffer);
            break;
        }
        case 2: {
            uint8_t tvgBuffer[] = {0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x40};
            setTVG(tvgBuffer);
            break;
        }
        case 3: {
            uint8_t tvgBuffer[] = {0x88, 0x88, 0x88, 0x82, 0x08, 0x20, 0x40, 0x60};
            setTVG(tvgBuffer);
            break;
        }
        default:
            break;
    }

    tvgBulkWrite();
}

/*---------------------------------------- initialisePGA460 -------------------------------------------
|  Function:    initialisePGA460
|
|  Purpose:     Sets up UART communication with the PGA460 and writes inital data to it
|
|  Parameters:
|       parameter_name (IN, OUT, or IN/OUT) -- The baud rate for communication between the board and PGA460
|
|  Returns:     none
*--------------------------------------------------------------------------------------------------*/
void PGA460::initialiseUART(uint32_t baudRate) {
    pinMode(COM_SEL, OUTPUT);
    digitalWrite(COM_SEL, LOW);

    pinMode(COM_PD, OUTPUT);
    digitalWrite(COM_PD, LOW);

    pinMode(MEM_HOLD, OUTPUT);
    digitalWrite(MEM_HOLD, HIGH);

    pinMode(MEM_CS, OUTPUT);
    digitalWrite(MEM_CS, HIGH);

    Serial2.begin(baudRate, SERIAL_8N2);
}
