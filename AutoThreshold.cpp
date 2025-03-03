#include "PGA460.hpp"

/*---------------------------- autoThreshold --------------------------
 |  Function autoThreshold
 |
 |  Purpose:  Automatically assigns threshold time and level values
 |  			based on a no-object burst/listen command
 |
 |  Parameters:
 |		noiseMargin (IN) -- margin between maximum downsampled noise
 |						value and the threshold level in intervals
 |						of 8.
 |		windowIndex (IN) -- spacing between each threshold time as an
 |						index (refer to datasheet for microsecond
 |						equivalent). To use the existing threshold
 |						times, enter a value of '16'.
 |		autoMax (IN) -- automatically set threshold levels up to this
 |					threshold point (maximum is 12). Remaining levels
 |					will not change.
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
bool PGA460::autoThreshold(uint8_t windowIndex, uint8_t autoMax, uint8_t noiseMargin) {
    // local variables
    uint8_t thresholdTime[6];     // threshold time values for selected preset
    uint8_t thresholdLevel[10];   // threshold level values for selected preset
    uint8_t thresholdMax[12];     // maximum echo data dump values per partition
    uint8_t thresholdOffset  = 0; // -6 to +7 where MSB is sign value
    bool thresholdOffsetFlag = 0; // when high, the level offset value is updated

    // set thresholdTime and thresholdLevel to existing threshold time and level values respectively
    thresholdTime[0]  = P1_THR_0.data;
    thresholdTime[1]  = P1_THR_1.data;
    thresholdTime[2]  = P1_THR_2.data;
    thresholdTime[3]  = P1_THR_3.data;
    thresholdTime[4]  = P1_THR_4.data;
    thresholdTime[5]  = P1_THR_5.data;
    thresholdLevel[0] = P1_THR_6.data;
    thresholdLevel[1] = P1_THR_7.data;
    thresholdLevel[2] = P1_THR_8.data;
    thresholdLevel[3] = P1_THR_9.data;
    thresholdLevel[4] = P1_THR_10.data;
    thresholdLevel[5] = P1_THR_11.data;
    thresholdLevel[6] = P1_THR_12.data;
    thresholdLevel[7] = P1_THR_13.data;
    thresholdLevel[8] = P1_THR_14.data;
    thresholdLevel[9] = P1_THR_15.data;

    // replace each preset time with windowIndex for the number of points to auto-calc
    if (windowIndex >= 16) { // skip threshold-time configuration
    } else {
        for (uint8_t i = 0; i < 12; i += 2) {
            if (autoMax > i) {
                thresholdTime[i / 2] = thresholdTime[i / 2] & 0x0F;
                thresholdTime[i / 2] = (windowIndex << 4) | thresholdTime[i / 2];
                if (autoMax > i + 1) {
                    thresholdTime[i / 2] = thresholdTime[i / 2] & 0xF0;
                    thresholdTime[i / 2] = (windowIndex & 0x0F) | thresholdTime[i / 2];
                }
            }
        }
    }

    // run preset1 burst-and-listen to collect EDD data
    EE_CNTRL.data = 0x80;
    registerWrite(EE_CNTRL);

    preset1BL(1);
    readEchoDataDump();

    EE_CNTRL.data = 0x00;
    registerWrite(EE_CNTRL);

    // determine the number of threshold points that are within the record length time
    unsigned int recTime         = 2 * 4096; // convert record length value to time equivalent in microseconds
    uint8_t numPoints            = 0;
    uint8_t timeRegister         = 0;
    unsigned int thresholdMicros = 0; // threshold total time in microseconds
    unsigned int eddMarker[12];       // echo data dump time marker between each threhsold point
    for (timeRegister = 0; timeRegister < 6; timeRegister++) {
        // check threshold 1 of 2 in single register
        switch ((thresholdTime[timeRegister] >> 4) & 0x0F) {
            case 0:
                thresholdMicros += 100;
                break;
            case 1:
                thresholdMicros += 200;
                break;
            case 2:
                thresholdMicros += 300;
                break;
            case 3:
                thresholdMicros += 400;
                break;
            case 4:
                thresholdMicros += 600;
                break;
            case 5:
                thresholdMicros += 800;
                break;
            case 6:
                thresholdMicros += 1000;
                break;
            case 7:
                thresholdMicros += 1200;
                break;
            case 8:
                thresholdMicros += 1400;
                break;
            case 9:
                thresholdMicros += 2000;
                break;
            case 10:
                thresholdMicros += 2400;
                break;
            case 11:
                thresholdMicros += 3200;
                break;
            case 12:
                thresholdMicros += 4000;
                break;
            case 13:
                thresholdMicros += 5200;
                break;
            case 14:
                thresholdMicros += 6400;
                break;
            case 15:
                thresholdMicros += 8000;
                break;
            default:
                break;
        }
        eddMarker[timeRegister * 2] = thresholdMicros;
        if (thresholdMicros >= recTime) {
            numPoints    = timeRegister * 2;
            timeRegister = 6; // forces exit
        } else {
            // check threshold 2 of 2 in single register
            switch (thresholdTime[timeRegister] & 0x0F) {
                case 0:
                    thresholdMicros += 100;
                    break;
                case 1:
                    thresholdMicros += 200;
                    break;
                case 2:
                    thresholdMicros += 300;
                    break;
                case 3:
                    thresholdMicros += 400;
                    break;
                case 4:
                    thresholdMicros += 600;
                    break;
                case 5:
                    thresholdMicros += 800;
                    break;
                case 6:
                    thresholdMicros += 1000;
                    break;
                case 7:
                    thresholdMicros += 1200;
                    break;
                case 8:
                    thresholdMicros += 1400;
                    break;
                case 9:
                    thresholdMicros += 2000;
                    break;
                case 10:
                    thresholdMicros += 2400;
                    break;
                case 11:
                    thresholdMicros += 3200;
                    break;
                case 12:
                    thresholdMicros += 4000;
                    break;
                case 13:
                    thresholdMicros += 5200;
                    break;
                case 14:
                    thresholdMicros += 6400;
                    break;
                case 15:
                    thresholdMicros += 8000;
                    break;
                default:
                    break;
            }
            eddMarker[timeRegister * 2 + 1] = thresholdMicros;
            if (thresholdMicros >= recTime) {
                numPoints    = (timeRegister * 2) + 1;
                timeRegister = 6; // forces exit
            }
        }
    }
    if (numPoints == 0) { // if all points fall within the record length
        numPoints = 11;
    }

    // convert up to 12 echo data dump markers from microseconds to index
    uint8_t eddIndex[13];
    eddIndex[0] = 0;
    for (uint8_t i = 0; i < 12; i++) {
        eddIndex[i + 1] = ((eddMarker[i] / 100) * 128) / (recTime / 100); // divide by 100 for best accuracy in MSP430
    }

    // downsample the echo data dump based on the number of partitions
    memset(thresholdMax, 0x00, 12); // zero thresholdMax array
    uint8_t eddLevel = 0;
    for (uint8_t i = 0; i < numPoints + 1; i++) {
        eddLevel = 0;
        for (uint8_t j = eddIndex[i]; j < eddIndex[i + 1]; j++) {
            eddLevel = echoDataDump[j];
            if (thresholdMax[i] < eddLevel) {
                thresholdMax[i] = eddLevel;
            }
        }
    }
    // set threhsold points which exceed the record length to same value as last valid value
    if (numPoints < autoMax) {
        for (uint8_t i = numPoints; i < autoMax; i++) {
            if (numPoints == 0) {
                thresholdMax[i] = 128;
            } else {
                thresholdMax[i] = thresholdMax[numPoints - 1];
            }
        }
    }

    // filter y-max for level compatibility of first eight points
    for (uint8_t i = 0; i < 8; i++) { // first eight levels must be mutliples of eight
        while ((thresholdMax[i] % 8 != 0) && (thresholdMax[i] < 248)) {
            thresholdMax[i] += 1;
        }
    }

    // apply noise floor offset
    for (uint8_t i = 0; i < 12; i++) {
        if (thresholdMax[i] + noiseMargin >= 248 && thresholdMax[i] + noiseMargin < 255) {
            thresholdMax[i]     = 248;
            thresholdOffset     = 0b0110; //+6
            thresholdOffsetFlag = true;
        } else if (thresholdMax[i] + noiseMargin >= 255) {
            thresholdMax[i]     = 248;
            thresholdOffset     = 0b0111; // +7
            thresholdOffsetFlag = true;
        } else {
            thresholdMax[i] += noiseMargin;
        }
    }

    // convert first eight auto calibrated levels to five-bit equivalents
    uint8_t rounding = 0;
    if (autoMax >= 8) {
        rounding = 8;
    } else {
        rounding = autoMax;
    }
    for (uint8_t i = 0; i < rounding; i++) {
        thresholdMax[i] = thresholdMax[i] / 8;
    }

    // concatenate and merge threshold level register values
    if (autoMax > 0) { // Px_THR_6 L1,L2
        thresholdLevel[0] = (thresholdLevel[0] & ~0xF8) | (thresholdMax[0] << 3);
    }
    if (autoMax > 1) { // Px_THR_6 L1,L2
        thresholdLevel[0] = (thresholdLevel[0] & ~0x07) | (thresholdMax[1] >> 2);
    }

    if (autoMax > 1) { // Px_THR_7 L2,L3,L4
        thresholdLevel[1] = (thresholdLevel[1] & ~0xC0) | (thresholdMax[1] << 6);
    }
    if (autoMax > 2) { // Px_THR_7 L2,L3,L4
        thresholdLevel[1] = (thresholdLevel[1] & ~0x3E) | (thresholdMax[2] << 1);
    }
    if (autoMax > 3) { // Px_THR_7 L2,L3,L4
        thresholdLevel[1] = (thresholdLevel[1] & ~0x01) | (thresholdMax[3] >> 4);
    }

    if (autoMax > 3) { // Px_THR_8 L4,L5
        thresholdLevel[2] = (thresholdLevel[2] & ~0xF0) | (thresholdMax[3] << 4);
    }
    if (autoMax > 4) { // Px_THR_8 L4,L5
        thresholdLevel[2] = (thresholdLevel[2] & ~0x0F) | (thresholdMax[4] >> 1);
    }

    if (autoMax > 4) { // Px_THR_9 L5,L6,L7
        thresholdLevel[3] = (thresholdLevel[3] & ~0x80) | (thresholdMax[4] << 7);
    }
    if (autoMax > 5) { // Px_THR_9 L5,L6,L7
        thresholdLevel[3] = (thresholdLevel[3] & ~0x7C) | (thresholdMax[5] << 2);
    }
    if (autoMax > 6) { // Px_THR_9 L5,L6,L7
        thresholdLevel[3] = (thresholdLevel[3] & ~0x03) | (thresholdMax[6] >> 3);
    }

    if (autoMax > 6) { // Px_THR_10 L7,L8
        thresholdLevel[4] = (thresholdLevel[4] & ~0xE0) | (thresholdMax[6] << 5);
    }
    if (autoMax > 7) { // Px_THR_10 L7,L8
        thresholdLevel[4] = (thresholdLevel[4] & ~0x1F) | (thresholdMax[7]);
    }

    if (autoMax > 8) { // Px_THR_11 L9
        thresholdLevel[5] = thresholdMax[8];
    }
    if (autoMax > 9) { // Px_THR_12 L10
        thresholdLevel[6] = thresholdMax[9];
    }
    if (autoMax > 10) { // Px_THR_13 L11
        thresholdLevel[7] = thresholdMax[10];
    }
    if (autoMax > 11) { // Px_THR_14 L12
        thresholdLevel[8] = thresholdMax[11];
    }
    if (thresholdOffsetFlag == true) { // Px_THR_15 LOff
        thresholdLevel[9] = thresholdOffset & 0x0F;
    }

    P1_THR_0.data  = thresholdTime[0];
    P1_THR_1.data  = thresholdTime[1];
    P1_THR_2.data  = thresholdTime[2];
    P1_THR_3.data  = thresholdTime[3];
    P1_THR_4.data  = thresholdTime[4];
    P1_THR_5.data  = thresholdTime[5];
    P1_THR_6.data  = thresholdLevel[0];
    P1_THR_7.data  = thresholdLevel[1];
    P1_THR_8.data  = thresholdLevel[2];
    P1_THR_9.data  = thresholdLevel[3];
    P1_THR_10.data = thresholdLevel[4];
    P1_THR_11.data = thresholdLevel[5];
    P1_THR_12.data = thresholdLevel[6];
    P1_THR_13.data = thresholdLevel[7];
    P1_THR_14.data = thresholdLevel[8];
    P1_THR_15.data = thresholdLevel[9];

    return thresholdBulkWrite();
}