#include "PGA460.hpp"

/*---------------------------------------- setTransducerSettings_UTR_1440K_TT_R ----------------------
|  Function:    setTransducerSettings_UTR_1440K_TT_R
|
|  Purpose:     Sets variables that hold the transducer settings with PUI UTR-1440K-TT-R settings
|
|  Parameters:  none
|
|  Returns:     Returns bool representing success of write to transducer settings registers
*--------------------------------------------------------------------------------------------------*/
bool PGA460::setTransducerSettings_UTR_1440K_TT_R() {
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
    TVGAIN0.data      = 0x9D;
    TVGAIN1.data      = 0xEE;
    TVGAIN2.data      = 0xEF;
    TVGAIN3.data      = 0x2D;
    TVGAIN4.data      = 0xB9;
    TVGAIN5.data      = 0xEF;
    TVGAIN6.data      = 0xDC;
    INIT_GAIN.data    = 0x03;
    FREQUENCY.data    = 0x32;
    DEADTIME.data     = 0x80;
    PULSE_P1.data     = 0x08;
    PULSE_P2.data     = 0x12;
    CURR_LIM_P1.data  = 0x72;
    CURR_LIM_P2.data  = 0x32;
    REC_LENGTH.data   = 0x19;
    FREQ_DIAG.data    = 0x33;
    SAT_FDIAG_TH.data = 0xEE;
    FVOLT_DEC.data    = 0x7C;
    DECPL_TEMP.data   = 0x8F;
    DSP_SCALE.data    = 0x00;
    TEMP_TRIM.data    = 0x00;
    P1_GAIN_CTRL.data = 0x09;
    P2_GAIN_CTRL.data = 0x29;

    return eepromBulkWrite();
}