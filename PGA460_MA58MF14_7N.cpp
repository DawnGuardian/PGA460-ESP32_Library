#include "PGA460.hpp"

/*---------------------------------------- setTransducerSettings_MA58MF14_7N ----------------------
|  Function:    setTransducerSettings_MA58MF14_7N
|
|  Purpose:     Sets variables that hold the transducer settings with Murata MA5814-7N settings
|
|  Parameters:  none
|
|  Returns:     Returns bool representing success of write to transducer settings registers
*--------------------------------------------------------------------------------------------------*/
bool PGA460::setTransducerSettings_MA58MF14_7N() {
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
    TVGAIN0.data      = 0xAA;
    TVGAIN1.data      = 0xAA;
    TVGAIN2.data      = 0xAA;
    TVGAIN3.data      = 0x82;
    TVGAIN4.data      = 0x08;
    TVGAIN5.data      = 0x20;
    TVGAIN6.data      = 0x80;
    INIT_GAIN.data    = 0x60;
    FREQUENCY.data    = 0x8F;
    DEADTIME.data     = 0xA0;
    PULSE_P1.data     = 0x04;
    PULSE_P2.data     = 0x10;
    CURR_LIM_P1.data  = 0x55;
    CURR_LIM_P2.data  = 0x55;
    REC_LENGTH.data   = 0x19;
    FREQ_DIAG.data    = 0x33;
    SAT_FDIAG_TH.data = 0xEE;
    FVOLT_DEC.data    = 0x7C;
    DECPL_TEMP.data   = 0x4F;
    DSP_SCALE.data    = 0x00;
    TEMP_TRIM.data    = 0x00;
    P1_GAIN_CTRL.data = 0x09;
    P2_GAIN_CTRL.data = 0x09;

    return eepromBulkWrite();
}