#include <Arduino.h>
#include "Wire.h"
#include "TFA9879.h"

void TFA_init(TFA9879 *TFA){
    TFA->deviceControl = 0x0000;
    TFA->serialInterface1 = 0x0A18;
    TFA->serialInterface2 = 0x0A18;
    TFA->pcmIomInput1 = 0x0007;
    TFA->pcmIomInput2 = 0x0007;

    TFA->eqWord1[0] = 0x59DD;
    TFA->eqWord2[0] = 0xC63E;
    TFA->eqWord1[1] = 0x651A;
    TFA->eqWord2[1] = 0xE53E;
    TFA->eqWord1[2] = 0x4616;
    TFA->eqWord2[2] = 0xD33E;
    TFA->eqWord1[3] = 0x4DF3;
    TFA->eqWord2[3] = 0xEA3E;
    TFA->eqWord1[4] = 0x5EE0;
    TFA->eqWord2[4] = 0xF93E;

    TFA->bypassControl = 0x0093;
    TFA->DRC = 0x92BA;
    TFA->bassTreble = 0x12A5;
    TFA->HPfilter = 0x0004;
    TFA->volumeControl = 0x10BD;
    TFA->otherControls = 0x0000;
}

//set I2C-address for the TFA instance
void TFA_setAddress(TFA9879 *TFA, uint8_t address){
    TFA->I2Caddress = address;
}

//sets volume in 1dB steps, +24dB -> -70dB, anything lower thant that -> mute
//so volume level 0 = 0dB, 24 = 24dB, -50 = -50dB
uint8_t TFA_setVolume(TFA9879 *TFA, int volumeLevel)
{
    //check that values are in range
    if(volumeLevel > 24){
        volumeLevel = 24;
    }
    else if(volumeLevel < -71){
        volumeLevel = -71;
    }

    uint8_t volume = 48 - (volumeLevel * 2);

    TFA->volumeControl = (TFA->volumeControl & ~(0xFF)) | (volume);

    Wire.beginTransmission(TFA->I2Caddress);
    Wire.write(0x13); //volume control 0x13
    Wire.write(TFA->volumeControl >> 8);
    Wire.write(TFA->volumeControl);
    Wire.endTransmission(true);

    return 1;
}

uint8_t TFA_setBassTreble(TFA9879 *TFA, int bassLevel, int trebleLevel)
{
    // check that values are in range
    if ((bassLevel > 9 || bassLevel < -9) || (trebleLevel > 9 || trebleLevel < -9))
    {
        return 0;
    }

    uint8_t treble = 9 + trebleLevel;
    uint8_t bass = 9 + bassLevel;

    TFA->bassTreble = (TFA->bassTreble & ~(0b11111 << 9)) | (treble << 9);
    TFA->bassTreble = (TFA->bassTreble & ~(0b11111 << 2)) | (bass << 2);

    Wire.beginTransmission(TFA->I2Caddress);
    Wire.write(0x11); // bass and treble control 0x11
    Wire.write(TFA->bassTreble >> 8);
    Wire.write(TFA->bassTreble);
    Wire.endTransmission(true);

    return 1;
}

uint8_t TFA_setBassTrebleCfrequency(TFA9879 *TFA, int bassCorner, int trebleCorner)
{

    TFA->bassTreble = (TFA->bassTreble & ~(3 << 7)) | (trebleCorner << 7);
    TFA->bassTreble = (TFA->bassTreble & ~(3 << 0)) | (bassCorner << 0);

    Wire.beginTransmission(TFA->I2Caddress);
    Wire.write(0x11); // bass and treble control 0x11
    Wire.write(TFA->bassTreble >> 8);
    Wire.write(TFA->bassTreble);
    Wire.endTransmission(true);

    return 1;
}

uint8_t TFA_setDeviceControl(TFA9879 *TFA, int powerMode, int i2sInput)
{
    TFA->deviceControl = (TFA->deviceControl & ~(1 << 4)) | (i2sInput << 4);
    TFA->deviceControl = ((TFA->deviceControl & ~(1 << 0)) | (TFA->deviceControl & ~(1 << 3))) | (powerMode << 0) | (powerMode << 3);

    Wire.beginTransmission(TFA->I2Caddress);
    Wire.write(0x00); // device control 0x00
    Wire.write(TFA->deviceControl >> 8);
    Wire.write(TFA->deviceControl); // input 2, amp output on, TF9789 powered
    Wire.endTransmission(true);

    return 1;
}

uint8_t TFA_setLRchannel(TFA9879 *TFA, int LRconf, int i2sChannel)
{
    //configure correct I2S-input channel (1 or 2)
    if(i2sChannel == 0){
        TFA->serialInterface1 = (TFA->serialInterface1 & ~(3 << 10)) | (LRconf << 10);

        Wire.beginTransmission(TFA->I2Caddress);
        Wire.write(0x01); // serial interface control 1 0x01
        Wire.write(TFA->serialInterface1 >> 8);
        Wire.write(TFA->serialInterface1);
        Wire.endTransmission(true);

        return 1;
    }
    else{
        TFA->serialInterface2 = (TFA->serialInterface2 & ~(3 << 10)) | (LRconf << 10);

        Wire.beginTransmission(TFA->I2Caddress);
        Wire.write(0x03); // serial interface control 2 0x03
        Wire.write(TFA->serialInterface2 >> 8);
        Wire.write(TFA->serialInterface2);
        Wire.endTransmission(true);

        return 1;
    }

    return 1;
}