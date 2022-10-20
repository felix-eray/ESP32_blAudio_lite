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
    if( volumeLevel > 24 || volumeLevel < -70){
        return 0;
    }

    uint8_t volume = 48 - (volumeLevel * 2);

    TFA->volumeControl = (TFA->bassTreble & ~(0xFF)) | (volume);

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