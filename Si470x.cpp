/**
 * Si470x
 *
 * Copyright (c) 2024 William Gao
 * 
 * This software is provided "as-is", without any warranty. See the LICENSE file 
 * for full license terms and conditions. Use of this software indicates agreement 
 * with those terms.
 */
#include "Si470x.h"
#include "Wire.h"

Si470x::Si470x(int pinRST, int pinSDIO, int pinSCLK, bool enableInterrupts) {
    _pinRST = pinRST;
    _pinSDIO = pinSDIO;
    _pinSCLK = pinSCLK;
    _enableInterrupts = enableInterrupts;
    _streamRDS = false;
    _resetRDS();
}

void Si470x::streamRDS(bool streamRDS) {
    _streamRDS = streamRDS;
    _resetRDS();
}

void Si470x::_readRegisters() {
    // Register address sequence: 0A,0B,0C,0D,0E,0F,00,01,02,03,04,05,06,07,08,09 = 32 bytes (16 words)
    Wire.requestFrom(I2C_ADDR, 32);
    while (Wire.available() < 32);

    for (uint8_t i = 0x0A; i <= 0x0F; i++) {
        _registers[i] = Wire.read() << 8 | Wire.read();
    }

    for (uint8_t i = 0x00; i <= 0x09; i++) {
        _registers[i] = Wire.read() << 8 | Wire.read();
    }
}

// Read register 0A into shadow
void Si470x::_readRegister0A() {
    Wire.requestFrom(I2C_ADDR, 2);
    _registers[STATUSRSSI] = Wire.read() << 8 | Wire.read();
}

byte Si470x::_updateRegisters() {
    Wire.beginTransmission(I2C_ADDR);

    for (uint8_t i = 0x02; i < 0x08; i++) {
        Wire.write(_registers[i] >> 8);                     // Upper 8 bits
        Wire.write(_registers[i] & 0x00FF);                 // Lower 8 bits
    }

    return Wire.endTransmission();
}

// Hardware initialization (see AN230 section 2.1.1)
void Si470x::_init() {
    // This function does steps 1 - 4 to bring device into a state where registers can be read and written.
    // Note that the breakout board has SEN pulled high. SDIO and SCLK are also pulled high.

    pinMode(_pinRST, OUTPUT);
    pinMode(_pinSDIO, OUTPUT);

    digitalWrite(_pinRST, LOW);                             // Keep RST pin low
    digitalWrite(_pinSDIO, LOW);                            // Select bus mode - low SDIO = a 2-wire interface
    delay(1);                                               // Allow pins to settle

    digitalWrite(_pinRST, HIGH);                            // Bring Si4703 out of reset
    delay(1);                                               // Allow Si4703 to come out of reset

    Wire.begin();                                           // Begin I2C
}

// Powerup sequence (see AN230 table 3)
void Si470x::_powerup() {
    // Write address 07h
    _readRegisters();
    _set(_registers[TEST1], XOSCEN, true);                  // Set the XOSCEN bit to power up the crystal
    _updateRegisters();
    delay(500);                                             // Wait for crystal to power up

    // Write address 02h
    _readRegisters();
    _registers[POWERCFG] = 0x4001;                          // Set to powerup state (= 0 | (1 << DMUTE) | (1 << ENABLE))
    _updateRegisters();
    delay(200);                                             // Wait for device powerup
}

// Get Seek/Tune Complete bit. Used for polling.
int Si470x::_getSTC() {
    _readRegister0A();
    return (_registers[STATUSRSSI] & (1 << STC));
}

void Si470x::begin() {
    _init();
    _powerup();

    // Configure registers (see AN230 table 1)

    _readRegisters();

    // Write remaining hardware configuration registers
    _set(_registers[TEST1], AHIZEN, 0);                                     // Disable Audio High-Z (default)
    _set(_registers[SYSCONFIG1], GPIO1, GPIO1_MASK, GPIO_Z);                // Clear GPIO1 bits - High impedance (default)
    _set(_registers[SYSCONFIG1], GPIO2, GPIO2_MASK, _enableInterrupts ? GPIO_I : GPIO_Z);   // Set RDS Interrupt
    _set(_registers[SYSCONFIG1], GPIO3, GPIO3_MASK, GPIO_Z);                // Clear GPIO3 bits - High impedance (default)
    _set(_registers[SYSCONFIG1], RDSIEN, _enableInterrupts ? 1 : 0);        // Set RDS Interrupt
    _set(_registers[SYSCONFIG1], STCIEN, 0);                                // Disable Seek/Tune Complete Interrupt (default)

    // Write the general configuration registers 
    _set(_registers[SYSCONFIG1], BLNDADJ, BLNDADJ_MASK, BLNDADJ_31_49);     // Set Stereo/Mono Blend Level Adjustment
    _set(_registers[POWERCFG], DSMUTE, 1);                                  // Disable Softmute
    // SMUTER
    // SMUTEA
    // VOLEXT
    // SEEKTH
    _set(_registers[SYSCONFIG3], SKSNR, SKSNR_DISABLED);                    // Set Seek SNR Threshold
    _set(_registers[SYSCONFIG3], SKCNT, SKCNT_DISABLED);                    // Set FM Impulse Detection Threshold
    _set(_registers[POWERCFG], RDSM, 1);                                    // Set RDS Mode to Verbose

    // Write the regional configuration registers (see AN230 section 3.4)
    _set(_registers[SYSCONFIG1], RDS, true);                                // Enable RDS

    // TODO: make these configurable
    _bandLowerLimit = 8750;
    _bandUpperLimit = 10800;
    _bandSpacing = 20;

    uint8_t band = BAND_US_EU;
    uint8_t spacing = SPACE_200KHz;
    uint8_t de = DE_75us;

    _set(_registers[SYSCONFIG2], BAND, BAND_MASK, band & 0b11);             // Set Band
    _set(_registers[SYSCONFIG2], SPACE, SPACE_MASK, spacing & 0b11);        // Set Channel Spacing
    _set(_registers[SYSCONFIG1], DE, de);                                   // Set De-emphasis

    _set(_registers[SYSCONFIG2], VOLUME, VOLUME_MASK, 0);                   // Set volume to 0

    _updateRegisters();
    delay(200);
}

bool Si470x::getSoftmute() {
    _readRegisters();
    return _get(_registers[POWERCFG], DSMUTE); // 0 = enable
}

void Si470x::setSoftmute(bool disabled) {
    _readRegisters();
    _set(_registers[POWERCFG], DSMUTE, disabled);
    _updateRegisters();
}

bool Si470x::getMute() {
    _readRegisters();
    return _get(_registers[POWERCFG], DMUTE) == 0; // 0 = enable
}

void Si470x::setMute(bool disabled) {
    _readRegisters();
    _set(_registers[POWERCFG], DMUTE, disabled);
    _updateRegisters();
}

bool Si470x::getMono() {
    _readRegisters();
    return _get(_registers[POWERCFG], MONO) == 1; // 1 = force mono
}

void Si470x::setMono(bool enabled) {
    _readRegisters();
    _set(_registers[POWERCFG], MONO, enabled);
    _updateRegisters();
}

void Si470x::setVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 15) vol = 15;

    _readRegisters();
    _set(_registers[SYSCONFIG2], VOLUME, VOLUME_MASK, vol & 0b1111);
    _set(_registers[POWERCFG], DMUTE, true);
    _updateRegisters();
}

int Si470x::getChannel() {
    _readRegisters();
    // freq (MHz) = spacing * channel + start
    return _bandSpacing * (_registers[READCHAN] & READCHAN_MASK) + _bandLowerLimit;
}

// Channel selection sequence (see AN230 table 15)
int Si470x::setChannel(int freq) {
    if (freq < _bandLowerLimit) freq = _bandLowerLimit;
    if (freq > _bandUpperLimit) freq = _bandUpperLimit;

    // freq (MHz) = spacing * channel + start
    int channel = (freq - _bandLowerLimit) / _bandSpacing;

    _readRegisters();
    _set(_registers[CHANNEL], CHAN, CHAN_MASK, channel & 0x3FF);            // Channel Select
    _set(_registers[CHANNEL], TUNE, true);                                  // Set Tune bit
    _updateRegisters();

    // TODO: use interrupts! Polling is not recommended as it can interfere with the tuning operation.

    delay(10);
    while (_getSTC() == 0) delay(1);                                        // Poll until STC bit is set

    _readRegisters();
    _set(_registers[CHANNEL], TUNE, false);                                 // Clear Tune bit
    _updateRegisters();

    while (_getSTC() != 0) delay(1);                                        // Poll until STC bit is cleared

    _resetRDS();

    return getChannel();
}

int Si470x::getRSSI() {
    _readRegister0A();
    return _get(_registers[STATUSRSSI], RSSI, RSSI_MASK);
}

// Seek up/seek down sequence (see table 14)
int Si470x::_seek(uint8_t dir) {
    _readRegisters();
    _set(_registers[POWERCFG], SKMODE, SKMODE_WRAP);                        // Wrap
    _set(_registers[POWERCFG], SEEKUP, dir);                                // Set direction
    _set(_registers[POWERCFG], SEEK, true);                                 // Set Seek bit
    _updateRegisters();

    while (_getSTC() == 0) delay(1);                                        // Poll until STC bit is set

    _readRegisters();
    bool sfbl = _registers[STATUSRSSI] & (1 << SF_BL);                      // Get seek status
    _set(_registers[POWERCFG], SEEK, false);                                // Clear Seek bit
    _updateRegisters();

    while (_getSTC() != 0) delay(1);                                        // Poll until STC bit is cleared

    if (sfbl) {
        return 0;
    }

    _resetRDS();

    return getChannel();
}

void Si470x::_resetRDS() {
    _rdsPI = 0;
    _rdsPTY = 0;

    // PS
    _rdsRecvSegments = 0;
    _rdsPS[0] = '\0';
    memset(_rdsPS, '\0', 8);
    memset(_rdsPS1, '\0', 8);
    memset(_rdsPS2, '\0', 8);
    memset(_rdsPS3, '\0', 8);

    // RT
    _rdsABFlag = 0;
    _rdsPrevABFlag = 0;
    _rdsIdx = 0;
    memset(_rdsRT, '\0', 64);
    memset(_rdsRTBuffer, '\0', 64);
}

// RDS sequence (see table 16)
bool Si470x::checkRDS() {
    unsigned long currMills = millis();
    if (currMills - _rdsMillis < 40) {
        // Polling too quick.
        return false;
    }
    _rdsMillis = currMills;

    _readRegisters();
    if (_registers[STATUSRSSI] & (1 << RDSR) == 0) {
        return false;
    }

    uint16_t blockA = _registers[RDSA];
    uint16_t blockB = _registers[RDSB];
    uint16_t blockC = _registers[RDSC];
    uint16_t blockD = _registers[RDSD];

    uint8_t errA = _get(_registers[STATUSRSSI], BLERA, BLERA_MASK);
    if (errA < 3) {
        _rdsPI = blockA;
    }

    uint8_t errB = _get(_registers[READCHAN], BLERB, BLERB_MASK);
    if (errB > 2) {
        return false;
    }

    uint8_t _rdsPTY = (blockB & 0x03E0) >> 5;
    uint8_t errD = _get(_registers[READCHAN], BLERD, BLERD_MASK);

    uint16_t groupType = 0x0A | (blockB & 0xF000) >> 8 | (blockB & 0x0800) >> 11;
    switch (groupType) {
        case 0x0A:
        case 0x0B: {
            if (errD >= 3) break;
            uint8_t idx = 2 * (blockB & 0b11);

            if (_streamRDS) {
                _rdsPS[idx] = blockD >> 8;
                _rdsPS[idx + 1] = blockD & 0x00FF;
                break;
            }

            _rdsPS3[idx] = _rdsPS2[idx];
            _rdsPS3[idx + 1] = _rdsPS2[idx + 1];
            _rdsPS2[idx] = _rdsPS1[idx];
            _rdsPS2[idx + 1] = _rdsPS1[idx + 1];
            _rdsPS1[idx] = blockD >> 8;
            _rdsPS1[idx + 1] = blockD & 0x00FF;

            if (idx == 6) {
                if (strcmp(_rdsPS1, _rdsPS2) == 0 || strcmp(_rdsPS1, _rdsPS3) == 0) {
                    strcpy(_rdsPS, _rdsPS1);
                } else if (strcmp(_rdsPS2, _rdsPS3) == 0) {
                    strcpy(_rdsPS, _rdsPS2);
                }
            }

            break;
        }

        case 0x2A: {
            _rdsABFlag = (blockB & 0x0010);
            uint8_t idx = 4 * (blockB & 0x000F);
            if (idx < _rdsIdx) {
                strcpy(_rdsRT, _rdsRTBuffer);
            }
            _rdsIdx = idx;

            if (_rdsABFlag != _rdsPrevABFlag) {
                _rdsPrevABFlag = _rdsABFlag;
                memset(_rdsRTBuffer, '\0', 64);
            }

            _rdsRTBuffer[idx++] = (blockC >> 8);
            _rdsRTBuffer[idx++] = (blockC & 0x00FF);
            _rdsRTBuffer[idx++] = (blockD >> 8);
            _rdsRTBuffer[idx++] = (blockD & 0x00FF);
            break;
        }
        default:
            break;
    }

    return true;
}

uint16_t Si470x::getProgramID() {
    return _rdsPI;
}

uint8_t Si470x::getProgramType() {
    return _rdsPTY;
}

const char* Si470x::getStationName() {
    return _rdsPS;
}

const char* Si470x::getRadioText() {
    return _rdsRT;
}

int Si470x::getPartNumber() {
    return _get(_registers[DEVICEID], PN, PN_MASK);
}

int Si470x::getManufacturerID() {
    return _get(_registers[DEVICEID], MFGID, MFGID_MASK);
}

int Si470x::getChipVersion() {
    return _get(_registers[CHIPID], REV, REV_MASK);
}

int Si470x::getDevice() {
    return _get(_registers[CHIPID], DEV, DEV_MASK);
}

int Si470x::getFirmwareVersion() {
    return _get(_registers[CHIPID], FIRMWARE, FIRMWARE_MASK);
}


// private helper functions

uint16_t Si470x::_get(uint16_t reg, uint16_t shift) {
    return (reg & (1 << shift)) >> shift;
}

uint16_t Si470x::_get(uint16_t reg, uint16_t shift, uint16_t mask) {
    return (reg & mask) >> shift;
}

void Si470x::_set(uint16_t& reg, uint16_t shift, bool val) {
    if (val) reg |= (1 << shift);
    else reg &= ~(1 << shift);
}

void Si470x::_set(uint16_t& reg, uint16_t shift, uint16_t mask, uint16_t field) {
    reg &= ~mask;
    if (field) reg |= (field << shift) & mask;
}
