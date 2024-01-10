#include "Si470x.h"
#include "Wire.h"

Si470x::Si470x(int pinRST, int pinSDIO, int pinSCLK) {
    _pinRST = pinRST;
    _pinSDIO = pinSDIO;
    _pinSCLK = pinSCLK;
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
    _set(_registers[SYSCONFIG1], GPIO2, GPIO2_MASK, GPIO_Z);                // Clear GPIO2 bits - High impedance (default)
    _set(_registers[SYSCONFIG1], GPIO3, GPIO3_MASK, GPIO_Z);                // Clear GPIO3 bits - High impedance (default)
    _set(_registers[SYSCONFIG1], RDSIEN, 0);                                // Disable RDS Interrupt (default)
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
    // RDSPRF
    _set(_registers[POWERCFG], RDSM, 0);                                    // Set RDS Mode to Standard (default)

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
    _updateRegisters();
}

int Si470x::getChannel() {
    _readRegisters();
    // freq (MHz) = spacing * channel + start
    return _bandSpacing * (_registers[READCHAN] & READCHAN_MASK) + _bandLowerLimit;
}

// Channel selection sequence (see AN230 table 15)
void Si470x::selectChannel(int freq) {
    if (freq < _bandLowerLimit) freq = _bandLowerLimit;
    if (freq > _bandUpperLimit) freq = _bandUpperLimit;

    // freq (MHz) = spacing * channel + start
    int channel = (freq - _bandLowerLimit) / _bandSpacing;

    _readRegisters();
    _set(_registers[CHANNEL], CHAN, CHAN_MASK, channel & 0x3FF);            // Channel Select
    _set(_registers[CHANNEL], TUNE, true);                                  // Set Tune bit
    _updateRegisters();

    delay(60);
    while (_getSTC() == 0) delay(1);                                        // Poll until STC bit is set

    _readRegisters();
    _set(_registers[CHANNEL], TUNE, false);                                 // Clear Tune bit
    _updateRegisters();

    while (_getSTC() != 0) delay(1);                                        // Poll until STC bit is cleared
}

int Si470x::getRSSI() {
    _readRegister0A();
    return _get(_registers[STATUSRSSI], RSSI, RSSI_MASK);
}

int Si470x::seekUp() {
    return _seek(SEEKUP_UP);
}

int Si470x::seekDown() {
    return _seek(SEEKUP_DOWN);
}

// Seek up/seek down Sequence (see table 14)
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

    return getChannel();
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
