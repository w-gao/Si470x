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

    for (int i = 0x02; i < 0x08; i++) {
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
    digitalWrite(_pinSDIO, LOW);                            // Select bus mode - low SDIO indicates a 2-wire interface
    delay(1);                                               // Allow pins to settle

    digitalWrite(_pinRST, HIGH);                            // Bring Si4703 out of reset
    delay(1);                                               // Allow Si4703 to come out of reset

    Wire.begin();                                           // Begin I2C
}

// Powerup sequence (see AN230 table 3)
void Si470x::_powerup() {
    // Write address 07h
    _readRegisters();
    _registers[TEST1] |= (1 << XOSCEN);                     // Set the XOSCEN bit to power up the crystal
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

    _readRegisters();

    // Regional configuration registers (TODO: make these configurable) (see AN230 section 3.4)
    _bandLow = 8750;
    _bandHigh = 10800;
    _bandSpacing = 20;

    uint8_t band = BAND_US_EU;
    uint8_t spacing = SPACE_200KHz;
    uint8_t de = DE_75us;

    _registers[SYSCONFIG2] &= ~BAND_MASK;
    _registers[SYSCONFIG2] |= (band & 0b11) << BAND;
    _registers[SYSCONFIG2] &= ~(SPACE_MASK);
    _registers[SYSCONFIG2] |= (spacing & 0b11) << SPACE;
    _registers[SYSCONFIG1] &= ~(1 << DE);
    if (de == DE_50us) _registers[SYSCONFIG1] |= (1 << DE);

    // Enable RDS
    _registers[SYSCONFIG1] |= (1 << RDS);

    // Set volume to 1
    int volume = 1;
    _registers[SYSCONFIG2] &= ~VOLUME_MASK;
    _registers[SYSCONFIG2] |= (volume & 0b1111) << VOLUME;

    _updateRegisters();
}

void Si470x::setVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 15) vol = 15;

    _readRegisters();
    _registers[SYSCONFIG2] &= ~VOLUME_MASK;
    _registers[SYSCONFIG2] |= (vol & 0b1111) << VOLUME;
    _updateRegisters();
}

// Channel selection sequence (see AN230 table 15)
void Si470x::selectChannel(int freq) {
    freq *= 10;

    if (freq < _bandLow) freq = _bandLow;
    if (freq > _bandHigh) freq = _bandHigh;

    // freq (MHz) = spacing * channel + _bandLow
    int channel = (freq - _bandLow) / _bandSpacing;

    _readRegisters();
    _registers[CHANNEL] &= ~CHAN_MASK;
    _registers[CHANNEL] |= channel & 0x3FF;                 // Channel Select
    _registers[CHANNEL] |= (1 << TUNE);                     // Set Tune bit
    _updateRegisters();

    while (_getSTC() == 0) delay(10);                       // Poll until STC bit is set

    // optionally, we can read ST, RSSI, and READCHAN

    _readRegisters();
    _registers[CHANNEL] &= ~(1 << TUNE);                    // Clear Tune bit
    _updateRegisters();

    while (_getSTC() != 0) delay(10);                       // Poll until STC bit is cleared
}

int Si470x::getPartNumber() {
    return (_registers[DEVICEID] & PN_MASK) >> PN;
}

int Si470x::getManufacturerID() {
    return (_registers[DEVICEID] & MFGID_MASK) >> MFGID;
}

int Si470x::getChipVersion() {
    return (_registers[CHIPID] & REV_MASK) >> REV;
}

int Si470x::getDevice() {
    return (_registers[CHIPID] & DEV_MASK) >> DEV;
}

int Si470x::getFirmwareVersion() {
    return (_registers[CHIPID] & FIRMWARE_MASK) >> FIRMWARE;
}
