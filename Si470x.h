#ifndef Si470X_H
#define Si470X_H

#include "Arduino.h"

// BAND ranges
static const uint8_t BAND_US_EU = 0b00;	                // 87.5–108 MHz (US / Europe)
static const uint8_t BAND_JPW = 0b01;                   // 76–108 MHz (Japan wide)
static const uint8_t BAND_JP = 0b10;                    // 76–90 MHz (Japan)

// Channel Spacing
static const uint8_t SPACE_200KHz = 0b00;               // 200 kHz (US / Australia)
static const uint8_t SPACE_100KHz = 0b01;               // 100 kHz (Europe / Japan)
static const uint8_t SPACE_50KHz = 0b10;                //  50 kHz (Other)

// De-emphasis
static const uint8_t DE_75us = 0b0;                     // De-emphasis 75 μs (US)
static const uint8_t DE_50us = 0b1;                     // De-emphasis 50 µs (Europe / Australia / Japan)

// Stereo/Mono Blend Level Adjustment
static const uint8_t BLNDADJ_31_49 = 0b00;              // 31–49 RSSI dBµV (default)
static const uint8_t BLNDADJ_37_55 = 0b01;              // 37–55 RSSI dBμV (+6 dB)
static const uint8_t BLNDADJ_19_37 = 0b10;              // 19–37 RSSI dBμV (–12 dB)
static const uint8_t BLNDADJ_25_43 = 0b11;              // 25–43 RSSI dBμV (–6 dB)

// Softmute Attack/Recover Rate
static const uint8_t SMUTER_FASTEST = 0b00;             // Fastest (default)
static const uint8_t SMUTER_FAST = 0b01;                // Fast
static const uint8_t SMUTER_SLOW = 0b10;                // Slow
static const uint8_t SMUTER_SLOWEST = 0b11;             // Slowest

// GPIO values
static const uint8_t GPIO_Z = 0b00;                     // High impedance (default)
static const uint8_t GPIO_I = 0b01;                     // GPIO1: Reserved, GPIO2: STC/RDS interrupt, GPIO3:  Mono/Stereo indicator
static const uint8_t GPIO_Low = 0b10;                   // Low output (GND level)
static const uint8_t GPIO_High = 0b11;                  // High output (VIO level)

// Seek Direction
static const uint8_t SEEKUP_DOWN = 0;                   // Seek down (default)
static const uint8_t SEEKUP_UP = 1;                     // Seek up

// Seek Mode
static const uint8_t SKMODE_WRAP = 0;                   // Wrap at the upper or lower band limit and continue seeking (default)
static const uint8_t SKMODE_STOP = 1;                   // Stop seeking at the upper or lower band limit

// Seek SNR Threshold
static const uint8_t SKSNR_DISABLED = 0b0000;           // disabled (default)
static const uint8_t SKSNR_MIN = 0b0001;                // min (most stops)
static const uint8_t SKSNR_MAX = 0b0111;                // max (fewest stops)

// Seek FM Impulse Detection Threshold
static const uint8_t SKCNT_DISABLED = 0b0000;           // disabled (default)
static const uint8_t SKCNT_MIN = 0b0001;                // max (most stops)
static const uint8_t SKCNT_MAX = 0b1111;                // min (fewest stops)


class Si470x {
  public:
    Si470x(int pinRST, int pinSDIO, int pinSCLK);
    void begin();

    int getPartNumber();                                // Get the part number
    int getManufacturerID();                            // Get the manufacturer ID
    int getChipVersion();                               // Get the chip version (silicon revision)
    int getDevice();                                    // Get the identity of device
    int getFirmwareVersion();                           // Get the firmware revision

    bool getMute();                                     // Get mute status
    void setMute(bool disabled);                        // Set mute (false = mute)
    void setMono(bool enabled);                         // Set mono output (false = stereo)
    void setVolume(int vol);			                      // Set volume. (0 <= vol <= 15)
    int getChannel();                                   // Get current channel
    int setChannel(int freq);		                        // Set channel (e.g., freq = 10170 => 101.7 MHz)
    int getRSSI();                                      // Get RSSI (Received Signal Strength Indicator) in dBµV

    // Seek up. Returns the tuned channel or 0.
    inline int seekUp() { return _seek(SEEKUP_UP); }

    // Seek down. Returns the tuned channel or 0.
    inline int seekDown() { return _seek(SEEKUP_DOWN); }

    // Increment channel by one step. Returns next channel (not necessarily tuned) or 0.
    inline int incChannel() { return setChannel(getChannel() + _bandSpacing); }

    // Decrement channel by one step. Returns previous channel (not necessarily tuned) or 0.
    inline int decChannel() { return setChannel(getChannel() - _bandSpacing); }

    bool pollRDS();                                     // Poll RDS
    uint16_t getRDSPICode();                            // Return the program identification (PI) code
    uint8_t getPTY();                                   // Return the program type (PTY) code
    const char* getStationName();                       // Return the station name
    void readRDS(char* buffer, long timeout); /// temp

  private:
    int _pinRST;
    int _pinSDIO;
    int _pinSCLK;
    uint16_t _registers[16]; // shadow registers

    int _bandLowerLimit;
    int _bandUpperLimit;
    int _bandSpacing;

    unsigned long _rdsMillis;
    // unsigned long _setChannelMillis;
    uint16_t _rdsPICode;
    uint8_t _pty;
    char _stationName[9];
    char _stationNameA[9];
    char _stationNameB[9];
    char _stationNameC[9];

    void _readRegisters();
    void _readRegister0A();
    byte _updateRegisters();

    void _init();
    void _powerup();

    int _getSTC();
    int _seek(uint8_t dir);

    // RDS
    void _processStationName(uint16_t blockB, uint16_t blockD);

    uint16_t _get(uint16_t reg, uint16_t shift);
    uint16_t _get(uint16_t reg, uint16_t shift, uint16_t mask);
    void _set(uint16_t& reg, uint16_t shift, bool val);
    void _set(uint16_t& reg, uint16_t shift, uint16_t mask, uint16_t field);

    static const int I2C_ADDR = 0x10;

    // Register definition (see Si4702/03-C19 pages 22-36)

    // Register names
    static const uint16_t DEVICEID = 0x00;              // Device ID
    static const uint16_t CHIPID = 0x01;                // Chip ID
    static const uint16_t POWERCFG = 0x02;              // Power Configuration
    static const uint16_t CHANNEL = 0x03;               // Channel
    static const uint16_t SYSCONFIG1 = 0x04;            // System Configuration 1
    static const uint16_t SYSCONFIG2 = 0x05;            // System Configuration 2
    static const uint16_t SYSCONFIG3 = 0x06;            // System Configuration 3
    static const uint16_t TEST1 = 0x07;                 // Test 1
    static const uint16_t STATUSRSSI = 0x0A;            // Status RSSI
    static const uint16_t READCHAN = 0x0B;              // Read Channel
    static const uint16_t RDSA = 0x0C;                  // RDS Block A
    static const uint16_t RDSB = 0x0D;                  // RDS Block B
    static const uint16_t RDSC = 0x0E;                  // RDS Block C
    static const uint16_t RDSD = 0x0F;                  // RDS Block D

    // Register 0x00 - DEVICEID
    static const uint16_t PN = 12;                      // Part Number
    static const uint16_t PN_MASK = 0xF000;
    static const uint16_t MFGID = 0;                    // Manufacturer ID
    static const uint16_t MFGID_MASK = 0x0FFF;

    // Register 0x01 - CHIPID
    static const uint16_t REV = 10;                     // Chip Version
    static const uint16_t REV_MASK = 0xFC00;
    static const uint16_t DEV = 6;                      // Device
    static const uint16_t DEV_MASK = 0x03C0;
    static const uint16_t FIRMWARE = 0;                 // Firmware Version
    static const uint16_t FIRMWARE_MASK = 0x003F;

    // Register 0x02 - POWERCFG
    static const uint16_t DSMUTE = 15;                  // Softmute Disable
    static const uint16_t DMUTE = 14;                   // Mute Disable
    static const uint16_t MONO = 13;                    // Mono Select
    static const uint16_t RDSM = 11;                    // RDS Mode
    static const uint16_t SKMODE = 10;                  // Seek Mode
    static const uint16_t SEEKUP = 9;                   // Seek Direction
    static const uint16_t SEEK = 8;                     // Seek
    static const uint16_t DISABLE = 6;                  // Powerup Disable
    static const uint16_t ENABLE = 0;                   // Powerup Enable

    // Register 0x03 - CHANNEL
    static const uint16_t TUNE = 15;                    // Tune
    static const uint16_t CHAN = 0;                     // Channel Select
    static const uint16_t CHAN_MASK = 0x03FF;

    // Register 0x04 - SYSCONFIG1
    static const uint16_t RDSIEN = 15;                  // RDS Interrupt Enable
    static const uint16_t STCIEN = 14;                  // Seek/Tune Complete Interrupt Enable
    static const uint16_t RDS = 12;                     // RDS Enable
    static const uint16_t DE = 11;                      // De-emphasis
    static const uint16_t AGCD = 10;                    // AGC Disable
    static const uint16_t BLNDADJ = 6;                  // Stereo/Mono Blend Level Adjustment
    static const uint16_t BLNDADJ_MASK = 0x00C0;
    static const uint16_t GPIO3 = 4;                    // General Purpose I/O 3
    static const uint16_t GPIO3_MASK = 0x0030;
    static const uint16_t GPIO2 = 2;                    // General Purpose I/O 2
    static const uint16_t GPIO2_MASK = 0x000C;
    static const uint16_t GPIO1 = 0;                    // General Purpose I/O 1
    static const uint16_t GPIO1_MASK = 0x0003;

    // Register 0x05 - SYSCONFIG2
    static const uint16_t SEEKTH = 8;                   // RSSI Seek Threshold
    static const uint16_t SEEKTH_MASK = 0xFF00;
    static const uint16_t BAND = 6;                     // Band Select
    static const uint16_t BAND_MASK = 0x00C0;
    static const uint16_t SPACE = 4;                    // Channel Spacing
    static const uint16_t SPACE_MASK = 0x0030;
    static const uint16_t VOLUME = 0;                   // Volume
    static const uint16_t VOLUME_MASK = 0x000F;

    // Register 0x06 - SYSCONFIG3
    static const uint16_t SMUTER = 14;                  // Softmute Attack/Recover Rate
    static const uint16_t SMUTER_MASK = 0xC000;
    static const uint16_t SMUTEA = 12;                  // Softmute Attenuation
    static const uint16_t SMUTEA_MASK = 0x3000;
    static const uint16_t VOLEXT = 8;                   // Extended Volume Range
    static const uint16_t VOLEXT_MASK = 0x0100;
    static const uint16_t SKSNR = 4;                    // Seek SNR Threshold
    static const uint16_t SKSNR_MASK = 0x00F0;
    static const uint16_t SKCNT = 0;                    // Seek FM Impulse Detection Threshold
    static const uint16_t SKCNT_MASK = 0x000F;

    // Register 0x07 - TEST1
    static const uint16_t XOSCEN = 15;                  // Crystal Oscillator Enable
    static const uint16_t AHIZEN = 14;                  // Audio High-Z Enable

    // Register 0x0A - STATUSRSSI
    static const uint16_t RDSR = 15;                    // RDS Ready
    static const uint16_t STC = 14;                     // Seek/Tune Complete
    static const uint16_t SF_BL = 13;                   // Seek Fail/Band Limit
    static const uint16_t AFCRL = 12;                   // AFC Rail
    static const uint16_t RDSS = 11;                    // RDS Synchronized
    static const uint16_t BLERA = 9;                    // RDS Block A Errors
    static const uint16_t BLERA_MASK = 0x0600;
    static const uint16_t ST = 8;                       // Stereo Indicator
    static const uint16_t RSSI = 0;                     // RSSI (Received Signal Strength Indicator)
    static const uint16_t RSSI_MASK = 0x00FF;

    // Register 0x0B - Read Channel
    static const uint16_t BLERB = 14;                   // RDS Block B Errors
    static const uint16_t BLERB_MASK = 0xC000;
    static const uint16_t BLERC = 12;                   // RDS Block C Errors
    static const uint16_t BLERC_MASK = 0x3000;
    static const uint16_t BLERD = 10;                   // RDS Block D Errors
    static const uint16_t BLERD_MASK = 0x0C00;
    // static const uint16_t READCHAN_ = 0;             // Read Channel
    static const uint16_t READCHAN_MASK = 0x03FF;
};

#endif
