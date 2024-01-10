#ifndef Si470X_H
#define Si470X_H

#include "Arduino.h"

// Regional register configuration (see AN230.pdf section 3.4)

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



class Si470x
{
  public:
    Si470x(int pinRST, int pinSDIO, int pinSCLK);
    void begin();

	int getPartNumber();                // Get the part number
	int getManufacturerID();            // Get the manufacturer ID
	int getChipVersion();               // Get the chip version (silicon revision)
	int getDevice();                    // Get the identity of device
	int getFirmwareVersion();           // Get the firmware revision

	void setVolume(int vol);			// Set volume. (0 <= vol <= 15)
	void selectChannel(int freq);		// Set channel

  private:
    int  _pinRST;
	int  _pinSDIO;
	int  _pinSCLK;
	uint16_t _registers[16]; // shadow registers

    int _bandLow;
    int _bandHigh;
    int _bandSpacing;

	void _readRegisters();
	void _readRegister0A();
	byte _updateRegisters();

	void _init();
	void _powerup();

	int _getSTC();

	static const int I2C_ADDR = 0x10;

    // Register definition (see Si4702/03-C19 pages 22-36)

	// Register names
	static const uint16_t DEVICEID = 0x00;              // Device ID
	static const uint16_t CHIPID = 0x01;                // Chip ID
	static const uint16_t POWERCFG = 0x02;              // Power Configuration
	static const uint16_t CHANNEL = 0x03;               // Channel
	static const uint16_t SYSCONFIG1 = 0x04;            // System Configuration 1
	static const uint16_t SYSCONFIG2 = 0x05;            // System Configuration 2
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
};

#endif
