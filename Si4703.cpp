#include "Wire.h"
#include "Si4703.h"

Si4703::Si4703(int pinRST, int pinSDIO, int pinSCLK)
{
  _pinRST = pinRST;
  _pinSDIO = pinSDIO;
  _pinSCLK = pinSCLK;
}

void Si4703::_readRegisters(){
  // Register address sequence: 0A,0B,0C,0D,0E,0F,00,01,02,03,04,05,06,07,08,09 = 32 bytes (16 words)
  Wire.requestFrom(SI4703, 32);
  while(Wire.available() < 32);

  for(int i = 0x0A; ; i++) {
    if(i == 0x10) i = 0;
    _registers[i] = Wire.read() << 8 | Wire.read();
    if(i == 0x09) break;
  }
}

byte Si4703::_updateRegisters() {
  Wire.beginTransmission(SI4703);

  for(int i = 0x02; i < 0x08; i++) {
    Wire.write(_registers[i] >> 8);        // Upper 8 bits
    Wire.write(_registers[i] & 0x00FF);    // Lower 8 bits
  }

  return Wire.endTransmission();
}

// https://cdn.sparkfun.com/datasheets/BreakoutBoards/AN230.pdf

// Hardware Initialization
void Si4703::_init()
{
  // See section 2.1.1
  // This function does section 2.1.1 steps 1 - 4 to bring device into a state where registers can be read and written.
  // Note that the breakout board has SEN pulled high. SDIO and SCLK are also pulled high.

  pinMode(_pinRST, OUTPUT);
  pinMode(_pinSDIO, OUTPUT);

  digitalWrite(_pinRST, LOW);                           // Keep RST pin low
  digitalWrite(_pinSDIO, LOW);                          // Select bus mode - low SDIO indicates a 2-wire interface
  delay(1);                                             // Allow pins to settle

  digitalWrite(_pinRST, HIGH);                          // Bring Si4703 out of reset
  delay(1);                                             // Allow Si4703 to come out of reset

  Wire.begin();                                         // Begin I2C
}

void Si4703::_powerUp()
{
  // See table 3 - Powerup Configuration Sequence

  // Write address 07h
  _readRegisters();
  _registers[TEST1] = 0 | (1 << XOSCEN);                 // Set the XOSCEN bit to power up the crystal
  _updateRegisters();
  delay(500);                                           // Wait for crystal to power up

  // Write address 02h
  _readRegisters();
  _registers[POWERCFG] = 0x4001;                         // Set to powerup state ((1 << DMUTE) | (1 << ENABLE))
  _updateRegisters();
  delay(110);                                           // Wait for device powerup
}

void Si4703::begin()
{
  _init();
  _powerUp();


}

int Si4703::getPN()
{
  return (_registers[DEVICEID] & PN_MASK) >> PN;
}

int Si4703::getMFGID()
{
  return (_registers[DEVICEID] & MFGID_MASK) >> MFGID;
}

int Si4703::getREV()
{
  return (_registers[CHIPID] & REV_MASK) >> REV;
}

int Si4703::getDEV()
{
  return (_registers[CHIPID] & DEV_MASK) >> DEV;
}

int Si4703::getFIRMWARE()
{
  return _registers[CHIPID] & FIRMWARE_MASK;
}
