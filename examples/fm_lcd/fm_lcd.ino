// fm_lcd.ino
//
// FM radio w/ LCD screen
//
// Created on Jan 11, 2024

#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "Si470x.h"
#include "RotaryEncoder.h"

#define PIN_RESET D2
#define PIN_SDIO A4
#define PIN_SCLK A5
#define PIN_EXTERN_MUTE D3
Si470x radio(PIN_RESET, PIN_SDIO, PIN_SCLK);

// Volume control
#define PIN_VOL_DT D5
#define PIN_VOL_CLK D4
#define PIN_VOL_SW D6

#define VOLUME_MIN 0
#define VOLUME_MAX 15

RotaryEncoder volumeControl(PIN_VOL_DT, PIN_VOL_CLK, PIN_VOL_SW);

// Channel control
#define PIN_CHAN_DT D8
#define PIN_CHAN_CLK D7
#define PIN_CHAN_SW D9

RotaryEncoder channelControl(PIN_CHAN_DT, PIN_CHAN_CLK, PIN_CHAN_SW);

// LCD screen
// SCK --> clock
// SDA --> COPI/D11
// RES --> reset
// DC --> CIPO/D12
// CS --> chip select
U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ D13, /* data=*/ D11, /* cs=*/ A1, /* dc=*/ D12, /* reset=*/ A0);

const int _channelCount = 47;
int _channels[_channelCount] = {
    8810, 8850, 8930, 8970, 9010, 9030, 9070, 9110, 9130, 9150,
    9170, 9210, 9230, 9330, 9370, 9410, 9450, 9490, 9530, 9570,
    9610, 9650, 9730, 9770, 9810, 9850, 9890, 9910, 9970, 10030,
    10090, 10130, 10170, 10210, 10250, 10290, 10330, 10370, 10410, 10450,
    10530, 10570, 10610, 10650, 10690, 10730, 10770
};
int _channelIndex = 32; // 101.7 MHz

// const int _channelCount = 4;
// int _channels[_channelCount] = {9730, 10170, 10610, 10690};
// int _channelIndex = 1; // 101.7 MHz


int volume = 8;
int channel = _channels[_channelIndex];
bool mono = false;
bool enableRDS = true;

unsigned long _statusMillis = 0;
const long _statusInterval = 2000;
int _rssi;

const char *_stationName;
char _rdsBuffer[10];


void setup() {
    pinMode(PIN_EXTERN_MUTE, OUTPUT);
    digitalWrite(PIN_EXTERN_MUTE, HIGH);

    // set up controls
    volumeControl.setup();
    channelControl.setup();

    // set up LCD screen
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setContrast(5);
    u8g2.setFont(u8g2_font_t0_11_mr);
    u8g2.setFontDirection(0);

    Serial.begin(9600);
    // while (!Serial) delay(10);
    delay(500);

    // set up Si470x
    Serial.println("Setting up Si470x...");
    radio.begin();

    digitalWrite(PIN_EXTERN_MUTE, LOW);

    Serial.println("\n\nSi470x Serial Interface");
    Serial.println("==================================================");  
    Serial.println("+ -     Volume (max 15)");
    Serial.println("u d     Seek up / down");
    Serial.println("f b     Forward / backward saved channels");
    Serial.println("t       Toggle mono/stereo");
    Serial.println("i       Show Si470x device information");
    // Serial.println("r       Listen for RDS (15 sec timeout)");
    Serial.println("==================================================");  

    radio.setVolume(volume);
    radio.setMono(mono);
    radio.setChannel(channel);
    volumeControl.setValue(volume);
    channelControl.setValue(_channelIndex);

    updateLCD();
}

void printDeviceInfo() {
    Serial.print("Part Number:\t");
    Serial.println(radio.getPartNumber());

    Serial.print("Manufacturer ID:\t0x");
    Serial.println(radio.getManufacturerID(), HEX);

    Serial.print("Version:\t");
    Serial.println(radio.getChipVersion());

    Serial.print("Device:\t0b");
    Serial.println(radio.getDevice(), BIN);

    Serial.print("Firmware:\t");
    Serial.println(radio.getFirmwareVersion());
}

void printStatus() {
    updateLCD();

    // channel = radio.getChannel();

    Serial.print("| Chan: ");
    Serial.print(float(channel) / 100, 2);
    Serial.print(" MHz");

    Serial.print(" | RSSI: ");
    Serial.print(_rssi);
    Serial.print(" dBuV");

    Serial.print(" | Vol: ");
    Serial.print(volume);

    Serial.print(" | ");
    Serial.print(mono ? "Mono" : "Stereo");

    Serial.print(" | PI code: 0x");
    Serial.print(radio.getRDSPICode(), HEX);

    Serial.print(" | Station name: ");
    Serial.print(radio.getStationName());

    Serial.println();
}

void updateLCD() {
    u8g2.clearBuffer();

    u8g2.setCursor(0, 12);
    u8g2.print("Chan: ");
    u8g2.print(float(channel) / 100, 2);
    u8g2.print(" MHz");

    u8g2.setCursor(0, 24);
    u8g2.print("RSSI: ");
    u8g2.print(_rssi);
    u8g2.print(" dBuV");

    u8g2.setCursor(0, 36);
    u8g2.print("Vol:  ");
    u8g2.print(volume);
    u8g2.print(" (");
    u8g2.print(mono ? "Mono" : "Stereo");
    u8g2.println(")");

    u8g2.setCursor(0, 48);
    if (enableRDS) u8g2.print(radio.getStationName());

    u8g2.sendBuffer();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - _statusMillis >= _statusInterval) {
        // poll rssi
        _rssi = radio.getRSSI();

        // periodically print status
        _statusMillis = currentMillis;
        printStatus();
    }

    // check volume control
    int newVolume = volumeControl.update();
    if (newVolume < VOLUME_MIN) {
        volumeControl.setValue(VOLUME_MIN);
        newVolume = VOLUME_MIN;
    }

    if (newVolume > VOLUME_MAX) {
        volumeControl.setValue(VOLUME_MAX);
        newVolume = VOLUME_MAX;
    }

    if (volume != newVolume) {
        volume = newVolume;
        radio.setVolume(volume);
        printStatus();
    }

    if (volumeControl.pressed()) {
        radio.setMute(!radio.getMute());
    }

    // check channel control
    int newChannelIndex = channelControl.update();
    if (newChannelIndex < 0) {
        channelControl.setValue(_channelCount - 1);
        newChannelIndex = _channelCount - 1;
    }

    if (newChannelIndex > _channelCount - 1) {
        channelControl.setValue(0);
        newChannelIndex = 0;
    }

    if (_channelIndex != newChannelIndex) {
        _channelIndex = newChannelIndex;
        channel = _channels[_channelIndex];
        radio.setChannel(channel);
        printStatus();
    }

    if (channelControl.pressed()) {
        enableRDS = !enableRDS;
    }

    // check serial input
    if (Serial.available()) {
        char ch = Serial.read();
        if (ch == 'i') {
            printDeviceInfo();
        } else if (ch == '+') {
            volume++;
            if (volume > 15) volume = 15;
            radio.setVolume(volume);
            volumeControl.setValue(volume);
            printStatus();
        } else if (ch == '-') {
            volume--;
            if (volume < 0) volume = 0;
            radio.setVolume(volume);
            volumeControl.setValue(volume);
            printStatus();
        } else if (ch == 't') {
            mono = !mono;
            radio.setMono(mono);
            printStatus();
        } else if (ch == 'u') {
            channel = radio.seekUp();
            printStatus();
        } else if (ch == 'd') {
            channel = radio.seekDown();
            printStatus();
        } else if (ch == 'f') {
            _channelIndex = (_channelIndex + 1) % _channelCount;
            channel = _channels[_channelIndex];
            radio.setChannel(channel);
            printStatus();
        } else if (ch == 'b') {
            _channelIndex = (_channelIndex - 1 + _channelCount) % _channelCount;
            channel = _channels[_channelIndex];
            radio.setChannel(channel);
            printStatus();
        }
    }

    // poll RDS
    if (enableRDS && radio.pollRDS()) {
        // true = we got something
    }
}
