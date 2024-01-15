// fm_serial.ino
//
// FM radio with controls from the Serial interface
//
// Created on Jan 10, 2024

#include <Wire.h>
#include "Si470x.h"

// Si470x
// SDIO --> A4 (I2C)
// SCLK --> A5 (I2C)
#define PIN_RESET D2
#define PIN_SDIO A4
#define PIN_SCLK A5
#define PIN_GPIO2 A6
#define PIN_EXTERN_MUTE D3
Si470x radio(PIN_RESET, PIN_SDIO, PIN_SCLK, /* enableInterrupts = */ true);

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


int volume = 5;
int channel = _channels[_channelIndex];
bool mono = false;
bool enableRDS = true;
bool streamRDS = false;

unsigned long _statusMillis = 0;
const long _statusInterval = 1000;
int _rssi;

const char *_radioText;
// u8g2_uint_t _radioTextWidth;
unsigned long _radioTextMillis = 0;
const long _radioTextInterval = 5000;

volatile bool _rdsReady = false;

void interruptHandler() {
    _rdsReady = true;
}

void setup() {
    pinMode(PIN_EXTERN_MUTE, OUTPUT);
    digitalWrite(PIN_EXTERN_MUTE, HIGH);

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
    Serial.println("==================================================");  

    radio.setVolume(volume);
    radio.setMono(mono);
    radio.setChannel(channel);

    pinMode(PIN_GPIO2, INPUT_PULLUP);
    attachInterrupt(PIN_GPIO2, interruptHandler, FALLING);
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
    Serial.print(radio.getProgramID(), HEX);

    Serial.print(" | PS: ");
    Serial.print(radio.getStationName());

    Serial.print(" | RT: ");
    Serial.print(radio.getRadioText());

    Serial.println();
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

    // poll radio text
    if (currentMillis - _radioTextMillis >= _radioTextInterval) {
        _radioText = radio.getRadioText();
        // _radioTextWidth = u8g2.getUTF8Width(_radioText);
        _radioTextMillis = currentMillis;
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
            printStatus();
        } else if (ch == '-') {
            volume--;
            if (volume < 0) volume = 0;
            radio.setVolume(volume);
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
    if (enableRDS && _rdsReady) {
        _rdsReady = false;
        radio.checkRDS();
    }
}
