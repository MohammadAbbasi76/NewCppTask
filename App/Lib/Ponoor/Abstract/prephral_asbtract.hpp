#pragma once
#include <cstdint>

enum PinMode { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN };
enum PinStatus { LOW = 0, HIGH = 1 };

// GPIO Abstract Class
class GPIO_Abstract {
public:
    virtual ~GPIO_Abstract() = default;
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
    virtual void digitalWrite(uint8_t pin, bool state) = 0;
    virtual bool digitalRead(uint8_t pin) = 0;
    virtual void togglePin(uint8_t pin) {
        bool currentState = digitalRead(pin);
        digitalWrite(pin, !currentState);
    }
};



// SPI Abstract Class
class SPI_Abstract {
public:
    virtual ~SPI_Abstract() = default;
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setBitOrder(uint8_t bitOrder) = 0;
    virtual void setDataMode(uint8_t dataMode) = 0;
    virtual void setClockDivider(uint32_t clockDiv) = 0;
    virtual uint8_t transfer(uint8_t data) = 0;
    virtual void transfer(uint8_t* txData, uint8_t* rxData, uint32_t length) = 0;
    virtual void setChipSelect(uint8_t pin) = 0;
    virtual void releaseChipSelect(uint8_t pin) = 0;
};


// Timing Abstract Class
class Timing_Abstract {
public:
    virtual ~Timing_Abstract() = default;
    virtual void delay(uint32_t milliseconds) = 0;
    virtual void delayMicroseconds(uint32_t microseconds) = 0;
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
};


