#ifndef MCU_ABSTRACT_H
#define MCU_ABSTRACT_H

#include <cstdint>

// GPIO Abstract Class
class GPIO_Abstract {
public:
    virtual ~GPIO_Abstract() = default;
    
    // Pin mode configuration
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
    
    // Digital write/read
    virtual void digitalWrite(uint8_t pin, bool state) = 0;
    virtual bool digitalRead(uint8_t pin) = 0;
    
    // Optional: Toggle pin
    virtual void togglePin(uint8_t pin) {
        bool currentState = digitalRead(pin);
        digitalWrite(pin, !currentState);
    }
};

// PWM Abstract Class
class PWM_Abstract {
public:
    virtual ~PWM_Abstract() = default;
    
    // PWM initialization for specific pin
    virtual void pwmInit(uint8_t pin, uint32_t frequency = 1000) = 0;
    
    // Set PWM duty cycle (0-255, 0-100%, or custom range)
    virtual void setPWM(uint8_t pin, uint8_t value) = 0;        // 0-255
    virtual void setPWMPercent(uint8_t pin, float percent) = 0; // 0-100%
    virtual void setPWMRaw(uint8_t pin, uint16_t value) = 0;    // Raw value
    
    // PWM configuration
    virtual void setPWMFrequency(uint8_t pin, uint32_t frequency) = 0;
    virtual void setPWMResolution(uint8_t pin, uint8_t resolution) = 0;
};

// SPI Abstract Class
class SPI_Abstract {
public:
    virtual ~SPI_Abstract() = default;
    
    // SPI initialization
    virtual void begin() = 0;
    virtual void end() = 0;
    
    // SPI configuration
    virtual void setBitOrder(uint8_t bitOrder) = 0;
    virtual void setDataMode(uint8_t dataMode) = 0;
    virtual void setClockDivider(uint32_t clockDiv) = 0;
    
    // Data transfer
    virtual uint8_t transfer(uint8_t data) = 0;
    virtual void transfer(uint8_t* txData, uint8_t* rxData, size_t length) = 0;
    // Chip select management (optional)
    virtual void setChipSelect(uint8_t pin) {
        // Default implementation - can be overridden
        digitalWrite(pin, LOW);
    }
    
    virtual void releaseChipSelect(uint8_t pin) {
        // Default implementation - can be overridden
        digitalWrite(pin, HIGH);
    }
};

// Main MCU Abstract Class
class MCU_Abstract {
public:
    virtual ~MCU_Abstract() = default;
    
    // Get instances of each module
    virtual GPIO_Abstract* getGPIO() = 0;
    virtual PWM_Abstract* getPWM() = 0;
    virtual SPI_Abstract* getSPI() = 0;
    
    // Utility functions
    virtual void delay(uint32_t milliseconds) = 0;
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
};

#endif // MCU_ABSTRACT_H

























#ifdef IMPLEMENATTION
#include "MCU_Abstract.h"

// User must implement these concrete classes for their specific MCU

class ConcreteGPIO : public GPIO_Abstract {
public:
    void pinMode(uint8_t pin, uint8_t mode) override {
        // User implements MCU-specific pin mode configuration
        // Example for Arduino:
        // ::pinMode(pin, mode);
    }
    
    void digitalWrite(uint8_t pin, bool state) override {
        // User implements MCU-specific digital write
        // Example for Arduino:
        // ::digitalWrite(pin, state);
    }
    
    bool digitalRead(uint8_t pin) override {
        // User implements MCU-specific digital read
        // Example for Arduino:
        // return ::digitalRead(pin);
        return false; // placeholder
    }
};

class ConcretePWM : public PWM_Abstract {
public:
    void pwmInit(uint8_t pin, uint32_t frequency = 1000) override {
        // User implements MCU-specific PWM initialization
        // Example for Arduino:
        // ::pinMode(pin, OUTPUT);
        // ::analogWriteFrequency(pin, frequency); // Some Arduino variants
    }
    
    void setPWM(uint8_t pin, uint8_t value) override {
        // User implements MCU-specific PWM set (0-255)
        // Example for Arduino:
        // ::analogWrite(pin, value);
    }
    
    void setPWMPercent(uint8_t pin, float percent) override {
        // Convert percentage to 0-255 range
        uint8_t value = static_cast<uint8_t>((percent / 100.0f) * 255);
        setPWM(pin, value);
    }
    
    void setPWMRaw(uint8_t pin, uint16_t value) override {
        // User implements MCU-specific raw PWM set
        // This might be platform-specific
        setPWM(pin, static_cast<uint8_t>(value & 0xFF));
    }
    
    void setPWMFrequency(uint8_t pin, uint32_t frequency) override {
        // User implements MCU-specific frequency setting
    }
    
    void setPWMResolution(uint8_t pin, uint8_t resolution) override {
        // User implements MCU-specific resolution setting
    }
};

class ConcreteSPI : public SPI_Abstract {
private:
    uint8_t csPin;
    
public:
    void begin() override {
        // User implements MCU-specific SPI begin
        // Example for Arduino:
        // ::SPI.begin();
    }
    
    void end() override {
        // User implements MCU-specific SPI end
        // Example for Arduino:
        // ::SPI.end();
    }
    
    void setBitOrder(uint8_t bitOrder) override {
        // User implements MCU-specific bit order setting
        // Example for Arduino:
        // ::SPI.setBitOrder(bitOrder);
    }
    
    void setDataMode(uint8_t dataMode) override {
        // User implements MCU-specific data mode setting
        // Example for Arduino:
        // ::SPI.setDataMode(dataMode);
    }
    
    void setClockDivider(uint32_t clockDiv) override {
        // User implements MCU-specific clock divider setting
        // Example for Arduino:
        // ::SPI.setClockDivider(clockDiv);
    }
    
    uint8_t transfer(uint8_t data) override {
        // User implements MCU-specific SPI transfer
        // Example for Arduino:
        // return ::SPI.transfer(data);
        return 0; // placeholder
    }
    
    void transfer(uint8_t* txData, uint8_t* rxData, size_t length) override {
        // User implements MCU-specific bulk SPI transfer
        // Example for Arduino:
        // for(size_t i = 0; i < length; i++) {
        //     rxData[i] = ::SPI.transfer(txData[i]);
        // }
    }
    
    void setChipSelect(uint8_t pin) override {
        csPin = pin;
        // User implements chip select activation
        // Example for Arduino:
        // ::digitalWrite(pin, LOW);
    }
    
    void releaseChipSelect(uint8_t pin) override {
        // User implements chip select deactivation
        // Example for Arduino:
        // ::digitalWrite(pin, HIGH);
    }
};

class ConcreteMCU : public MCU_Abstract {
private:
    ConcreteGPIO gpio;
    ConcretePWM pwm;
    ConcreteSPI spi;
    
public:
    GPIO_Abstract* getGPIO() override { return &gpio; }
    PWM_Abstract* getPWM() override { return &pwm; }
    SPI_Abstract* getSPI() override { return &spi; }
    
    void delay(uint32_t milliseconds) override {
        // User implements MCU-specific delay
        // Example for Arduino:
        // ::delay(milliseconds);
    }
    
    uint32_t millis() override {
        // User implements MCU-specific millis
        // Example for Arduino:
        // return ::millis();
        return 0; // placeholder
    }
    
    uint32_t micros() override {
        // User implements MCU-specific micros
        // Example for Arduino:
        // return ::micros();
        return 0; // placeholder
    }
};


#endif