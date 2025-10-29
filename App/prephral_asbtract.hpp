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