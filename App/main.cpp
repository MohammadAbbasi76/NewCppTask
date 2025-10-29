// #include <string>
// #include <stdio.h>
// #include "test.h"


// int main()
// {
//     testClass print;
//     print.test();
//    return 0;
// }



#include <Arduino.h>
#include <pwm.h>
#include <Wire.h>
#include <SPI.h>
#include <DAC.h>
#include "PiFracMotor.h"

class PinManager {
private:
    struct SpecialPinConfig {
        unsigned long baseAddress;
        int baseIndex;
    };

    static const SpecialPinConfig getPinConfig(int pinIndex);

public:
    static void setupSpecialPin(int pinIndex, PinMode pm, PinStatus ps);
};

const PinManager::SpecialPinConfig PinManager::getPinConfig(int pinIndex) {
    if (pinIndex >= 10 && pinIndex <= 15) {
        return {0x40040828, 10};
    } else if (pinIndex >= 100 && pinIndex <= 115) {
        return {0x40040840, 100};
    } else if (pinIndex >= 200 && pinIndex <= 206) {
        return {0x40040880, 200};
    } else if (pinIndex >= 212 && pinIndex <= 215) {
        return {0x400408B0, 215};
    } else if (pinIndex >= 300 && pinIndex <= 307) {
        return {0x400408C0, 300};
    } else if (pinIndex >= 400 && pinIndex <= 415) {
        return {0x40040900, 400};
    }
    return {0, 0}; // Invalid pin
}

void PinManager::setupSpecialPin(int pinIndex, PinMode pm, PinStatus ps) {
    auto config = getPinConfig(pinIndex);
    if (config.baseAddress == 0) return; // Invalid pin
    
    volatile unsigned char *address = 
        (volatile unsigned char *)(config.baseAddress + 3 + (pinIndex - config.baseIndex) * 4);
    
    if (pm == OUTPUT) {
        *address = (ps == LOW) ? 0x04 : 0x05;
    } else if (pm == INPUT) {
        *address &= 0xFB; // Configure pin as input
    }
}

class FrontPanel {
private:
    static const int LED_PIN = A2;
    static const int SWITCH_PIN = A1;

public:
    static void initialize();
    static void setLED(bool on);
    static bool isSwitchPressed();
};

void FrontPanel::initialize() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(SWITCH_PIN, INPUT);
}

void FrontPanel::setLED(bool on) {
    digitalWrite(LED_PIN, on ? HIGH : LOW);
}

bool FrontPanel::isSwitchPressed() {
    return digitalRead(SWITCH_PIN) == LOW;
}

class LampController {
private:
    static const int LAMP_PIN = 9;
    PwmOut pwmLamp;

public:
    LampController();
    void initialize();
    void setPeriodDutyCycle(uint32_t period, uint32_t duty_cycle);
};

LampController::LampController() : pwmLamp(LAMP_PIN) {}

void LampController::initialize() {
    pinMode(LAMP_PIN, OUTPUT);
    setPeriodDutyCycle(1000000, 500000); // Default: 1s period, 50% duty
}

void LampController::setPeriodDutyCycle(uint32_t period, uint32_t duty_cycle) {
    pwmLamp.begin(period, duty_cycle);
}

class AirPumpController {
private:
    static const int PRESSURE_DAC_PIN = A0;
    static const int ENABLE_PIN = 015;

public:
    void initialize();
    void setPressure(uint16_t pressure);
    void enable(bool enable);
};

void AirPumpController::initialize() {
    analogWriteResolution(12);
    PinManager::setupSpecialPin(ENABLE_PIN, OUTPUT, LOW);
}

void AirPumpController::setPressure(uint16_t pressure) {
    if (pressure > 4095) return; // 12-bit max
    analogWrite(PRESSURE_DAC_PIN, pressure);
}

void AirPumpController::enable(bool enable) {
    PinManager::setupSpecialPin(ENABLE_PIN, OUTPUT, enable ? HIGH : LOW);
}

class CollectorController {
private:
    static const int GUTTER_PIN = 012;
    static const int ADVANCE_PIN = 013;
    static const int GUTTER_PULSE_TIME_MS = 260;
    static const int ADVANCE_PULSE_TIME_MS = 260;

public:
    void initialize();
    void returnToGutter();
    void advance();
};

void CollectorController::initialize() {
    PinManager::setupSpecialPin(GUTTER_PIN, OUTPUT, LOW);
    PinManager::setupSpecialPin(ADVANCE_PIN, OUTPUT, LOW);
}

void CollectorController::returnToGutter() {
    PinManager::setupSpecialPin(GUTTER_PIN, OUTPUT, HIGH);
    delay(GUTTER_PULSE_TIME_MS);
    PinManager::setupSpecialPin(GUTTER_PIN, OUTPUT, LOW);
}

void CollectorController::advance() {
    PinManager::setupSpecialPin(ADVANCE_PIN, OUTPUT, HIGH);
    delay(ADVANCE_PULSE_TIME_MS);
    PinManager::setupSpecialPin(ADVANCE_PIN, OUTPUT, LOW);
}

class SolenoidController {
private:
    static const int ENABLE_PIN = 4;

public:
    void initialize();
    void enable();
    void disable();
};

void SolenoidController::initialize() {
    pinMode(ENABLE_PIN, OUTPUT);
    disable();
}

void SolenoidController::enable() {
    digitalWrite(ENABLE_PIN, HIGH);
}

void SolenoidController::disable() {
    digitalWrite(ENABLE_PIN, LOW);
}

class MotorController {
private:
    static const int STANDBY_PIN = 5;
    static const int CHIP_SELECT_PINS[6];
    static const int BUSY_PIN = A5;
    static const int FLAG_PIN = A4;

    int stepDirection = 1;

public:
    void initialize();
    void toggleMotors();
    bool isBusy(int motorIndex);
    bool hasFault(int motorIndex);
};

const int MotorController::CHIP_SELECT_PINS[6] = {10, 2, 7, 6, 3, 4}; // nCS1 to nCS6

void MotorController::initialize() {
    // Setup standby pin
    pinMode(STANDBY_PIN, OUTPUT);
    digitalWrite(STANDBY_PIN, LOW);
    delayMicroseconds(50);
    
    // Setup chip select pins
    for (int i = 0; i < 6; i++) {
        pinMode(CHIP_SELECT_PINS[i], OUTPUT);
        digitalWrite(CHIP_SELECT_PINS[i], HIGH);
    }
    
    // Setup status pins
    pinMode(BUSY_PIN, INPUT);
    pinMode(FLAG_PIN, INPUT);
    
    // Initialize SPI
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    delay(100);
    
    digitalWrite(STANDBY_PIN, HIGH);
    delayMicroseconds(50);
    
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    
    // Configure motor drivers
    for (int i = 0; i < 5; i++) { // Only first 5 motors
        PiFracMotor::configureDefault(CHIP_SELECT_PINS[i]);
    }
}

void MotorController::toggleMotors() {
    if (stepDirection == 1) {
        PiFracMotor::moveMotor(CHIP_SELECT_PINS[0], FWD, 10000);
        PiFracMotor::moveMotor(CHIP_SELECT_PINS[1], FWD, 10000);
    } else {
        PiFracMotor::moveMotor(CHIP_SELECT_PINS[0], REV, 10000);
        PiFracMotor::moveMotor(CHIP_SELECT_PINS[1], REV, 10000);
    }
    
    // Wait for motors to complete movement
    while (PiFracMotor::isBusy(CHIP_SELECT_PINS[0]) || 
           PiFracMotor::isBusy(CHIP_SELECT_PINS[1])) {
        delay(100);
    }
    
    delay(1000); // Additional delay
    stepDirection *= -1;
}

bool MotorController::isBusy(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 6) return false;
    return PiFracMotor::isBusy(CHIP_SELECT_PINS[motorIndex]);
}

bool MotorController::hasFault(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 6) return false;
    return PiFracMotor::hasFault(CHIP_SELECT_PINS[motorIndex]);
}

class SystemController {
private:
    LampController lamp;
    AirPumpController airPump;
    CollectorController collector;
    SolenoidController solenoid;
    MotorController motors;
    bool systemActive = false;

public:
    void initialize();
    void runMainLoop();
    void shutdown();
};

void SystemController::initialize() {
    FrontPanel::initialize();
    lamp.initialize();
    airPump.initialize();
    collector.initialize();
    solenoid.initialize();
    motors.initialize();
    
    FrontPanel::setLED(true);
    systemActive = true;
}

void SystemController::runMainLoop() {
    if (!systemActive) return;
    
    motors.toggleMotors();
    
    // Add additional system logic here
    if (FrontPanel::isSwitchPressed()) {
        // Handle front panel switch press
        delay(50); // Debounce
    }
}

void SystemController::shutdown() {
    systemActive = false;
    FrontPanel::setLED(false);
    airPump.enable(false);
    solenoid.disable();
    // Add other shutdown procedures
}

// Global system instance
SystemController systemController;

void setup() {
    systemController.initialize();
}

void loop() {
    systemController.runMainLoop();
}