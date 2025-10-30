#include <Arduino.h>
#include <pwm.h>
#include <Wire.h>
#include <SPI.h>
#include <DAC.h>
#include "PiFracMotor.h"

// standard Arduino Uno pin assignments
#define pwmLAMP       9    // digital pin used for lamp output
#define AIR_DAC      A0    // 12-bit DAC controls air pump pressure
#define fpSWT        A1    // front panel SPST momentary "0", system ON/STANDBY
#define onFPLED      A2    // pin used for front panel LED
#define nBUSY        A5    // motor controller BUSY="0"
#define nFLAG        A4    // motor issue if "0"
#define nSTBY         5    // put motor in standby (reset)="0"
#define nCS6          4    // chip select AUX
#define nCS5          3    // chip select Motor5
#define nCS4          2    // chip select Motor4
#define nCS3          6    // chip select Motor3
#define nCS2          7    // chip select Motor2
#define nCS1         10    // chip select Motor1 
#define ENSOLENOID    4    // "1" to ON solenoid (12V output 1.5A max) 

// special pin assignments
#define GUTTER 012  // pulse "1" output to signal collector return to GUTTER
#define ADVANCE 013 // pusle "1" output to signal collector to ADVANCE
#define AirEN 015   // air pump ON/OFF, ON="1"

// pulse timings
#define GUTTER_PULSE_TIME_MS 260    // pulse time (in ms) for sending collector to gutter
#define ADVANCE_PULSE_TIME_MS 260   // pulse time (in ms) for advancing collector


PwmOut pwmLamp(pwmLAMP);

int stepDir = 1;

// turn the front panel LED on or off
void frontPanelLED(bool on)
{
    if (on)
    {
       digitalWrite(onFPLED, HIGH);
    }
    else
    {
       digitalWrite(onFPLED, LOW);
    }
}

void specialPinMode(int pinIndex, PinMode pm,  PinStatus ps)
{
   unsigned int32_t baseAddress = 0x40040800;
   int baseIndex = 0;
   if (pinIndex >= 10 && pinIndex <= 15)
   {
     baseIndex = 10;
     baseAddress = 0x40040828;
   }
   else if (pinIndex >= 100 && pinIndex <=115)
   {
     baseIndex = 100;
     baseAddress = 0x40040840;
   }
   else if (pinIndex >= 200 && pinIndex <= 206)
   {
     baseIndex = 200;
     baseAddress = 0x40040880;
   }
   else if (pinIndex >= 212 && pinIndex <= 215)
   {
     baseIndex = 215;
     baseAddress = 0x400408B0;
   }
   else if (pinIndex >= 300 && pinIndex <= 307)
   {
     baseIndex = 300;
     baseAddress = 0x400408C0;
   }
   else if (pinIndex >= 400 && pinIndex <= 415)
   {
     baseIndex = 400;
     baseAddress = 0x40040900;
   }
   volatile unsigned char *address = (volatile unsigned char *)(baseAddress + 3 + (pinIndex - baseIndex)*4);
   if (pm == OUTPUT)
   {
      if (ps == LOW)
      {
         *address = 0x04;
      }
      else 
      {
        // high output
        *address = 0x05;
      }
   }
   else if (pm == INPUT)
   {
      *address &= 0xFB; // configure pin as input
   }
}

// pulse “1” output to return signal collector to GUTTER
void returnToGutter() 
{
   specialPinMode(GUTTER, PinMode::OUTPUT, HIGH);
   delay(GUTTER_PULSE_TIME_MS);
   specialPinMode(GUTTER, PinMode::OUTPUT, LOW);
}

// pulse "1" output to advance signal collector 
void advanceCollector()
{
   specialPinMode(ADVANCE, PinMode::OUTPUT, HIGH);
   delay(ADVANCE_PULSE_TIME_MS);
   specialPinMode(ADVANCE, PinMode::OUTPUT, LOW);
}

/// @brief set the air pump pressure
/// @param airPumpPressureVal - pressure value between 0 and 4095 (12-bit value controls D to A output)
void setAirPumpPressure(uint16_t airPumpPressureVal)
{
    if (airPumpPressureVal > 4095)
    {
       // air pump pressure value is too high
       return;
    }
    analogWrite(A0, (int)airPumpPressureVal);
}

/// @brief enable / disable the air pump
/// @param enable - set to true to enable the air pump, false to disable.
void enableAirPump(bool enable)
{
   if (enable)
   {
       // enable air pump
       specialPinMode(AirEN, PinMode::OUTPUT, HIGH);
   }
   else
   {
       // disable air pump
       specialPinMode(AirEN, PinMode::OUTPUT, LOW);
   }
}

/// @brief set lamp period and duty cycle (both in us)
/// @param period Period of pulse width modulation (in us)
/// @param duty_cycle Duty cycle of pulse width modulation (in us).
void setLampPeriodDutyCycle(uint32_t period, uint32_t duty_cycle)
{
    pwmLamp.begin(period, duty_cycle);
}

/// @brief enable / disable the solenoid
/// @param enable - set to true to enable the solenoid, false to disable.
void enableSolenoid(bool enable)
{
   if (enable)
   {
       digitalWrite(ENSOLENOID, HIGH);       
   }
   else{
       digitalWrite(ENSOLENOID, LOW);
   }
}


void setup() {
  // put your setup code here, to run once:
  // set lamp to output
  pinMode(pwmLAMP, OUTPUT);
  setLampPeriodDutyCycle(1000000, 500000); // set lamp to 1 s period with 50% duty cycle
  // set FPSWT pin as input
  pinMode(fpSWT, INPUT);
  // set ONFPLED as output
  pinMode(onFPLED, OUTPUT);
  // set gutter as output
  specialPinMode(GUTTER, OUTPUT, LOW);
  // set advance as output
  specialPinMode(ADVANCE, OUTPUT, LOW);
  // set the DAC to 12-bit resolution
  analogWriteResolution(12);
  // set air pump off
  specialPinMode(AirEN, OUTPUT, LOW);
  // put motor (when selected) in standby (reset)
  pinMode(nSTBY, OUTPUT);
  digitalWrite(nSTBY, LOW);
  delayMicroseconds(50);
  // set all chip selects to outputs (high / not selected)
  pinMode(nCS6, OUTPUT);
  digitalWrite(nCS6, HIGH);
  pinMode(nCS5, OUTPUT);
  digitalWrite(nCS5, HIGH);
  pinMode(nCS4, OUTPUT);
  digitalWrite(nCS4, HIGH);
  pinMode(nCS3, OUTPUT);
  digitalWrite(nCS3, HIGH);
  pinMode(nCS2, OUTPUT);
  digitalWrite(nCS2, HIGH);
  pinMode(nCS1, OUTPUT);
  digitalWrite(nCS1, HIGH);
 
  pinMode(ENSOLENOID, OUTPUT);
  digitalWrite(ENSOLENOID, LOW);

  // setup default SPI configuration
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  delay(100);
  digitalWrite(nSTBY, HIGH);
  delayMicroseconds(50);
  SPI.begin();
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

  // set all motor drivers for an example default configuration
  PiFracMotor::configureDefault(nCS1);
  PiFracMotor::configureDefault(nCS2);
  PiFracMotor::configureDefault(nCS3);
  PiFracMotor::configureDefault(nCS4);
  PiFracMotor::configureDefault(nCS5);

  // setup analog output for 12-bits
  analogWriteResolution(12);
  
  // turn front panel LED on
  frontPanelLED(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  // move the 1st and 2nd motors back and forth (use nCS2 for 2nd motor, nCS3 for third motor, etc.)
  if (stepDir == 1) 
  {
    PiFracMotor::moveMotor(nCS1, FWD, 10000);
    PiFracMotor::moveMotor(nCS2, FWD, 10000);
  }
  else 
  {
    PiFracMotor::moveMotor(nCS1, REV, 10000);
    PiFracMotor::moveMotor(nCS2, REV, 10000);
  }
  
  while (PiFracMotor::isBusy(nCS1) || PiFracMotor::isBusy(nCS2))
  {
    delay(100);
  } 
  delay(1000); // additional 1 s delay of no motor movement
  stepDir *= -1;
}
