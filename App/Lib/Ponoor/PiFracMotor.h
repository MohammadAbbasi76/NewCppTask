#ifndef PiFracMotor_h
#define PiFracMotor_h
#include "Ponoor_L6470Library.h"
#include "Abstract/prephral_config.hpp"
/// <summary>
/// Class used for controlling the motor driver chips on the PiFrac controller board.
/// </summary>
class PiFracMotor : AutoDriver
{
  public:
    // Constructor
    PiFracMotor(int csPin);
    
    // Destructor
    ~PiFracMotor();
    int csPin; // chip select pin
    static bool isBusy(int csPin); // check to see if a particular motor is busy or not
    static void moveMotor(int csPin, uint8_t dir, uint32_t numSteps); // move a motor
    static void configureDefault(int csPin, ConcreteSPI *SPI); // configure the motor driver
    static void configureSpeed(int csPin, float maxSpeed, float fullSpeed, ConcreteSPI *SPI); // configure maximum and full (steady state) speed of the motor
    static void configureAcc(int csPin, float acc, float dec, ConcreteSPI *SPI); // configure the acceleration and decceleration of the motor
    static void configureStepMode(int csPin, int stepMode); // configure the microstepping setting of the motor

private:
    void setCSPin(bool csPinHigh); // set CS pin high or low
    static void Initialize(int csPin); // initialize a motor driver using a particular chip select pin from the host microcontroller
};
#endif // PiFracMotor_h
