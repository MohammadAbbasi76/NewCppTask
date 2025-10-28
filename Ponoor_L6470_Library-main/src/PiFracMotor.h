#ifndef PiFracMotor_h
#define PiFracMotor_h
#include "Ponoor_L6470Library.h"

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
    static void moveMotor(int csPin, byte dir, unsigned long numSteps); // move a motor
    static void configureDefault(int csPin); // configure the motor driver
    static void configureSpeed(int csPin, float maxSpeed, float fullSpeed); // configure maximum and full (steady state) speed of the motor
    static void configureAcc(int csPin, float acc, float dec); // configure the acceleration and decceleration of the motor
    static void configureStepMode(int csPin, int stepMode); // configure the microstepping setting of the motor

private:
    void setCSPin(bool csPinHigh); // set CS pin high or low
    static void Initialize(int csPin); // initialize a motor driver using a particular chip select pin from the host microcontroller

};
#endif // PiFracMotor_h
