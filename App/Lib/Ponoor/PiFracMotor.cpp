#include "PiFracMotor.h"
#include "Abstract/prephral_config.hpp"
#define nBUSY        A5    // motor controller BUSY="0"
#define nFLAG        A4    // motor issue if "0"
#define nSTBY         5    // put motor in standby (reset)="0"

static PiFracMotor* motorDriver = NULL; // motor driver 

/// <summary>
/// Constructor
/// </summary>
/// <param name="csPin">chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param>
PiFracMotor::PiFracMotor(int csPin) : AutoDriver(0, csPin, nSTBY, nBUSY)
{
	this->csPin = csPin;
	// make sure csPin is low for at least 1 us
	setCSPin(false);
	motorDriver = this;
}

PiFracMotor::~PiFracMotor()
{
	// we are done communicating with this particular motor, so put its chip select high
	setCSPin(true);
}

/// <summary>
/// Check to see if a particular motor is busy or not
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param>
/// <returns>true if the motor driver chip is already executing a particular move.</returns>
bool PiFracMotor::isBusy(int csPin)
{
	Initialize(csPin);
	bool retVal = (bool)motorDriver->busyCheck();
	return retVal;
}


/// <summary>
/// Move a motor forward or backwards a certain number of steps.
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param>
/// <param name="dir"></param>
/// <param name="numSteps"></param>
void PiFracMotor::moveMotor(int csPin, uint8_t dir, unsigned int32_t numSteps)
{
	Initialize(csPin);
	motorDriver->move(dir, numSteps);
}

/// <summary>
/// Initialize a particular motor driver chip
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param> 
void PiFracMotor::Initialize(int csPin)
{
	if (motorDriver)
	{
		if (motorDriver->csPin != csPin)
		{
			delete motorDriver;
			new PiFracMotor(csPin);
		}
	}
	else
	{
		new PiFracMotor(csPin);
	}
}

/// <summary>
/// Configure the maximum and full (steady state) speed of the motor in steps per second.
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param> 
/// <param name="maxSpeed">The maximum speed of the motor in steps per second.</param>
/// <param name="fullSpeed">The full (steady state) speed of the motor in steps per second.</param>
void PiFracMotor::configureSpeed(int csPin, float maxSpeed, float fullSpeed)
{
	Initialize(csPin);
	motorDriver->SPIPortConnect(&SPI);      // connect the SPI to the motor driver
	motorDriver->setMaxSpeed(maxSpeed);        // 10000 steps/s max
	motorDriver->setFullSpeed(fullSpeed);       // microstep below 10000 steps/s
}

/// <summary>
/// Configure the acceleration and deceleration of the motor.
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param> 
/// <param name="acc">Acceleration (in steps per second per second).</param>
/// <param name="dec">Deceleration (int steps per second per second).</param>
void PiFracMotor::configureAcc(int csPin, float acc, float dec)
{
	Initialize(csPin);
	motorDriver->SPIPortConnect(&SPI);      // connect the SPI to the motor driver
	motorDriver->setAcc(acc);
	motorDriver->setDec(dec);
}

/// <summary>
/// Configure the microstepping setting of the motor
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param> 
/// <param name="stepMode">number between STEP_FS and STEP_FS_128 that corresponds to the 
/// microstepping setting of the motor.</param>
void PiFracMotor::configureStepMode(int csPin, int stepMode)
{
	if (stepMode < STEP_FS || stepMode > STEP_FS_128)
	{
		// invalid step mode
		return;
	}
	Initialize(csPin);
	motorDriver->configStepMode(stepMode);
}

/// <summary>
/// configure the motor driver with an example default cofiguration.
/// TODO: add additional code for custom motor configurations 
/// </summary>
/// <param name="csPin">Chip select pin. Pin on the micro that is used to select the 
/// motor controller chip. Set to low in order to enable the motor controller chip.</param> 
void PiFracMotor::configureDefault(int csPin)
{
	Initialize(csPin);
	motorDriver->SPIPortConnect(&SPI);      // connect the SPI to the motor driver
	motorDriver->configSyncPin(BUSY_PIN, 0);// BUSY pin low during operations;
	motorDriver->configStepMode(STEP_FS);   // 0 microsteps per step
	motorDriver->setMaxSpeed(1000);        // 1000 steps/s max
	motorDriver->setFullSpeed(1000);       
	motorDriver->setAcc(1000);             // accelerate at 1000 steps/s/s
	motorDriver->setDec(1000);
	motorDriver->setSlewRate(SR_530V_us);   // Upping the edge speed increases torque.
	motorDriver->setOCThreshold(OC_3000mA);  // OC threshold 750mA
	motorDriver->setPWMFreq(PWM_DIV_2, PWM_MUL_2); // 31.25kHz PWM freq
	motorDriver->setOCShutdown(OC_SD_DISABLE); // don't shutdown on OC
	motorDriver->setVoltageComp(VS_COMP_DISABLE); // don't compensate for motor V
	motorDriver->setSwitchMode(SW_USER);    // Switch is not hard stop
	motorDriver->setOscMode(CONFIG_INT_16MHZ);
	motorDriver->setAccKVAL(128);           // We'll tinker with these later, if needed.
	motorDriver->setDecKVAL(128);
	motorDriver->setRunKVAL(128);
	motorDriver->setHoldKVAL(32);           // This controls the holding current; keep it low.
}

/// <summary>
/// Set CS pin high or low
/// </summary>
/// <param name="csPinHigh">set to true to set CS pin high, or false to set it low.</param>
void PiFracMotor::setCSPin(bool csPinHigh)
{
	if (csPinHigh)
	{
		digitalWrite(this->csPin, HIGH);
	}
	else
	{
		digitalWrite(this->csPin, LOW);
	}
	// make sure pin is at low or high state for at least 1 us before doing anything
	delayMicroseconds(1);
}