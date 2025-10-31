#include <string>
#include <stdio.h>
#include "PiFracMotor.h"



int main()
{
  ConcreteSPI SPI;
  PiFracMotor::configureDefault(10,&SPI);

   return 0;
}

