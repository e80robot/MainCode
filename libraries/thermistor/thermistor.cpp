#include "ButtonSampler.h"
#include "Printer.h"

extern Printer printer;

AnalogSource::AnalogSource(String name, int pin) 
  : name, pin, DataSource(name,"float") // from DataSource
{}


void AnalogSource::init(void)
{ 
  pinMode(pin,INPUT); 
  // when the button is not pressed the voltage
  // at USER_BUTTON will be high
}


void AnalogSource::updateState(void)
// This function is called in the main loop of Default_Robot.ino
{
  pinVoltage = (3.3 / 1024.0) * analogRead(pin);
}


String AnalogSource::printState(void)
// This function returns a string that the Printer class 
// can print to the serial monitor if desired
{
  return "Analog Voltage at " + name + ": " + String(voltage);
}


size_t AnalogSource::writeDataBytes(unsigned char * buffer, size_t idx)
// This function writes data to the micro SD card
{
  bool * data_slot = (bool *) &buffer[idx];
  data_slot[0] = pinVoltage;
  return idx + sizeof(bool);
}
