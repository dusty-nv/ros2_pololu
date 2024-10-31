/**
 * Pololu Simple Motor Controller
 */

#include "pololu/smc.h"


// constructor
SmcDevice::SmcDevice() : UsbDevice()
{
	memset(&mVariables, 0, sizeof(Variables));
}


// Open
SmcDevice* SmcDevice::Open( uint16_t productID, const char* serial )
{
  SmcDevice* dev = new SmcDevice();
  UsbDevice* usb = (UsbDevice*)dev;
  
  if( productID != 0 )
  {
    if( !usb->Open(VendorID, productID, serial) )
    {
      delete dev;
      return NULL;
    }
  }
  else
  {
    if( !usb->Open(VendorID, ProductID_18v15, serial) &&
        !usb->Open(VendorID, ProductID_24v12, serial) )
    {
      delete dev;
      return NULL;
    }
  }
  
  return dev;
}

// ExitSafeStart
bool SmcDevice::ResetErrors()
{
	if( !ControlTransfer(0x40, 0x91, 0, 0) )
		return false;

	return true;
}

// SetSpeed
bool SmcDevice::SetSpeed( SmcDevice::Direction dir, int speed )
{
	if( speed > 3200 )
		return false;

	//printf("set speed %i direction %i\n", speed, (int)dir);

	return ControlTransfer(0x40, 0x90, speed, dir);
}

// SetSpeed
bool SmcDevice::SetSpeed( int speed )
{
	//printf("set speed %i\n", speed);

	//if( speed < 3200 || speed > 3200 )
		//return false;

	if( speed < 0 )
		return SetSpeed(DIRECTION_REVERSE, -speed);
		
	return SetSpeed(DIRECTION_FORWARD, speed);
}

// ReadVariables
bool SmcDevice::ReadVariables( SmcDevice::Variables* var )
{
	if( !var )
		return false;

	if( !DataTransfer(0xC0, 0x83, var, sizeof(Variables)) )
		return false;

	return true;
}

// PrintVariables
void SmcDevice::PrintVariables()
{
	SmcDevice::Variables var;
	memset(&var, 0, sizeof(SmcDevice::Variables));

	if( !ReadVariables(&var) )
	{
		printf("[Pololu SMC] failed to read status of motor %s\n", mSerial.c_str());
		return;
	}

	printf("[Pololu SMC] status of motor %s\n", mSerial.c_str());
	
	// print variables
	printf("   errors:       ");

	if( var.errorStatus.safeStart )		printf("safe-start ");
	if( var.errorStatus.lowVIN )		printf("low-VIN ");
	if( var.errorStatus.overheat )		printf("overheat ");

	printf("\n");

	printf("   targetSpeed:  %i\n", var.targetSpeed);
	printf("   speed:        %i\n", var.speed);
	printf("   brake:        %u\n", var.brake);
	printf("   temperature:  %f degrees C\n", ((float)var.temperature * 0.1f));
	printf("   voltage:      %f V\n", ((float)var.voltage * 0.001f));
}


