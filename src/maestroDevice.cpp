/**
 * Pololu Maestro servo controller
 */

#include "pololu/maestro.h"


// constructor
MaestroDevice::MaestroDevice() : UsbDevice()
{

}


// Open
MaestroDevice* MaestroDevice::Open( uint16_t productID, const char* serial )
{
  MaestroDevice* dev = new MaestroDevice();
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
    if( !usb->Open(VendorID, ProductID_18ch, serial) &&
        !usb->Open(VendorID, ProductID_12ch, serial) &&
        !usb->Open(VendorID, ProductID_6ch, serial) )
    {
      delete dev;
      return NULL;
    }
  }
  
  return dev;
}


// SetSpeed
bool MaestroDevice::SetPosition( uint8_t servo, uint16_t position )
{
	//if( speed > 3200 )
		//return false;

	printf("[Maestro] set servo=%u position=%u\n", (uint32_t)servo, (uint32_t)position);

	// 0x85 REQUEST_SET_TARGET
	return ControlTransfer(0x40, 0x85, position * 4, servo);
}


// ResetErrors
bool MaestroDevice::ResetErrors()
{
	// 0x86 REQUEST_CLEAR_ERRORS
	if( !ControlTransfer(0x40, 0x86, 0, 0) )
		return false;

	return true;
}


