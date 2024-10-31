/**
 * Pololu Maestro servo controller
 */
 
#ifndef __POLOLU_MAESTRO_H
#define __POLOLU_MAESTRO_H

#include "pololu/usb.h"


/**
 * MaestroDevice
 */
class MaestroDevice : public UsbDevice
{
public:
	/**
	 * Open device (by default any Maestro will be used)
	 */
	static MaestroDevice* Open( uint16_t productID=0, const char* serial=NULL );

	/**
 	 * Set servo position
	 */
	bool SetPosition( uint8_t servo, uint16_t position );

	/**
	 * Reset errors
	 */
	bool ResetErrors();
	
	/**
	 * Usb Vendor Id
	 */
	static const uint16_t VendorID = 0x1ffb;		// Pololu Corporation

	/**
	 * Usb Product Id
	 */
	static const uint16_t ProductID_6ch  = 0x89;			// Maestro 6-channel
	static const uint16_t ProductID_12ch = 0x8a;			// Maestro 12-channel
	static const uint16_t ProductID_18ch = 0x8b;			// Maestro 18-channel


protected:
  MaestroDevice();
  
};


#endif

