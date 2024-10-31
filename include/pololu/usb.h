/**
 * USB device manager
 */
 
#ifndef __POLOLU_USB_DEVICE_H
#define __POLOLU_USB_DEVICE_H

#include <libusb-1.0/libusb.h>

#include <stdio.h>
#include <string>
#include <cstring>


/**
 * USB device manager
 */
class UsbDevice
{
public:
	/**
	 * constructor
	 */
	UsbDevice();

	/**
	 * destructor
	 */
	~UsbDevice();

	/**
	 * Open
	 */
	bool Open( uint16_t vendorID, uint16_t productID, const char* serial=NULL );

	/**
	 * Close
	 */
	void Close();

  /**
   * Find
   */
  libusb_device* Find( uint16_t vendorID, uint16_t productID, const char* serial=NULL );
  
	/**
	 * Return device serial string.
	 */
	inline const char* GetSerial() const								{ return mSerial.c_str(); }
	
	/**
	 * Return device vendor name.
	 */
	inline const char* GetVendor() const								{ return mVendor.c_str(); }

  /**
	 * Return device product name.
	 */
	inline const char* GetProduct() const								{ return mProduct.c_str(); }

	/**
	 * Control transfer
	 */
	bool ControlTransfer( uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, void* data, uint32_t size );
	
	/**
	 * Control transfer
	 */
	bool ControlTransfer( uint8_t requestType, uint8_t request, uint16_t  value, uint16_t index );
	
	/**
	 * Data transfer
	 */
	bool DataTransfer( uint8_t requestType, uint8_t request, void* data, uint32_t size );
	
protected:

	libusb_device* mDevice;
	libusb_device_handle* mDeviceCtx;
  libusb_context* mUsbContext;
  
	std::string mSerial;
	std::string mVendor;
	std::string mProduct;
	
	uint16_t mVendorID;
	uint16_t mProductID;
};


#endif

