/**
 * USB device manager
 */

#include "pololu/usb.h"


// constructor
UsbDevice::UsbDevice()
{
  mDevice = NULL;
	mDeviceCtx = NULL;
	mUsbContext = NULL;
	
	mVendorID = 0;
	mProductID = 0;
}


// destructor
UsbDevice::~UsbDevice()
{
	Close();
}


// Open
bool UsbDevice::Open( uint16_t vendorID, uint16_t productID, const char* serial )
{
	mDevice = Find(vendorID, productID, serial);
	
	if( !mDevice )
	  return false;
	  
	if( libusb_open(mDevice, &mDeviceCtx) != 0 )
	{
		printf("[usb] failed to open USB device\n");
		return false;
	}

	libusb_device_descriptor desc;

	if( libusb_get_device_descriptor(mDevice, &desc) != 0 )
		return false;

	mVendorID = desc.idVendor;
	mProductID = desc.idProduct;

  // get serial number
	char buffer[512];
	memset(buffer, 0, sizeof(buffer));

	if( libusb_get_string_descriptor_ascii(mDeviceCtx, desc.iSerialNumber, (uint8_t*)buffer, sizeof(buffer)) > 0 )
		mSerial = buffer;

  if( libusb_get_string_descriptor_ascii(mDeviceCtx, desc.iManufacturer, (uint8_t*)buffer, sizeof(buffer)) > 0 )
		mVendor = buffer;
		
  if( libusb_get_string_descriptor_ascii(mDeviceCtx, desc.iProduct, (uint8_t*)buffer, sizeof(buffer)) > 0 )
		mProduct = buffer;
		
	printf("[usb] opened '%s %s' (vendor=%#04x product=%#04x serial=%s)\n", mVendor.c_str(), mProduct.c_str(), mVendorID, mProductID, mSerial.c_str());
	return true;
}


// Close
void UsbDevice::Close()
{
	if( mDeviceCtx != NULL )
	{
	  printf("[usb] closing device '%s %s' (vendor=%#04x product=%#04x serial=%s)\n", mVendor.c_str(), mProduct.c_str(), mVendorID, mProductID, mSerial.c_str());
		libusb_close(mDeviceCtx);
		mDeviceCtx = NULL;
	}
	
	if( mUsbContext != NULL )
	{
		libusb_exit(mUsbContext);
		mUsbContext = NULL;
	}
}


// Find
libusb_device* UsbDevice::Find( uint16_t vendorID, uint16_t productID, const char* serial )
{
	// open USB context
	if( !mUsbContext && libusb_init(&mUsbContext) != 0 )
	{
		printf("[usb] failed to create libusb context\n");
		return NULL;
	}
	
  // get devices
	libusb_device** devList;
	const uint32_t numDevices = libusb_get_device_list(mUsbContext, &devList);
	
	printf("[usb] %u devices\n\n", numDevices);

	for( uint32_t n=0; n < numDevices; n++ )
	{
		libusb_device_descriptor desc;

		if( libusb_get_device_descriptor(devList[n], &desc) != 0 )
			continue;

		printf("   [%02u]   vendor %#04x\t\tproduct %#04x\n", n, desc.idVendor, desc.idProduct);

    if( vendorID != 0 && vendorID != desc.idVendor )
      continue;
      
    if( productID != 0 && productID != desc.idProduct )
      continue;
    
    if( serial != NULL )
    {
        libusb_device_handle* deviceCtx;
        
        if( libusb_open(devList[n], &deviceCtx) != 0 )
	      {
		      printf("[usb] failed to open USB device for reading serial (vendor=%#04x product %#04x)\n", desc.idVendor, desc.idProduct);
		      continue;
	      }

        // get serial number
	      char buffer[512];
	      memset(buffer, 0, sizeof(buffer));

	      libusb_get_string_descriptor_ascii(deviceCtx, desc.iSerialNumber, (uint8_t*)buffer, sizeof(buffer));
	      libusb_close(deviceCtx);
	      
	      if( strcmp(serial, buffer) == 0 )
	        break;
	  }

    return devList[n];
  }
  
  printf("[usb] couldn't find device with vendor=%#04x product=%#04x serial=%s\n", vendorID, productID, serial);
  return NULL;
}


// ControlTransfer
bool UsbDevice::ControlTransfer( uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, void* data, uint32_t size )
{
	const int res = libusb_control_transfer(mDeviceCtx, requestType, request, value, index, (uint8_t*)data, size, 5000); 

	if( res != (int)size )
	{
		printf("[usb] usb control transfer failed (size=%i result=%i) (vendor='%s' product='%s' serial=%s\n", size, res, mVendor.c_str(), mProduct.c_str(), mSerial.c_str());
		return false;
	}
	
	return true;
}  

// ControlTransfer
bool UsbDevice::ControlTransfer( uint8_t requestType, uint8_t request, uint16_t value, uint16_t index )
{
	return ControlTransfer(requestType, request, value, index, 0, 0);
} 

// DataTransfer
bool UsbDevice::DataTransfer( uint8_t requestType, uint8_t request, void* data, uint32_t size )
{
	return ControlTransfer(requestType, request, 0, 0, data, size);
} 

