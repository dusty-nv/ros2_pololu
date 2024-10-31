/**
 * Pololu Jrk serial controller
 */
 
#ifndef __POLOLU_JRK_H
#define __POLOLU_JRK_H

#include <string>
#include <stdint.h>

#define JRK_INPUT_RELATIVE		 0
#define JRK_INPUT_ABSOLUTE		 1

#define JRK_READ_INPUT			 0xA1
#define JRK_READ_TARGET			 0xA3
#define JRK_READ_FEEDBACK		 0xA5
#define JRK_READ_FEEDBACK_SCALED 0xA7
#define JRK_READ_PID_ERROR		 0xA9
#define JRK_READ_DUTY_TARGET	 0xAB
#define JRK_READ_DUTY_CYCLE		 0xAD
#define JRK_READ_PID_PERIODS	 0xB1

#define JRK_ERROR_AWAITING_CMD  (1<<0)
#define JRK_ERROR_NO_POWER      (1<<1)
#define JRK_ERROR_MOTOR_DRIVER  (1<<2)
#define JRK_ERROR_PWM_INPUT     (1<<3)
#define JRK_ERROR_INPUT_DISCONNECTED	  (1<<4)
#define JRK_ERROR_FEEDBACK_DISCONNECTED	(1<<5)
#define JRK_ERROR_OVERCURRENT			(1<<6)
#define JRK_ERROR_SERIAL_SIGNAL   (1<<7)
#define JRK_ERROR_SERIAL_OVERRUN  (1<<8)
#define JRK_ERROR_SERIAL_RX_FULL  (1<<9)
#define JRK_ERROR_SERIAL_CRC      (1<<10)
#define JRK_ERROR_SERIAL_PROTOCOL (1<<11)
#define JRK_ERROR_SERIAL_TIMEOUT  (1<<12)
#define JRK_ERROR_COUNT					  13
#define JRK_ERROR_SYSTEM          UINT16_MAX


/**
 * Pololu Jrk serial controller
 */
class JrkDevice
{
public:
	/**
	 * Open device
	 */
	static JrkDevice* Open( const char* device="/dev/ttyACM0" );

  /**
   * Close device
   */
  void Close();
  
  /**
   * Destructor
   */
  ~JrkDevice();
  
  /**
   * Get the path to the device
   */
  inline const char* GetPath() const    { return mPath.c_str(); }
  
  /**
   * Transmit the target position (0-4096)
   */
	bool SetPosition( uint16_t target );
	
	/**
	 * Read the current position from the controller
	 */
  int ReadPosition();
  
  /**
	 * Read back one of the JRK_READ_* defines
	 */
  int ReadVariable( uint8_t variable );
  
  /**
   * Read and optionally clear error conditions
   */
  uint16_t ReadErrors( bool reset=false );
  
	/**
	 * Return and clear any error conditions
	 */
	uint16_t ResetErrors();

protected:
  JrkDevice();
  bool Init(const char* device);
  
  std::string mPath;
	int         mFile;
	
	uint8_t  mInputMode;
	
	uint16_t mLimitMin;
	uint16_t mLimitMax;
	
	uint16_t mLastTarget;		// last user-requested position
	int      mLastFeedback;		// last jrk-reported position

	float    mRelativeTarget;
};


inline const char* jrkErrorStr( uint16_t error )
{
	switch(error)
	{
		case JRK_ERROR_AWAITING_CMD:    return "AWAITING_CMD";
		case JRK_ERROR_NO_POWER:        return "NO_POWER";
		case JRK_ERROR_MOTOR_DRIVER:    return "MOTOR_DRIVER";
		case JRK_ERROR_PWM_INPUT:			  return "PWM_INPUT";
		case JRK_ERROR_INPUT_DISCONNECTED:		return "INPUT_DISCONNECTED";
		case JRK_ERROR_FEEDBACK_DISCONNECTED:	return "FEEDBACK_DISCONNECTED";
		case JRK_ERROR_OVERCURRENT:			return "OVERCURRENT";
		case JRK_ERROR_SERIAL_SIGNAL:   return "SERIAL_SIGNAL";
		case JRK_ERROR_SERIAL_OVERRUN:  return "SERIAL_OVERRUN";
		case JRK_ERROR_SERIAL_RX_FULL:  return "SERIAL_RX_FULL";
		case JRK_ERROR_SERIAL_CRC:			return "SERIAL_CRC";
		case JRK_ERROR_SERIAL_PROTOCOL: return "SERIAL_PROTOCOL";
		case JRK_ERROR_SERIAL_TIMEOUT:  return "SERIAL_TIMEOUT";
		case JRK_ERROR_SYSTEM:          return "SYSTEM_API";
		default:  return "UNKNOWN";
	}
}

inline void jrkErrorPrint( uint16_t errors )
{
  printf("[jrk] errors 0x%04hX [", errors);
  
	for( uint32_t n=0; n < JRK_ERROR_COUNT; n++ )
	{
		uint16_t nf = (1<<n);

		if( errors & nf )
			printf(" %s ", jrkErrorStr(nf));
	}
	
	if( errors == JRK_ERROR_SYSTEM )
	  printf(" %s ", jrkErrorStr(errors));
	  
	printf("]\n");
}


#endif

