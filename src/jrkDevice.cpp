/**
 * Pololu Jrk serial controller
 */

#include "pololu/jrk.h"

#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>


// 16-bit clamp
static inline uint16_t clamp16u( uint16_t value, uint16_t lower, uint16_t upper )
{
	if( value < lower )		return lower;
	else if( value > upper ) return upper;
	else					return value;
}


// constructor
JrkDevice::JrkDevice() : mFile(-1)
{
	mLimitMin       = 0;
	mLimitMax       = 4096;
	mLastTarget     = 2048;
	mLastFeedback   = 0;
	mRelativeTarget = 0.0f;
	mInputMode      = JRK_INPUT_ABSOLUTE;
}


// destructor
JrkDevice::~JrkDevice()
{
  Close();
}


// Init
bool JrkDevice::Init( const char* path )
{
  mFile = open(path, O_RDWR|O_NOCTTY);
  mPath = path;
  
	if( mFile < 0 )
	{
		const int err = errno;
		printf("[jrk] failed to open serial port %s  (err=%i %s)\n", path, err, strerror(err));
		return false;
	}
	
	printf("[jrk] opened serial device %s\n", path);
	
	// serial port options
	struct termios options;
	tcgetattr(mFile, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(mFile, TCSANOW, &options);
	
  // retrieve the motor's position at start-up
	mLastFeedback = ReadPosition();

  if( mLastFeedback == -1 )
    return false;
    
	printf("[jrk] %s  initial position feedback %i\n", path, mLastFeedback);
	
	if( mLastFeedback >= 0 && mLastFeedback < 4096 )
	{
		mLastTarget = clamp16u(mLastFeedback, mLimitMin, mLimitMax);
		mRelativeTarget = float(mLastTarget);
	}
	
	// check for motor errors
	const uint16_t read_errors = ReadErrors();

	jrkErrorPrint(read_errors);

  if( read_errors == JRK_ERROR_SYSTEM )
    return false;	

	return true;
}


// Open
JrkDevice* JrkDevice::Open( const char* path )
{
  JrkDevice* dev = new JrkDevice();
  
  if( !dev->Init(path) )
  {
    delete dev;
    return NULL;
  }
  
  return dev;
}


// Close
void JrkDevice::Close()
{
	if( mFile >= 0 )
	{
		close(mFile);
		mFile = -1;
	}
}


// SetPosition
bool JrkDevice::SetPosition( uint16_t target )
{
	if( target > mLimitMax || target < mLimitMin )
	{
		printf("[jrk] invalid target position (%hu) (min=%hu max=%hu)\n", target, mLimitMin, mLimitMax);
		return false;
	}
	
	if( mLastTarget == target )
		return true;
		
	uint8_t cmd[] = { 0xC0 + (target & 0x1F), (target >> 5) & 0x7F };
	
	if( write(mFile, cmd, sizeof(cmd)) < 0 )
	{
		const int err = errno;
		printf("[jrk] writing %s failed during SetPosition() (err=%i %s)\n", mPath.c_str(), err, strerror(err));
		return false;
	}
	
	mLastTarget = target;
	return true;
}


// ReadPosition
int JrkDevice::ReadPosition()
{
  return ReadVariable(JRK_READ_FEEDBACK);
}


// ReadVariable
int JrkDevice::ReadVariable( uint8_t variable )
{
	if( mFile < 0 )
		return -1;
		
	if( write(mFile, &variable, 1) < 0 )
	{
		const int err = errno;
		printf("[jrk] writing %s failed while reading variable 0x%02X  (err=%i %s)\n", mPath.c_str(), (uint32_t)variable, err, strerror(err));
		return -1;
	}

	uint8_t response[2];
	const int result = read(mFile, response, 2);
	
	if( result != 2 )
	{
		printf("[jrk] reading %s failed while reading variable 0x%02X  (%i of 2 bytes)\n", mPath.c_str(), (uint32_t)variable, result);
		return -1;
	}
	
	return response[0] + 256 * response[1];
}


// ReadErrors
uint16_t JrkDevice::ReadErrors( bool reset )
{
	const uint8_t cmd = reset ? 0xB3 : 0xB5;

	if( write(mFile, &cmd, 1) < 0 )
	{
		const int err = errno;
		printf("[jrk] writing %s failed while reading errors (err=%i %s)\n", mPath.c_str(), err, strerror(err));
		return JRK_ERROR_SYSTEM;
	}
	
	uint16_t errors = 0;
	
	const int result = read(mFile, &errors, 2);
	
	if( result != 2 )
	{
		printf("[jrk] reading %s failed while reading errors (%i of 2 bytes)\n", mPath.c_str(), result);
		return JRK_ERROR_SYSTEM;
	}
	
	return errors;
}


// ResetErrors
uint16_t JrkDevice::ResetErrors()
{
	return ReadErrors(true);
}


#if 0 
if( mInputMode == JRK_INPUT_RELATIVE )
	{
		const float input = inputPtr[0];

		if( mLastFeedback >= 0 )
			mRelativeTarget = mLastFeedback + input; 
		else
			mRelativeTarget += input;

		if( mRelativeTarget > mLimitMax ) 		mRelativeTarget = mLimitMax;
		else if( mRelativeTarget < mLimitMin )  mRelativeTarget = mLimitMin;

		printf("%s  input %f  rel %f\n", GetInstanceName(), input, mRelativeTarget);

		setTarget(mRelativeTarget);
	}
	else if( mInputMode == JRK_INPUT_ABSOLUTE )
	{
		const float input = clampf(inputPtr[0], -1.0f, 1.0f);
		const float range = (mLimitMax - mLimitMin) / 2;
		const float value = ((input + 1.0f) * range) + mLimitMin;

		const uint16_t tgt = (uint16_t)value;

		printf("%s  input %f  abs %hu\n", GetInstanceName(), input, tgt);
		setTarget(tgt);
	}
#endif

