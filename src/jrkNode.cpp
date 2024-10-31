/*
 * Pololu Jrk serial controller
 */
 
#include "pololu/jrk.h"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <stdexcept>
#include <functional>

using namespace std::chrono_literals;


class JrkNode : public rclcpp::Node
{
  public:
    JrkNode() : Node("jrk")
    {
      mDevice = JrkDevice::Open();
      
      if( !mDevice )
      {
        RCLCPP_ERROR(get_logger(), "Failed to open Jrk serial controller");
        throw std::runtime_error("Failed to open Jrk serial controller");
      }
      
      mTimer = create_wall_timer(1s,
        std::bind(&JrkNode::ReadState, this)
      );
    }
    
    ~JrkNode()
    {
      if( mDevice != NULL )
      {
        delete mDevice;
        mDevice = NULL;
      }
    }

  private:
    void ReadState()
    {
      const uint16_t errors = mDevice->ReadErrors();
      
      if( errors != 0 )
      {
	      jrkErrorPrint(errors);
	      RCLCPP_ERROR(get_logger(), "Jrk errors 0x%02X (%s)", errors, jrkErrorStr(errors));
	    }
	    
      const int pos = mDevice->ReadPosition();
      
      if( pos != -1 )
        RCLCPP_INFO(get_logger(), "Jrk controller position:  %i", pos);
      else
        RCLCPP_ERROR(get_logger(), "Failed to read position from Jrk controller");
    }
    
    JrkDevice* mDevice;
    rclcpp::TimerBase::SharedPtr mTimer;
};

  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JrkNode>());
  rclcpp::shutdown();
  
  return 0;
}
