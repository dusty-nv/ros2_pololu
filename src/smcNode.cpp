/*
 * Pololu Simple Motor Controller
 */
 
#include "pololu/smc.h"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <stdexcept>
#include <functional>

using namespace std::chrono_literals;


class SmcNode : public rclcpp::Node
{
  public:
    SmcNode() : Node("smc")
    {
      mDevice = SmcDevice::Open();
      mTestSeq = 0;
      
      if( !mDevice )
      {
        RCLCPP_ERROR(get_logger(), "Failed to open Pololu Simple Motor Controller USB device");
        throw std::runtime_error("Failed to open Pololu Simple Motor Controller USB device");
      }
      
      mTimer = create_wall_timer(1s,
        std::bind(&SmcNode::ReadState, this)
      );
      
      /*create_wall_timer(2s,
        std::bind(&SmcNode::TestMotor, this)
      );*/
    }
    
    ~SmcNode()
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
      mDevice->PrintVariables();
    }
    
    void TestMotor()
    {
      int speed = 800;
      
      if( mTestSeq++ % 2 != 0 )
        speed *= -1;
        
      RCLCPP_INFO(get_logger(), "Test sequence:  setting motor to %i", speed);
      
      if( !mDevice->SetSpeed(speed) )
        RCLCPP_ERROR(get_logger(), "Failed to set motor to %i during test sequence", speed);
    }
    
    SmcDevice* mDevice;
    rclcpp::TimerBase::SharedPtr mTimer;
    
    int mTestSeq;
};

  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmcNode>());
  rclcpp::shutdown();
  
  return 0;
}
