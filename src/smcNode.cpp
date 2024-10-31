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
      
      if( !mDevice )
      {
        RCLCPP_ERROR(get_logger(), "Failed to open Pololu Simple Motor Controller USB device");
        throw std::runtime_error("Failed to open Pololu Simple Motor Controller USB device");
      }
      
      mTimer = create_wall_timer(1s,
        std::bind(&SmcNode::ReadState, this)
      );
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
    
    SmcDevice* mDevice;
    rclcpp::TimerBase::SharedPtr mTimer;
};

  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmcNode>());
  rclcpp::shutdown();
  
  return 0;
}
