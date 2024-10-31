/*
 * Pololu Maestro servo controller
 */
 
#include "pololu/maestro.h"
#include "rclcpp/rclcpp.hpp"

#include <stdexcept>


class MaestroNode : public rclcpp::Node
{
  public:
    MaestroNode() : Node("maestro")
    {
      mDevice = MaestroDevice::Open();
      
      if( !mDevice )
      {
        RCLCPP_ERROR(get_logger(), "Failed to open Pololu Maestro USB device");
        throw std::runtime_error("Failed to open Pololu Maestro USB device");
      }
    }
    
    ~MaestroNode()
    {
      if( mDevice != NULL )
      {
        delete mDevice;
        mDevice = NULL;
      }
    }

  private:
    MaestroDevice* mDevice;
};

  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaestroNode>());
  rclcpp::shutdown();
  
  return 0;
}
