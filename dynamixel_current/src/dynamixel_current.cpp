#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <std_msgs/Float64.h>
#include "dynamixel_current.hpp"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;


void currentCallback(const std_msgs::Float64::ConstPtr &msg)
{
  goal_current_value = msg->data;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dynamixel_current_control");
  ros::NodeHandle n;
  ros::Subscriber current_sub = n.subscribe("current_command", 1000, currentCallback);

  portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyACM0");
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (portHandler->openPort())
  {
    ROS_INFO("Succeeded to open the port!");
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
    return 0;
  }

  portHandler->setBaudRate(baudrate);

  // Set the current control mode for the Dynamixel actuator
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, 64, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
    return 0;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
    return 0;
  }
  else
  {
    ROS_INFO("Successfully set the current control mode!");
  }

  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    // Read the Pressent Current value from the Dynmaixel 
    int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, 126, (uint16_t*)&present_current_value, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
      return 0;
    }
    else if (dxl_error != 0)
    {
      ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
      return 0;
    }
    else
    {
      // ROS_INFO("Successfully wrote the current value to the Dynamixel actuator!");
      ROS_INFO("Present Current value : %d", present_current_value);
    }

    // Write the Goal Current value to the Dynamixel actuator
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, 102, goal_current_value, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
      return 0;
    }
    else if (dxl_error != 0)
    {
      ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
      return 0;
    }
    else
    {
      // ROS_INFO("Successfully wrote the current value to the Dynamixel actuator!");
      // ROS_INFO("Goal Current value : %d",goal_current_value);
    }

    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}