#include <teleop_driving_wheel_controller/teleop_driving_wheel_controller.h>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_driving_wheel_controller");
   ros::NodeHandle node_handle;

   Teleop::DrivingWheelControllerNode teleop_husky(&node_handle);

   teleop_husky.processMsgs();

   return 0;
}
