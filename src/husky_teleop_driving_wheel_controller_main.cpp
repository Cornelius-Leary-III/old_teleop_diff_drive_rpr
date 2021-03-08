#include <husky_teleop/husky_teleop_driving_wheel_controller.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop_driving_wheel_controller");
   ros::NodeHandle node_handle;

   Teleop::DrivingWheelControllerNode teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
