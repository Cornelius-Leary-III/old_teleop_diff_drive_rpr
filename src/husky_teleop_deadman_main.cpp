#include <husky_teleop/husky_teleoperation_deadman.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop_deadman");
   ros::NodeHandle node_handle;

   TeleopNodeDeadman teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
