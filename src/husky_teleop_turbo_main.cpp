#include <husky_teleop/husky_teleop_turbo.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop_turbo");
   ros::NodeHandle node_handle;

   TeleopNodeTurbo teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
