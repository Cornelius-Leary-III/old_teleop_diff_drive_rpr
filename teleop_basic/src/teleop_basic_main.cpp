#include <teleop_basic/husky_teleop_basic.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop");
   ros::NodeHandle node_handle;

   TeleopNode teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
