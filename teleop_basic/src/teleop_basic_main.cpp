#include <teleop_basic/teleop_basic.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop");
   ros::NodeHandle node_handle;

   TeleopNode teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
