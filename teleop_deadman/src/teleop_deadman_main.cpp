#include <teleop_deadman/teleop_deadman.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_deadman");
   ros::NodeHandle node_handle;

   TeleopNodeDeadman teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
