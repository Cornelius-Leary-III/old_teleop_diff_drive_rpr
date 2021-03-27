#include <teleop_smooth/teleop_smooth.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_smooth");
   ros::NodeHandle node_handle;

   SmoothTeleopNode teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
