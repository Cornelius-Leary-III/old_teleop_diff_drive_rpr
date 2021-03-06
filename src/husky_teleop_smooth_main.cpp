#include <husky_teleop/husky_teleop_smooth.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop_smooth");
   ros::NodeHandle node_handle;

   SmoothTeleopNode teleop_husky(&node_handle, "cmd_vel");

   teleop_husky.processMsgs();

   return 0;
}
