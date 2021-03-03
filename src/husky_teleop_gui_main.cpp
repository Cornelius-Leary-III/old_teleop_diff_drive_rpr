#include <husky_teleop/husky_teleoperation_gui.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "husky_teleop_gui");
   ros::NodeHandle node_handle;

   TeleopNodeGui teleop_gui(&node_handle);

   teleop_gui.processMsgs();

   return 0;
}
