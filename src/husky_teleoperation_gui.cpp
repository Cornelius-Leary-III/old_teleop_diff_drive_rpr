#include <husky_teleop/husky_teleoperation_gui.h>

TeleopNodeGui::TeleopNodeGui(int argc, char** argv, QObject* parent)
   : QThread(parent),
     mNodeHandle(nullptr),
     mArgC(argc),
     mArgV(argv)
{
}

TeleopNodeGui::~TeleopNodeGui()
{
   if (mNodeHandle != nullptr)
   {
      delete mNodeHandle;
      mNodeHandle = nullptr;
   }
}

void TeleopNodeGui::run()
{
   ros::init(mArgC, mArgV, "husky_teleop_gui");

   mNodeHandle = new ros::NodeHandle();

   ros::AsyncSpinner spinner(0);
   spinner.start();

   mTwistSubscriber = mNodeHandle->subscribe("cmd_vel", 10, &TeleopNodeGui::twistMsgCallback, this);

   mJoySubscriber = mNodeHandle->subscribe("joy", 10, &TeleopNodeGui::joyMsgCallback, this);

   mOdometrySubscriber =
      mNodeHandle->subscribe("odometry/filtered", 10, &TeleopNodeGui::odomMsgCallback, this);

   ros::waitForShutdown();
}

void TeleopNodeGui::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
   emit velocityCommanded(twist_msg);
}

void TeleopNodeGui::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   emit joystickMsgReceived(joy_msg);
}

void TeleopNodeGui::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
   emit odometryMsgReceived(odometry_msg);
}

#include "husky_teleoperation_gui.moc"
