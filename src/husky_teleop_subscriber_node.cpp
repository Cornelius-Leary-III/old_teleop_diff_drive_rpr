#include <husky_teleop/husky_teleop_subscriber_node.h>

TeleopSubscriberNode::TeleopSubscriberNode(int argc, char** argv, QObject* parent)
   : QThread(parent),
     mNodeHandle(nullptr),
     mArgC(argc),
     mArgV(argv)
{
}

TeleopSubscriberNode::~TeleopSubscriberNode()
{
   if (mNodeHandle != nullptr)
   {
      delete mNodeHandle;
      mNodeHandle = nullptr;
   }
}

void TeleopSubscriberNode::run()
{
   ros::init(mArgC, mArgV, "husky_teleop_gui");

   mNodeHandle = new ros::NodeHandle();

   ros::AsyncSpinner spinner(0);
   spinner.start();

   mTwistSubscriber =
      mNodeHandle->subscribe("cmd_vel", 10, &TeleopSubscriberNode::twistMsgCallback, this);

   mJoySubscriber = mNodeHandle->subscribe("joy", 10, &TeleopSubscriberNode::joyMsgCallback, this);

   mOdometrySubscriber =
      mNodeHandle->subscribe("odometry/filtered", 10, &TeleopSubscriberNode::odomMsgCallback, this);

   ros::waitForShutdown();
}

void TeleopSubscriberNode::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
   emit velocityCommanded(twist_msg);
}

void TeleopSubscriberNode::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   emit joystickMsgReceived(joy_msg);
}

void TeleopSubscriberNode::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
   emit odometryMsgReceived(odometry_msg);
}

#include "husky_teleop_subscriber_node.moc"
