#include <teleop_telemetry_gui/teleop_telemetry_subscriber_node.h>

TelemetrySubscriberNode::TelemetrySubscriberNode(int argc, char** argv, QObject* parent)
   : QThread(parent),
     mNodeHandle(nullptr),
     mArgC(argc),
     mArgV(argv)
{
}

TelemetrySubscriberNode::~TelemetrySubscriberNode()
{
   if (mNodeHandle != nullptr)
   {
      delete mNodeHandle;
      mNodeHandle = nullptr;
   }
}

void TelemetrySubscriberNode::run()
{
   ros::init(mArgC, mArgV, "telemetry_subscriber_node");

   mNodeHandle = new ros::NodeHandle();

   ros::AsyncSpinner spinner(0);
   spinner.start();

   mTwistSubscriber =
      mNodeHandle->subscribe("cmd_vel", 10, &TelemetrySubscriberNode::twistMsgCallback, this);

   mJoySubscriber =
      mNodeHandle->subscribe("joy", 10, &TelemetrySubscriberNode::joyMsgCallback, this);

   mOdometrySubscriber = mNodeHandle->subscribe("odometry/filtered",
                                                10,
                                                &TelemetrySubscriberNode::odomMsgCallback,
                                                this);

   ros::waitForShutdown();
}

void TelemetrySubscriberNode::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
   emit velocityCommanded(twist_msg);
}

void TelemetrySubscriberNode::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   emit joystickMsgReceived(joy_msg);
}

void TelemetrySubscriberNode::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
   emit odometryMsgReceived(odometry_msg);
}

#include "teleop_telemetry_subscriber_node.moc"
