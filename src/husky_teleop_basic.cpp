#include <husky_teleop/husky_teleop_basic.h>

TeleopNode::TeleopNode(ros::NodeHandle* node_handle, const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mTwistTopicName(twist_topic)
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>(mTwistTopicName, 1);
   mJoySubscriber  = mNodeHandle.subscribe("joy", 10, &TeleopNode::joyMsgCallback, this);
}

void TeleopNode::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   geometry_msgs::Twist twist_msg;
   twist_msg.linear.x  = mScaleLinear * joy_msg->axes[mLinearAxisIndex];
   twist_msg.angular.z = mScaleAngular * joy_msg->axes[mAngularAxisIndex];

   mTwistPublisher.publish(twist_msg);
}

void TeleopNode::processMsgs()
{
   ros::Rate sleep_timer(10.0);

   while (ros::ok())
   {
      ros::spinOnce();
      sleep_timer.sleep();
   }
}
