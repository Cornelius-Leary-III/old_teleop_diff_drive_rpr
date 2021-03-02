#include <husky_teleop/husky_teleoperation_smooth.h>

SmoothTeleopNode::SmoothTeleopNode(ros::NodeHandle* node_handle, const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mTwistTopicName(twist_topic),
     mCurrentTwistMsg()
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>(mTwistTopicName, 1);
   mJoySubscriber  = mNodeHandle.subscribe("joy", 10, &SmoothTeleopNode::joyMsgCallback, this);

   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void SmoothTeleopNode::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentTwistMsg.linear.x  = mScaleLinear * joy_msg->axes[mLinearAxisIndex];
   mCurrentTwistMsg.angular.z = mScaleAngular * joy_msg->axes[mAngularAxisIndex];

   ROS_INFO("Current linear X velocity: %f || Current angular Z velocity: (%f)",
            mCurrentTwistMsg.linear.x,
            mCurrentTwistMsg.angular.z);
}

void SmoothTeleopNode::processMsgs()
{
   ros::Rate sleep_timer(10.0);

   while (ros::ok())
   {
      ros::spinOnce();
      sleep_timer.sleep();

      mTwistPublisher.publish(mCurrentTwistMsg);
   }
}
