#include <teleop_deadman/teleop_deadman.h>

TeleopNodeDeadman::TeleopNodeDeadman(ros::NodeHandle* node_handle, const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mDeadmanButtonIndex(0),
     mIsDeadmanPressed(false),
     mTwistTopicName(twist_topic),
     mCurrentTwistMsg()
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>(mTwistTopicName, 1);
   mJoySubscriber  = mNodeHandle.subscribe("joy", 10, &TeleopNodeDeadman::joyMsgCallback, this);

   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void TeleopNodeDeadman::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mIsDeadmanPressed = joy_msg->buttons.at(mDeadmanButtonIndex);

   if (mIsDeadmanPressed)
   {
      mCurrentTwistMsg.linear.x  = mScaleLinear * joy_msg->axes[mLinearAxisIndex];
      mCurrentTwistMsg.angular.z = mScaleAngular * joy_msg->axes[mAngularAxisIndex];
   }
   else
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
   }

   ROS_INFO("Current linear X velocity: %f || Current angular Z velocity: (%f)",
            mCurrentTwistMsg.linear.x,
            mCurrentTwistMsg.angular.z);
}

void TeleopNodeDeadman::processMsgs()
{
   ros::Rate sleep_timer(10.0);

   while (ros::ok())
   {
      ros::spinOnce();
      sleep_timer.sleep();

      mTwistPublisher.publish(mCurrentTwistMsg);
   }
}
