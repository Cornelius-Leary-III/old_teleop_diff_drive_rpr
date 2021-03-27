#include <teleop_turbo/husky_teleop_turbo.h>

#include <iostream>

TeleopNodeTurbo::TeleopNodeTurbo(ros::NodeHandle* node_handle, const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mDeadmanButtonIndex(0),
     mIsDeadmanPressed(false),
     mIsDeadmanRequired(false),
     mTurboButtonIndex(1),
     mIsTurboPressed(false),
     mIsTurboAllowed(false),
     mScaleTurbo(0.0),
     mTwistTopicName(twist_topic),
     mCurrentTwistMsg(),
     mCurrentJoyMsg()
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

   mNodeHandle.param("teleop_turbo/turbo_allowed", mIsTurboAllowed, mIsTurboAllowed);
   mNodeHandle.param("teleop_turbo/deadman_required", mIsDeadmanRequired, mIsDeadmanRequired);

   ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "\tDeadman Required? " << mIsDeadmanRequired
                                       << std::endl);
   ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "\tTurbo Allowed? " << mIsTurboAllowed << std::endl);

   mScaleTurbo = 1.5;

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>(mTwistTopicName, 1);
   mJoySubscriber  = mNodeHandle.subscribe("joy", 10, &TeleopNodeTurbo::joyMsgCallback, this);

   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void TeleopNodeTurbo::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   if (isDeadmanPressed())
   {
      mCurrentTwistMsg.linear.x  = mScaleLinear * joy_msg->axes[mLinearAxisIndex];
      mCurrentTwistMsg.angular.z = mScaleAngular * joy_msg->axes[mAngularAxisIndex];

      if (isTurboModeActive())
      {
         mCurrentTwistMsg.linear.x *= mScaleTurbo;
      }
   }
   else
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
   }
}

void TeleopNodeTurbo::processMsgs()
{
   ros::Rate sleep_timer(20.0);

   while (ros::ok())
   {
      ros::spinOnce();
      sleep_timer.sleep();

      mTwistPublisher.publish(mCurrentTwistMsg);
   }
}

bool TeleopNodeTurbo::isDeadmanPressed()
{
   mIsDeadmanPressed = mCurrentJoyMsg.buttons.at(mDeadmanButtonIndex);

   return !mIsDeadmanRequired || mIsDeadmanPressed;
}

bool TeleopNodeTurbo::isTurboModeActive()
{
   mIsTurboPressed = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);

   return mIsTurboAllowed && mIsTurboPressed;
}
