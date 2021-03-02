#include <husky_teleop/husky_teleoperation_turbo.h>

TeleopNodeTurbo::TeleopNodeTurbo(ros::NodeHandle* node_handle, const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mDeadmanButtonIndex(0),
     mIsDeadmanPressed(false),
     mTurboButtonIndex(1),
     mIsTurboPressed(false),
     mScaleTurbo(0.0),
     mTwistTopicName(twist_topic),
     mCurrentTwistMsg(),
     mCurrentJoyMsg()
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

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

   return mIsDeadmanPressed;
}

bool TeleopNodeTurbo::isTurboModeActive()
{
   mIsTurboPressed = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);

   return mIsTurboPressed;
}
