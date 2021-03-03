#include <husky_teleop/husky_teleoperation_gui.h>

TeleopNodeGui::TeleopNodeGui(ros::NodeHandle* node_handle)
   : QObject(nullptr),
     mNodeHandle(*node_handle),
     mLinearAxisIndex(1),
     mAngularAxisIndex(2),
     mDeadmanButtonIndex(0),
     mIsDeadmanPressed(false),
     mTurboButtonIndex(1),
     mIsTurboPressed(false),
     mScaleTurbo(0.0),
     mCurrentTwistMsg(),
     mCurrentJoyMsg(),
     mCurrentOdometryMsg()
{
   mNodeHandle.param("axis_linear", mLinearAxisIndex, mLinearAxisIndex);
   mNodeHandle.param("axis_angular", mAngularAxisIndex, mAngularAxisIndex);

   mNodeHandle.param("scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("scale_angular", mScaleAngular, mScaleAngular);

   mNodeHandle.param("scale_turbo", mScaleTurbo, mScaleTurbo);

   mTwistSubscriber = mNodeHandle.subscribe("cmd_vel", 10, &TeleopNodeGui::twistMsgCallback, this);
   mJoySubscriber   = mNodeHandle.subscribe("joy", 10, &TeleopNodeGui::joyMsgCallback, this);
   mOdometrySubscriber = mNodeHandle.subscribe("odometry/filtered", 10, &TeleopNodeGui::odomMsgCallback, this);
}

void TeleopNodeGui::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
   mCurrentTwistMsg = *twist_msg;
}

void TeleopNodeGui::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   //   if (isDeadmanPressed())
   //   {
   //      mCurrentTwistMsg.linear.x  = mScaleLinear * joy_msg->axes[mLinearAxisIndex];
   //      mCurrentTwistMsg.angular.z = mScaleAngular * joy_msg->axes[mAngularAxisIndex];

   //      if (isTurboModeActive())
   //      {
   //         mCurrentTwistMsg.linear.x *= mScaleTurbo;
   //      }
   //   }
   //   else
   //   {
   //      mCurrentTwistMsg.linear.x  = 0.0;
   //      mCurrentTwistMsg.angular.z = 0.0;
   //   }
}

void TeleopNodeGui::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
   mCurrentOdometryMsg = *odometry_msg;
}

void TeleopNodeGui::processMsgs()
{
   //   ros::Rate sleep_timer(20.0);

   //   while (ros::ok())
   //   {
   //      ros::spinOnce();
   //      sleep_timer.sleep();
   //
   //      mTwistPublisher.publish(mCurrentTwistMsg);
   //   }

   ros::spin();
}

bool TeleopNodeGui::isDeadmanPressed()
{
   mIsDeadmanPressed = mCurrentJoyMsg.buttons.at(mDeadmanButtonIndex);

   return mIsDeadmanPressed;
}

bool TeleopNodeGui::isTurboModeActive()
{
   mIsTurboPressed = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);

   return mIsTurboPressed;
}

#include "husky_teleoperation_gui.moc"
