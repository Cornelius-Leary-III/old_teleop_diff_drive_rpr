#include <husky_teleop/husky_teleop_driving_wheel_controller.h>

#include <iostream>

namespace Teleop
{
DrivingWheelControllerNode::DrivingWheelControllerNode(ros::NodeHandle*   node_handle,
                                                       const std::string& twist_topic)
   : mNodeHandle(*node_handle),
     mThrottleAxisIndex(2),
     mBrakeAxisIndex(3),
     mSteeringAxisIndex(0),
     mDeadmanPedalAxisIndex(1),
     mDeadmanActiveThreshold(0),
     mThrottleActiveThreshold(0.2),
     mBrakeActiveThreshold(0.2),
     mIsBrakeApplied(false),
     mIsDeadmanPressed(false),
     mIsDeadmanRequired(false),
     mTurboButtonIndex(1),
     mIsTurboPressed(false),
     mIsTurboAllowed(false),
     mScaleTurbo(1.5),
     mTwistTopicName(twist_topic),
     mCurrentTwistMsg(),
     mCurrentJoyMsg()
{
   mNodeHandle.param("teleop_turbo/axis_throttle", mThrottleAxisIndex, mThrottleAxisIndex);
   mNodeHandle.param("teleop_turbo/axis_steering", mSteeringAxisIndex, mSteeringAxisIndex);
   mNodeHandle.param("teleop_turbo/axis_brake_pedal", mBrakeAxisIndex, mBrakeAxisIndex);

   mNodeHandle.param("teleop_turbo/scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param("teleop_turbo/scale_angular", mScaleSteering, mScaleSteering);
   mNodeHandle.param("teleop_turbo/scale_turbo", mScaleTurbo, mScaleTurbo);

   mNodeHandle.param("teleop_turbo/axis_deadman", mDeadmanPedalAxisIndex, mDeadmanPedalAxisIndex);
   mNodeHandle.param("teleop_turbo/turbo_button_index", mTurboButtonIndex, mTurboButtonIndex);
   mNodeHandle.param("teleop_turbo/turbo_allowed", mIsTurboAllowed, mIsTurboAllowed);
   mNodeHandle.param("teleop_turbo/deadman_required", mIsDeadmanRequired, mIsDeadmanRequired);

   std::cout << __PRETTY_FUNCTION__ << "\tDeadman Required? " << mIsDeadmanRequired << std::endl;
   std::cout << __PRETTY_FUNCTION__ << "\tTurbo Allowed? " << mIsTurboAllowed << std::endl;

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>(mTwistTopicName, 1);
   mJoySubscriber =
      mNodeHandle.subscribe("joy", 10, &DrivingWheelControllerNode::joyMsgCallback, this);

   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void DrivingWheelControllerNode::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   if (!isDeadmanPressed())
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
      return;
   }

   double current_brake       = readBrake();
   double current_steering    = readSteeringAngle();
   mCurrentTwistMsg.angular.z = mScaleSteering * current_steering;

   if (mIsBrakeApplied)
   {
      // deceleration?
      mCurrentTwistMsg.linear.x *= (1.0 - current_brake);
      return;
   }

   double current_throttle   = readThrottle();
   mCurrentTwistMsg.linear.x = mScaleLinear * current_throttle;

   if (isTurboModeActive())
   {
      mCurrentTwistMsg.linear.x *= mScaleTurbo;
   }
}

void DrivingWheelControllerNode::processMsgs()
{
   ros::Rate sleep_timer(10.0);

   while (ros::ok())
   {
      ros::spinOnce();
      sleep_timer.sleep();

      mTwistPublisher.publish(mCurrentTwistMsg);
   }
}

bool DrivingWheelControllerNode::isDeadmanPressed()
{
   if (!mIsDeadmanRequired)
   {
      return true;
   }

   bool is_deadman_pressed = false;

   if (isAxisIndexValid(mDeadmanPedalAxisIndex))
   {
      double deadman_raw   = mCurrentJoyMsg.axes.at(mDeadmanPedalAxisIndex);
      double deadman_value = (deadman_raw + 1.0) / 2.0;

      is_deadman_pressed = deadman_value > mDeadmanActiveThreshold;

      std::cout << __PRETTY_FUNCTION__ << "\tdeadman raw:        " << deadman_raw << "\n";
      std::cout << __PRETTY_FUNCTION__ << "\tdeadman processed:  " << deadman_value << "\n";
      std::cout << __PRETTY_FUNCTION__ << "\tis deadman pressed? " << is_deadman_pressed << "\n";
   }

   return is_deadman_pressed;
}

bool DrivingWheelControllerNode::isTurboModeActive()
{
   if (isButtonIndexValid(mTurboButtonIndex))
   {
      mIsTurboPressed = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);
   }

   return mIsTurboAllowed && mIsTurboPressed;
}

double DrivingWheelControllerNode::readThrottle()
{
   double throttle_value = 0.0;

   if (!isAxisIndexValid(mThrottleAxisIndex))
   {
      return throttle_value;
   }

   double throttle_raw = mCurrentJoyMsg.axes.at(mThrottleAxisIndex);
   throttle_value      = (throttle_raw + 1.0) / 2.0;

   std::cout << __PRETTY_FUNCTION__ << "\tthrottle raw:        " << throttle_raw << "\n";
   std::cout << __PRETTY_FUNCTION__ << "\tthrottle processed:  " << throttle_value << "\n";

   return throttle_value;
}

double DrivingWheelControllerNode::readBrake()
{
   double brake_value = 0.0;

   if (!isAxisIndexValid(mBrakeAxisIndex))
   {
      return brake_value;
   }

   double brake_raw = mCurrentJoyMsg.axes.at(mBrakeAxisIndex);
   brake_value      = (brake_raw + 1.0) / 2.0;

   mIsBrakeApplied = brake_value > mBrakeActiveThreshold;

   std::cout << __PRETTY_FUNCTION__ << "\tbrake raw:        " << brake_raw << "\n";
   std::cout << __PRETTY_FUNCTION__ << "\tbrake processed:  " << brake_value << "\n";
   std::cout << __PRETTY_FUNCTION__ << "\tis brake pressed? " << mIsBrakeApplied << "\n";

   return brake_value;
}

double DrivingWheelControllerNode::readSteeringAngle()
{
   double steering_angle_value = 0.0;

   if (!isAxisIndexValid(mSteeringAxisIndex))
   {
      return steering_angle_value;
   }

   double steering_angle_raw = mCurrentJoyMsg.axes.at(mSteeringAxisIndex);

   return steering_angle_raw;
}

bool DrivingWheelControllerNode::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) <= mCurrentJoyMsg.axes.size();
}

bool DrivingWheelControllerNode::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) <= mCurrentJoyMsg.buttons.size();
}
} // namespace Teleop
