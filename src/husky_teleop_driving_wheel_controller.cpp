#include <husky_teleop/husky_teleop_driving_wheel_controller.h>

#include <iostream>

namespace Teleop
{
const double gHIDMaxMagnitude        = 1.0;
const double gHIDValueRange          = 2.0 * gHIDMaxMagnitude;
const double gTwistPublishRate       = 10.0;
const double gDefaultTurboScale      = 1.5;
const double gBrakeActiveThreshold   = 0.2;
const double gDeadmanActiveThreshold = 0.0;

DrivingWheelControllerNode::DrivingWheelControllerNode(ros::NodeHandle* node_handle)
   : mNodeHandle(*node_handle),
     mSteeringAxisIndex(0),
     mDeadmanPedalAxisIndex(1),
     mIsDeadmanRequired(false),
     mBrakeAxisIndex(3),
     mIsBrakeApplied(false),
     mCurrentDrivingGear(DrivingGearType::FORWARD),
     mGearScalingFactor(1.0),
     mForwardGearButtonIndex(0),
     mReverseGearButtonIndex(3),
     mThrottleAxisIndex(2),
     mTurboButtonIndex(1),
     mIsTurboPressed(false),
     mIsTurboAllowed(false),
     mScaleTurbo(gDefaultTurboScale),
     mCurrentTwistMsg(),
     mCurrentJoyMsg()
{
   std::string node_name("teleop_driving_wheel_controller/");

   mNodeHandle.param(node_name + "axis_throttle", mThrottleAxisIndex, mThrottleAxisIndex);
   mNodeHandle.param(node_name + "axis_steering", mSteeringAxisIndex, mSteeringAxisIndex);
   mNodeHandle.param(node_name + "axis_brake_pedal", mBrakeAxisIndex, mBrakeAxisIndex);

   mNodeHandle.param(node_name + "scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param(node_name + "scale_angular", mScaleSteering, mScaleSteering);
   mNodeHandle.param(node_name + "scale_turbo", mScaleTurbo, mScaleTurbo);

   mNodeHandle.param(node_name + "axis_deadman", mDeadmanPedalAxisIndex, mDeadmanPedalAxisIndex);
   mNodeHandle.param(node_name + "turbo_button_index", mTurboButtonIndex, mTurboButtonIndex);
   mNodeHandle.param(node_name + "turbo_allowed", mIsTurboAllowed, mIsTurboAllowed);
   mNodeHandle.param(node_name + "deadman_required", mIsDeadmanRequired, mIsDeadmanRequired);

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

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

   processGearButtonStates();

   double current_brake = readBrake();
   if (current_brake > gBrakeActiveThreshold)
   {
      // deceleration?
      mCurrentTwistMsg.linear.x *= (1.0 - current_brake) * mGearScalingFactor;
      return;
   }

   double current_steering    = readSteeringAngle();
   mCurrentTwistMsg.angular.z = mScaleSteering * current_steering /** mGearScalingFactor*/;

   double current_throttle   = readThrottle();
   mCurrentTwistMsg.linear.x = mScaleLinear * current_throttle * mGearScalingFactor;

   if (isTurboModeActive())
   {
      mCurrentTwistMsg.linear.x *= mScaleTurbo;
   }
}

void DrivingWheelControllerNode::processMsgs()
{
   ros::Rate sleep_timer(gTwistPublishRate);

   while (ros::ok())
   {
      ros::spinOnce();

      mTwistPublisher.publish(mCurrentTwistMsg);

      sleep_timer.sleep();
   }
}

bool DrivingWheelControllerNode::isDeadmanPressed()
{
   if (!mIsDeadmanRequired)
   {
      return true;
   }

   double deadman_value = readHardwareInputDevice(mDeadmanPedalAxisIndex);

   return deadman_value > gDeadmanActiveThreshold;
}

bool DrivingWheelControllerNode::isTurboModeActive()
{
   if (isButtonIndexValid(mTurboButtonIndex))
   {
      mIsTurboPressed = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);
   }
   else
   {
      mIsTurboPressed = false;
   }

   return mIsTurboAllowed && mIsTurboPressed;
}

void DrivingWheelControllerNode::processGearButtonStates()
{
   bool valid =
      isButtonIndexValid(mForwardGearButtonIndex) && isButtonIndexValid(mReverseGearButtonIndex);

   if (!valid)
   {
      mCurrentDrivingGear = DrivingGearType::UNKNOWN_INVALID;
      return;
   }

   bool current_fwd_gear_button = mCurrentJoyMsg.buttons.at(mForwardGearButtonIndex);
   bool current_rev_gear_button = mCurrentJoyMsg.buttons.at(mReverseGearButtonIndex);

   switch (mCurrentDrivingGear)
   {
      case DrivingGearType::UNKNOWN_INVALID:
      {
         if (!current_fwd_gear_button && current_fwd_gear_button != mCurrentGearButtonForward)
         {
            mCurrentDrivingGear = DrivingGearType::FORWARD;
         }
         else if (!current_rev_gear_button && current_rev_gear_button != mCurrentGearButtonReverse)
         {
            mCurrentDrivingGear = DrivingGearType::REVERSE;
         }
         break;
      }
      case DrivingGearType::FORWARD:
      {
         if (!current_rev_gear_button && current_rev_gear_button != mCurrentGearButtonReverse)
         {
            mCurrentDrivingGear = DrivingGearType::REVERSE;
         }
         break;
      }
      case DrivingGearType::REVERSE:
      {
         if (!current_fwd_gear_button && current_fwd_gear_button != mCurrentGearButtonForward)
         {
            mCurrentDrivingGear = DrivingGearType::FORWARD;
         }
         break;
      }
   }

   mCurrentGearButtonForward = current_fwd_gear_button;
   mCurrentGearButtonReverse = current_rev_gear_button;

   switch (mCurrentDrivingGear)
   {
      case DrivingGearType::UNKNOWN_INVALID:
      {
         mGearScalingFactor = 0.0;
         break;
      }
      case DrivingGearType::FORWARD:
      {
         mGearScalingFactor = 1.0;
         break;
      }
      case DrivingGearType::REVERSE:
      {
         mGearScalingFactor = -1.0;
         break;
      }
   }
}

double DrivingWheelControllerNode::readThrottle()
{
   return readHardwareInputDevice(mThrottleAxisIndex);
}

double DrivingWheelControllerNode::readBrake()
{
   return readHardwareInputDevice(mBrakeAxisIndex);
}

double DrivingWheelControllerNode::readSteeringAngle()
{
   if (!isAxisIndexValid(mSteeringAxisIndex))
   {
      return 0.0;
   }

   return mCurrentJoyMsg.axes.at(mSteeringAxisIndex);
}

double DrivingWheelControllerNode::readHardwareInputDevice(int device_index)
{
   if (!isAxisIndexValid(device_index))
   {
      return 0.0;
   }

   double raw_value      = mCurrentJoyMsg.axes.at(device_index);
   double adjusted_value = (raw_value + gHIDMaxMagnitude) / gHIDValueRange;

   return adjusted_value;
}

bool DrivingWheelControllerNode::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.axes.size();
}

bool DrivingWheelControllerNode::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.buttons.size();
}
} // namespace Teleop
