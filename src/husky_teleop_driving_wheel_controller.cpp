#include <husky_teleop/husky_teleop_driving_wheel_controller.h>

#include <iostream>

namespace Teleop
{
static const double gHIDMaxMagnitude = 1.0;
static const double gHIDValueRange   = 2.0 * gHIDMaxMagnitude;

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
   std::string node_name("teleop_turbo/");

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

   double deadman_value = readHardwareInputDevice(mDeadmanPedalAxisIndex);

   return deadman_value > mDeadmanActiveThreshold;
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
   double throttle_value = readHardwareInputDevice(mThrottleAxisIndex);

   return throttle_value;
}

double DrivingWheelControllerNode::readBrake()
{
   double brake_value = readHardwareInputDevice(mBrakeAxisIndex);

   mIsBrakeApplied = brake_value > mBrakeActiveThreshold;

   return brake_value;
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
   return static_cast<size_t>(index) <= mCurrentJoyMsg.axes.size();
}

bool DrivingWheelControllerNode::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) <= mCurrentJoyMsg.buttons.size();
}
} // namespace Teleop
