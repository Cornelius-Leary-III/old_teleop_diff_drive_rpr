#ifndef HUSKY_TELEOP_DRIVING_WHEEL_CONTROLLER_H
#define HUSKY_TELEOP_DRIVING_WHEEL_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace Teleop
{
class DrivingWheelControllerNode
{
public:
   DrivingWheelControllerNode(ros::NodeHandle* node_handle, const std::string& twist_topic);

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   double readThrottle();
   double readBrake();
   double readSteeringAngle();
   double readHardwareInputDevice(int device_index);

   bool isDeadmanPressed();
   bool isTurboModeActive();

   bool isAxisIndexValid(int index);
   bool isButtonIndexValid(int index);

   ros::NodeHandle mNodeHandle;

   int    mSteeringAxisIndex;
   double mScaleSteering;

   int  mDeadmanPedalAxisIndex;
   bool mIsDeadmanRequired;

   int  mBrakeAxisIndex;
   bool mIsBrakeApplied;

   double mScaleLinear;
   int    mThrottleAxisIndex;
   int    mTurboButtonIndex;
   bool   mIsTurboPressed;
   bool   mIsTurboAllowed;
   double mScaleTurbo;

   std::string          mTwistTopicName;
   ros::Publisher       mTwistPublisher;
   geometry_msgs::Twist mCurrentTwistMsg;

   ros::Subscriber  mJoySubscriber;
   sensor_msgs::Joy mCurrentJoyMsg;
};
} // namespace Teleop

#endif // HUSKY_TELEOP_DRIVING_WHEEL_CONTROLLER_H
