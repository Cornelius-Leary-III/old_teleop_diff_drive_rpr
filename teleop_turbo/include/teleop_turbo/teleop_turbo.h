#ifndef TELEOP_TURBO_H
#define TELEOP_TURBO_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNodeTurbo
{
public:
   TeleopNodeTurbo(ros::NodeHandle* node_handle, const std::string& twist_topic);

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   bool isDeadmanPressed();
   bool isTurboModeActive();

   ros::NodeHandle mNodeHandle;

   int    mLinearAxisIndex;
   int    mAngularAxisIndex;
   double mScaleLinear;
   double mScaleAngular;

   int  mDeadmanButtonIndex;
   bool mIsDeadmanPressed;
   bool mIsDeadmanRequired;

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

#endif // TELEOP_TURBO_H
