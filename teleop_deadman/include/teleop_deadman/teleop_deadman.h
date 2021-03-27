#ifndef TELEOP_DEADMAN_H
#define TELEOP_DEADMAN_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNodeDeadman
{
public:
   TeleopNodeDeadman(ros::NodeHandle* node_handle, const std::string& twist_topic);

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   ros::NodeHandle mNodeHandle;

   int    mLinearAxisIndex;
   int    mAngularAxisIndex;
   double mScaleLinear;
   double mScaleAngular;

   int  mDeadmanButtonIndex;
   bool mIsDeadmanPressed;

   std::string          mTwistTopicName;
   ros::Publisher       mTwistPublisher;
   geometry_msgs::Twist mCurrentTwistMsg;

   ros::Subscriber mJoySubscriber;
};

#endif // TELEOP_DEADMAN_H
