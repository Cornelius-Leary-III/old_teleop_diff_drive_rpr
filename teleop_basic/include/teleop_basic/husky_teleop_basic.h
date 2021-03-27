#ifndef HUSKY_TELEOP_BASIC_H
#define HUSKY_TELEOP_BASIC_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNode
{
public:
   TeleopNode(ros::NodeHandle* node_handle, const std::string& twist_topic);

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   ros::NodeHandle mNodeHandle;

   int    mLinearAxisIndex;
   int    mAngularAxisIndex;
   double mScaleLinear;
   double mScaleAngular;

   std::string    mTwistTopicName;
   ros::Publisher mTwistPublisher;

   ros::Subscriber mJoySubscriber;
};

#endif // HUSKY_TELEOP_BASIC_H
