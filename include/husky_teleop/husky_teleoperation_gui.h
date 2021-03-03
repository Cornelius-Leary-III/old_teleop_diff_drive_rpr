#ifndef HUSKY_TELEOPERATION_GUI_H
#define HUSKY_TELEOPERATION_GUI_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <QObject>

class TeleopNodeGui : public QObject
{
   Q_OBJECT

public:
   TeleopNodeGui(ros::NodeHandle* node_handle);

   virtual ~TeleopNodeGui()
   {
   }

   void processMsgs();

   void twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg);

   void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);

   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

signals:
public slots:

private:
   bool isDeadmanPressed();
   bool isTurboModeActive();

   ros::NodeHandle mNodeHandle;

   int    mLinearAxisIndex;
   int    mAngularAxisIndex;
   double mScaleLinear;
   double mScaleAngular;

   int  mDeadmanButtonIndex;
   bool mIsDeadmanPressed;

   int    mTurboButtonIndex;
   bool   mIsTurboPressed;
   double mScaleTurbo;

   ros::Subscriber      mTwistSubscriber;
   geometry_msgs::Twist mCurrentTwistMsg;

   ros::Subscriber  mJoySubscriber;
   sensor_msgs::Joy mCurrentJoyMsg;

   ros::Subscriber    mOdometrySubscriber;
   nav_msgs::Odometry mCurrentOdometryMsg;
};

#endif // HUSKY_TELEOPERATION_GUI_H
