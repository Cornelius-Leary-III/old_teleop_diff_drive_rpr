#ifndef HUSKY_TELEOPERATION_GUI_H
#define HUSKY_TELEOPERATION_GUI_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <QObject>
#include <QThread>

class TeleopNodeGui : public QThread
{
   Q_OBJECT

public:
   TeleopNodeGui(int argc, char** argv, QObject* parent = nullptr);

   virtual ~TeleopNodeGui();

   void run() override;

   void twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg);

   void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);

   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

signals:
   void velocityCommanded(const geometry_msgs::Twist::ConstPtr& twist_msg);

   void odometryMsgReceived(const nav_msgs::Odometry::ConstPtr& odometry_msg);

   void joystickMsgReceived(const sensor_msgs::Joy::ConstPtr& joy_msg);

public slots:
private:
   ros::NodeHandle* mNodeHandle;

   int    mArgC;
   char** mArgV;

   ros::Subscriber mTwistSubscriber;
   ros::Subscriber mJoySubscriber;
   ros::Subscriber mOdometrySubscriber;
};

#endif // HUSKY_TELEOPERATION_GUI_H
