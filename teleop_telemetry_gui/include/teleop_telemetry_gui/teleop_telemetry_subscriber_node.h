#ifndef TELEOP_GUI_H
#define TELEOP_GUI_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <QObject>
#include <QThread>

class TelemetrySubscriberNode : public QThread
{
   Q_OBJECT

public:
   TelemetrySubscriberNode(int argc, char** argv, QObject* parent = nullptr);

   virtual ~TelemetrySubscriberNode();

   void run() override;

signals:
   void velocityCommanded(const geometry_msgs::Twist::ConstPtr& twist_msg);
   void odometryMsgReceived(const nav_msgs::Odometry::ConstPtr& odometry_msg);
   void joystickMsgReceived(const sensor_msgs::Joy::ConstPtr& joy_msg);

public slots:
private:
   void twistMsgCallback(const geometry_msgs::Twist::ConstPtr& twist_msg);
   void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   ros::NodeHandle* mNodeHandle;

   int    mArgC;
   char** mArgV;

   ros::Subscriber mTwistSubscriber;
   ros::Subscriber mJoySubscriber;
   ros::Subscriber mOdometrySubscriber;
};

#endif // TELEOP_GUI_H
