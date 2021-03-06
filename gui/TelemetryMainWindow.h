#ifndef TELEOPMAINWINDOW_H
#define TELEOPMAINWINDOW_H

#include <string>
#include <QMainWindow>
#include "DisplayValue.h"
#include "ui_DisplayValue.h"

#include <husky_teleop/husky_teleop_telemetry_subscriber_node.h>

namespace Ui
{
class TelemetryMainWindow;
}

class TelemetryMainWindow : public QMainWindow
{
   Q_OBJECT

public:
   TelemetryMainWindow(int argc, char** argv, QWidget* parent = nullptr);
   ~TelemetryMainWindow();

public slots:
   void onVelocityCommanded(const geometry_msgs::Twist::ConstPtr& twist_msg);
   void onOdometryMsgReceived(const nav_msgs::Odometry::ConstPtr& odometry_msg);
   void onJoystickMsgReceived(const sensor_msgs::Joy::ConstPtr& joy_msg);

private:
   enum MainWindowRegion
   {
      Region_VehicleDetails,
      Region_Joystick,
      Region_Pose,
      Region_Twist
   };

   DisplayValue* createDisplayValueWidget(const QString&   field_name,
                                          const QString&   initial_value,
                                          MainWindowRegion region);

   void updateDoubleValue(DisplayValue* widget, double value);
   void updateBooleanValue(DisplayValue* widget, bool value);
   void updateStringValue(DisplayValue* widget, const std::string& value);

   QMap<bool, QString> mBooleanTextMap;

   int    mArgC;
   char** mArgV;

   TelemetrySubscriberNode* mTelemetrySubscriberNode;

   int mLinearAxisIndex;
   int mAngularAxisIndex;
   int mDeadmanButtonIndex;
   int mTurboButtonIndex;

   geometry_msgs::Twist mCurrentTwistMsg;
   sensor_msgs::Joy     mCurrentJoyMsg;
   nav_msgs::Odometry   mCurrentOdometryMsg;

   Ui::TelemetryMainWindow* mUi;

   DisplayValue* mDisplayTimeStamp;
   DisplayValue* mDisplayFrameId;

   DisplayValue* mDisplayLinearAxis;
   DisplayValue* mDisplayAngularAxis;
   DisplayValue* mDisplayDeadmanButton;
   DisplayValue* mDisplayTurboButton;

   DisplayValue* mDisplayPositionX;
   DisplayValue* mDisplayPositionY;
   DisplayValue* mDisplayPositionZ;

   DisplayValue* mDisplayOrientationX;
   DisplayValue* mDisplayOrientationY;
   DisplayValue* mDisplayOrientationZ;
   DisplayValue* mDisplayOrientationW;

   DisplayValue* mDisplayTwistLinearX;
   DisplayValue* mDisplayTwistLinearY;
   DisplayValue* mDisplayTwistLinearZ;

   DisplayValue* mDisplayTwistAngularX;
   DisplayValue* mDisplayTwistAngularY;
   DisplayValue* mDisplayTwistAngularZ;
};

#endif // TELEOPMAINWINDOW_H
