#ifndef TELEOPMAINWINDOW_H
#define TELEOPMAINWINDOW_H

#include <QMainWindow>
#include "DisplayValue.h"
#include "ui_DisplayValue.h"

#include <husky_teleop/husky_teleoperation_gui.h>

namespace Ui
{
class TeleopMainWindow;
}

class TeleopMainWindow : public QMainWindow
{
   Q_OBJECT

public:
   TeleopMainWindow(int argc, char** argv, QWidget* parent = nullptr);
   ~TeleopMainWindow();

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

   QMap<bool, QString> mBooleanTextMap;

   int    mArgC;
   char** mArgV;

   TeleopNodeGui* mGuiNode;

   int mLinearAxisIndex;
   int mAngularAxisIndex;
   int mDeadmanButtonIndex;
   int mTurboButtonIndex;

   geometry_msgs::Twist mCurrentTwistMsg;
   sensor_msgs::Joy     mCurrentJoyMsg;
   nav_msgs::Odometry   mCurrentOdometryMsg;

   Ui::TeleopMainWindow* mUi;

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
