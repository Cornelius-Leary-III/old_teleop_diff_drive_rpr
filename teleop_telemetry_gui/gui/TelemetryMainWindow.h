#ifndef TELEOPMAINWINDOW_H
#define TELEOPMAINWINDOW_H

#include <string>
#include <QMainWindow>
#include "DisplayValue.h"
#include "ui_DisplayValue.h"

#include <teleop_telemetry_gui/teleop_telemetry_subscriber_node.h>

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
                                          MainWindowRegion region,
                                          const QString&   initial_value = "<Unknown>");

   void updateDoubleValue(DisplayValue* widget, double value);
   void updateBooleanValue(DisplayValue* widget, bool value);
   void updateStringValue(DisplayValue* widget, const std::string& value);
   void updateField(DisplayValue* widget, const QString& field_value);

   void updateJoyAxisValue(DisplayValue* widget, int axis_index);
   void updateJoyButtonValue(DisplayValue* widget, int button_index);

   bool isAxisIndexValid(int index);
   bool isButtonIndexValid(int index);

   QMap<bool, QString> mBooleanTextMap;

   int    mArgC;
   char** mArgV;

   TelemetrySubscriberNode* mTelemetrySubscriberNode;

   int mLinearAxisIndex;
   int mBrakeAxisIndex;
   int mAngularAxisIndex;
   int mDeadmanAxisIndex;
   int mTurboButtonIndex;

   geometry_msgs::Twist mCurrentTwistMsg;
   sensor_msgs::Joy     mCurrentJoyMsg;
   nav_msgs::Odometry   mCurrentOdometryMsg;

   Ui::TelemetryMainWindow* mUi;

   DisplayValue* mDisplayTimeStamp;
   DisplayValue* mDisplayFrameId;

   DisplayValue* mDisplayLinearAxis;
   DisplayValue* mDisplayBrakeAxis;
   DisplayValue* mDisplayAngularAxis;
   DisplayValue* mDisplayDeadmanAxis;
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
