#include "TelemetryMainWindow.h"
#include "ui_TelemetryMainWindow.h"

const int gDefaultSteeringAxisIndex     = 0;
const int gDefaultDeadmanPedalAxisIndex = 1;
const int gDefaultThrottleAxisIndex     = 2;
const int gDefaultBrakeAxisIndex        = 3;
// const int gDefaultForwardGearButtonIndex = 0;
// const int gDefaultReverseGearButtonIndex = 3;
const int gDefaultTurboButtonIndex = 1;

Q_DECLARE_METATYPE(geometry_msgs::Twist::ConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::Joy::ConstPtr);
Q_DECLARE_METATYPE(nav_msgs::Odometry::ConstPtr);

TelemetryMainWindow::TelemetryMainWindow(int argc, char** argv, QWidget* parent)
   : QMainWindow(parent),
     mArgC(argc),
     mArgV(argv),
     mTelemetrySubscriberNode(new TelemetrySubscriberNode(argc, argv)),
     mLinearAxisIndex(gDefaultThrottleAxisIndex),
     mBrakeAxisIndex(gDefaultBrakeAxisIndex),
     mAngularAxisIndex(gDefaultSteeringAxisIndex),
     mDeadmanAxisIndex(gDefaultDeadmanPedalAxisIndex),
     mTurboButtonIndex(gDefaultTurboButtonIndex),
     mCurrentTwistMsg(),
     mCurrentJoyMsg(),
     mCurrentOdometryMsg(),
     mUi(new Ui::TelemetryMainWindow),
     mDisplayTimeStamp(nullptr),
     mDisplayFrameId(nullptr),
     mDisplayLinearAxis(nullptr),
     mDisplayAngularAxis(nullptr),
     mDisplayDeadmanAxis(nullptr),
     mDisplayTurboButton(nullptr),
     mDisplayPositionX(nullptr),
     mDisplayPositionY(nullptr),
     mDisplayPositionZ(nullptr),
     mDisplayOrientationX(nullptr),
     mDisplayOrientationY(nullptr),
     mDisplayOrientationZ(nullptr),
     mDisplayOrientationW(nullptr),
     mDisplayTwistLinearX(nullptr),
     mDisplayTwistLinearY(nullptr),
     mDisplayTwistLinearZ(nullptr),
     mDisplayTwistAngularX(nullptr),
     mDisplayTwistAngularY(nullptr),
     mDisplayTwistAngularZ(nullptr)
{
   mUi->setupUi(this);

   connect(mTelemetrySubscriberNode,
           &TelemetrySubscriberNode::velocityCommanded,
           this,
           &TelemetryMainWindow::onVelocityCommanded,
           Qt::ConnectionType::QueuedConnection);

   connect(mTelemetrySubscriberNode,
           &TelemetrySubscriberNode::odometryMsgReceived,
           this,
           &TelemetryMainWindow::onOdometryMsgReceived,
           Qt::ConnectionType::QueuedConnection);

   connect(mTelemetrySubscriberNode,
           &TelemetrySubscriberNode::joystickMsgReceived,
           this,
           &TelemetryMainWindow::onJoystickMsgReceived,
           Qt::ConnectionType::QueuedConnection);

   mDisplayTimeStamp = createDisplayValueWidget("Current Time Stamp:",
                                                MainWindowRegion::Region_VehicleDetails,
                                                "<Time>");

   mDisplayFrameId = createDisplayValueWidget("Frame ID:", MainWindowRegion::Region_VehicleDetails);

   mDisplayLinearAxis = createDisplayValueWidget("Linear Axis:", MainWindowRegion::Region_Joystick);

   mDisplayBrakeAxis = createDisplayValueWidget("Brake Axis:", MainWindowRegion::Region_Joystick);

   mDisplayAngularAxis =
      createDisplayValueWidget("Angular Axis:", MainWindowRegion::Region_Joystick);

   mDisplayDeadmanAxis =
      createDisplayValueWidget("Deadman Axis:", MainWindowRegion::Region_Joystick);

   mDisplayTurboButton =
      createDisplayValueWidget("Turbo Button Pressed?", MainWindowRegion::Region_Joystick);

   mDisplayPositionX =
      createDisplayValueWidget("Current Position X:", MainWindowRegion::Region_Pose);

   mDisplayPositionY =
      createDisplayValueWidget("Current Position Y:", MainWindowRegion::Region_Pose);

   mDisplayPositionZ =
      createDisplayValueWidget("Current Position Z:", MainWindowRegion::Region_Pose);

   mDisplayOrientationX =
      createDisplayValueWidget("Current Orientation X:", MainWindowRegion::Region_Pose);

   mDisplayOrientationY =
      createDisplayValueWidget("Current Orientation Y:", MainWindowRegion::Region_Pose);

   mDisplayOrientationZ =
      createDisplayValueWidget("Current Orientation Z:", MainWindowRegion::Region_Pose);

   mDisplayOrientationW =
      createDisplayValueWidget("Current Orientation W:", MainWindowRegion::Region_Pose);

   mDisplayTwistLinearX =
      createDisplayValueWidget("Current Twist Linear X:", MainWindowRegion::Region_Twist);

   mDisplayTwistLinearY =
      createDisplayValueWidget("Current Twist Linear Y:", MainWindowRegion::Region_Twist);

   mDisplayTwistLinearZ =
      createDisplayValueWidget("Current Twist Linear Z:", MainWindowRegion::Region_Twist);

   mDisplayTwistAngularX =
      createDisplayValueWidget("Current Twist Angular X:", MainWindowRegion::Region_Twist);

   mDisplayTwistAngularY =
      createDisplayValueWidget("Current Twist Angular Y:", MainWindowRegion::Region_Twist);

   mDisplayTwistAngularZ =
      createDisplayValueWidget("Current Twist Angular Z:", MainWindowRegion::Region_Twist);

   mBooleanTextMap.insert(false, "FALSE");
   mBooleanTextMap.insert(true, "TRUE");

   mTelemetrySubscriberNode->start();
}

TelemetryMainWindow::~TelemetryMainWindow()
{
   if (mUi != nullptr)
   {
      delete mUi;
      mUi = nullptr;
   }

   if (mTelemetrySubscriberNode != nullptr)
   {
      mTelemetrySubscriberNode->quit();
      mTelemetrySubscriberNode->wait();
   }
}

void TelemetryMainWindow::onVelocityCommanded(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
   mCurrentTwistMsg = *twist_msg;

   if (twist_msg != nullptr)
   {
      updateDoubleValue(mDisplayTwistLinearX, twist_msg->linear.x);
      updateDoubleValue(mDisplayTwistLinearY, twist_msg->linear.y);
      updateDoubleValue(mDisplayTwistLinearZ, twist_msg->linear.z);

      updateDoubleValue(mDisplayTwistAngularX, twist_msg->angular.x);
      updateDoubleValue(mDisplayTwistAngularY, twist_msg->angular.y);
      updateDoubleValue(mDisplayTwistAngularZ, twist_msg->angular.z);
   }
}

void TelemetryMainWindow::onOdometryMsgReceived(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
   mCurrentOdometryMsg = *odometry_msg;

   if (odometry_msg != nullptr)
   {
      updateStringValue(mDisplayFrameId, odometry_msg->header.frame_id);
      updateDoubleValue(mDisplayTimeStamp, odometry_msg->header.stamp.toSec());

      updateDoubleValue(mDisplayPositionX, odometry_msg->pose.pose.position.x);
      updateDoubleValue(mDisplayPositionY, odometry_msg->pose.pose.position.y);
      updateDoubleValue(mDisplayPositionZ, odometry_msg->pose.pose.position.z);

      updateDoubleValue(mDisplayOrientationX, odometry_msg->pose.pose.orientation.x);
      updateDoubleValue(mDisplayOrientationY, odometry_msg->pose.pose.orientation.y);
      updateDoubleValue(mDisplayOrientationZ, odometry_msg->pose.pose.orientation.z);
      updateDoubleValue(mDisplayOrientationW, odometry_msg->pose.pose.orientation.w);
   }
}

void TelemetryMainWindow::onJoystickMsgReceived(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   if (joy_msg != nullptr)
   {
      updateJoyAxisValue(mDisplayLinearAxis, mLinearAxisIndex);
      updateJoyAxisValue(mDisplayAngularAxis, mAngularAxisIndex);
      updateJoyAxisValue(mDisplayBrakeAxis, mBrakeAxisIndex);
      updateJoyAxisValue(mDisplayDeadmanAxis, mDeadmanAxisIndex);
      updateJoyButtonValue(mDisplayTurboButton, mTurboButtonIndex);
   }
}

void TelemetryMainWindow::updateJoyAxisValue(DisplayValue* widget, int axis_index)
{
   if (isAxisIndexValid(axis_index))
   {
      updateDoubleValue(widget, mCurrentJoyMsg.axes.at(axis_index));
   }
}

void TelemetryMainWindow::updateJoyButtonValue(DisplayValue* widget, int button_index)
{
   if (isButtonIndexValid(button_index))
   {
      updateBooleanValue(widget, mCurrentJoyMsg.buttons.at(button_index));
   }
}

void TelemetryMainWindow::updateDoubleValue(DisplayValue* widget, double value)
{
   auto value_text = QString::number(value);

   updateField(widget, value_text);
}

void TelemetryMainWindow::updateBooleanValue(DisplayValue* widget, bool flag)
{
   QString value_text = mBooleanTextMap.value(flag, "FALSE");

   updateField(widget, value_text);
}

void TelemetryMainWindow::updateStringValue(DisplayValue* widget, const std::string& value)
{
   auto value_text = QString::fromStdString(value);

   updateField(widget, value_text);
}

void TelemetryMainWindow::updateField(DisplayValue* widget, const QString& field_value)
{
   if (widget != nullptr)
   {
      widget->setFieldValue(field_value);
   }
}

DisplayValue* TelemetryMainWindow::createDisplayValueWidget(const QString&   field_name,
                                                            MainWindowRegion region,
                                                            const QString&   initial_value)
{
   auto display_widget = new DisplayValue(nullptr);

   display_widget->setFieldName(field_name);
   display_widget->setFieldValue(initial_value);

   QLayout* layout = nullptr;

   switch (region)
   {
      case MainWindowRegion::Region_VehicleDetails:
      {
         layout = mUi->widgetVehicleDetails->layout();
         break;
      }
      case MainWindowRegion::Region_Joystick:
      {
         layout = mUi->widgetJoy->layout();
         break;
      }
      case MainWindowRegion::Region_Pose:
      {
         layout = mUi->widgetPose->layout();
         break;
      }
      case MainWindowRegion::Region_Twist:
      {
         layout = mUi->widgetTwist->layout();
         break;
      }
   }

   layout->addWidget(display_widget);

   return display_widget;
}

bool TelemetryMainWindow::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.axes.size();
}

bool TelemetryMainWindow::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.buttons.size();
}
