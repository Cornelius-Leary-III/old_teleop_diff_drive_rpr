#include "gui/TelemetryMainWindow.h"

#include <QApplication>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   QApplication teleop_gui_app(argc, argv);

   TelemetryMainWindow telemetry_gui_main_window(argc, argv);
   telemetry_gui_main_window.show();

   teleop_gui_app.exec();

   return 0;
}
