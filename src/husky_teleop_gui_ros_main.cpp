#include "gui/TeleopMainWindow.h"

#include <QApplication>

int main(int argc, char** argv)
{
   QApplication husky_teleop_gui_app(argc, argv);

   TeleopMainWindow teleop_gui_main_window(argc, argv);
   teleop_gui_main_window.show();

   husky_teleop_gui_app.exec();

   return 0;
}
