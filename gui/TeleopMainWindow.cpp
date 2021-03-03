#include "TeleopMainWindow.h"
#include "ui_TeleopMainWindow.h"

TeleopMainWindow::TeleopMainWindow(QWidget* parent)
   : QMainWindow(parent),
     ui(new Ui::TeleopMainWindow)
{
   ui->setupUi(this);
}

TeleopMainWindow::~TeleopMainWindow()
{
   delete ui;
}
