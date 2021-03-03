#ifndef TELEOPMAINWINDOW_H
#define TELEOPMAINWINDOW_H

#include <QMainWindow>

namespace Ui
{
class TeleopMainWindow;
}

class TeleopMainWindow : public QMainWindow
{
   Q_OBJECT

public:
   explicit TeleopMainWindow(QWidget* parent = nullptr);
   ~TeleopMainWindow();

private:
   Ui::TeleopMainWindow* ui;
};

#endif // TELEOPMAINWINDOW_H
