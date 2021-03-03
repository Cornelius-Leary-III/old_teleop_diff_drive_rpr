#ifndef DISPLAYVALUE_H
#define DISPLAYVALUE_H

#include <QWidget>

namespace Ui
{
class DisplayValue;
}

class DisplayValue : public QWidget
{
   Q_OBJECT

public:
   explicit DisplayValue(QWidget* parent = nullptr);
   ~DisplayValue();

private:
   Ui::DisplayValue* ui;
};

#endif // DISPLAYVALUE_H
