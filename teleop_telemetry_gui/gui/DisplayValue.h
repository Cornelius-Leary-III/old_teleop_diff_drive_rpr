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

   void setFieldName(const QString& name);
   void setFieldValue(const QString& value);

private:
   Ui::DisplayValue* mUi;

   QString mFieldName;
   QString mFieldValue;
};

#endif // DISPLAYVALUE_H
