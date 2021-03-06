#include "DisplayValue.h"
#include "ui_DisplayValue.h"

DisplayValue::DisplayValue(QWidget* parent) : QWidget(parent), mUi(new Ui::DisplayValue)
{
   mUi->setupUi(this);
}

DisplayValue::~DisplayValue()
{
   delete mUi;
}

void DisplayValue::setFieldName(const QString& name)
{
   mFieldName = name;

   mUi->labelName->setText(mFieldName);
}

void DisplayValue::setFieldValue(const QString& value)
{
   mFieldValue = value;

   mUi->labelValue->setText(mFieldValue);
}
