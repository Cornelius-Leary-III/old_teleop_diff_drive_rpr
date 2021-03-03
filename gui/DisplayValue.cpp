#include "DisplayValue.h"
#include "ui_DisplayValue.h"

DisplayValue::DisplayValue(QWidget* parent) : QWidget(parent), ui(new Ui::DisplayValue)
{
   ui->setupUi(this);
}

DisplayValue::~DisplayValue()
{
   delete ui;
}
