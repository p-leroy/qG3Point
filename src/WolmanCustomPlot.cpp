#include "WolmanCustomPlot.h"
#include "ui_WolmanCustomPlot.h"

WolmanCustomPlot::WolmanCustomPlot(QWidget *parent):
	ui(new Ui::WolmanCustomPlot)
{
	ui->setupUi(this);
}

WolmanCustomPlot::~WolmanCustomPlot()
{
	delete ui;
}
