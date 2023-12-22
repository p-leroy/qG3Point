#include "G3PointDialog.h"
#include "ui_qG3PointDialog.h"

G3PointDialog::G3PointDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::qG3PointDialog)
{
	ui->setupUi(this);

	connect(this->ui->pushButtonSegment, &QPushButton::clicked, this, &G3PointDialog::emitRun);
}

G3PointDialog::~G3PointDialog()
{
	delete ui;
}

void G3PointDialog::emitRun()
{
	emit run();
}

int G3PointDialog::getkNN()
{
	return this->ui->spinBoxkNN->value();
}

bool G3PointDialog::isSteepestSlope()
{
	return this->ui->radioButtonSteepestSlope->isChecked();
}
