#include "G3PointDialog.h"
#include "ui_qG3PointDialog.h"

G3PointDialog::G3PointDialog(QString cloudName, QWidget *parent)
	: QDialog(parent)
	, ui(new Ui::qG3PointDialog)
{
	ui->setupUi(this);

	setAttribute(Qt::WA_DeleteOnClose, true);
	setWindowFlag(Qt::WindowStaysOnTopHint, true);
	setWindowTitle("G3Point");
	ui->labelCloud->setText(cloudName);

	connect(this->ui->pushButtonSegment, &QPushButton::clicked, this, &G3PointDialog::emitSegment);
	connect(this->ui->pushButtonCluster, &QPushButton::clicked, this, &G3PointDialog::emitCluster);
	connect(this->ui->pushButtonSegmentCluster, &QPushButton::clicked, this, &G3PointDialog::emitSegmentCluster);
}

G3PointDialog::~G3PointDialog()
{
	delete ui;
}

double G3PointDialog::getMaxAngle1()
{
	return this->ui->doubleSpinBoxMaxAngle1->value();
}

int G3PointDialog::getkNN()
{
	return this->ui->spinBoxkNN->value();
}

int G3PointDialog::getPointSize()
{
	return this->ui->spinBoxPointSize->value();
}

double G3PointDialog::getRadiusFactor()
{
	return ui->doubleSpinBoxRadiusFactor->value();
}

bool G3PointDialog::isSteepestSlope()
{
	return this->ui->radioButtonSteepestSlope->isChecked();
}

void G3PointDialog::enableCluster(bool state)
{
	this->ui->pushButtonCluster->setEnabled(state);
}
