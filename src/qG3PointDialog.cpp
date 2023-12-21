#include "qG3PointDialog.h"
#include "ui_qG3PointDialog.h"

qG3PointDialog::qG3PointDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::qG3PointDialog)
{
	ui->setupUi(this);

	connect(this->ui->pushButtonSegment, &QPushButton::clicked, this, &qG3PointDialog::emitRun);
}

qG3PointDialog::~qG3PointDialog()
{
	delete ui;
}

void qG3PointDialog::emitRun()
{
	emit run();
}

int qG3PointDialog::getkNN()
{
	return this->ui->spinBoxkNN->value();
}
