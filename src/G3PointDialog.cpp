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
	connect(this->ui->pushButtonClusterAndOrClean, &QPushButton::clicked, this, &G3PointDialog::emitClusterAndOrClean);
	connect(this->ui->pushButtonGetBorders, &QPushButton::clicked, this, &G3PointDialog::emitGetBorders);

	connect(this->ui->radioButtonAll, &QRadioButton::clicked, this, &G3PointDialog::emitAllClicked);
	connect(this->ui->radioButtonOnlyOne, &QRadioButton::clicked, this, &::G3PointDialog::emitOnlyOneClicked);
	connect(this->ui->spinBoxOnlyOne, qOverload<int>(&QSpinBox::valueChanged), this, &::G3PointDialog::emitOnlyOneChanged);

	connect(this->ui->pushButtonFit, &QPushButton::clicked, this, &G3PointDialog::emitFit);
	connect(this->ui->checkBoxSurfaces, &QCheckBox::clicked, this, &G3PointDialog::emitDrawSurfaces);
	connect(this->ui->checkBoxWireframes, &QCheckBox::clicked, this, &::G3PointDialog::emitDrawLines);
	connect(this->ui->checkBoxPoints, &QCheckBox::clicked, this, &G3PointDialog::emitDrawPoints);
	connect(this->ui->spinBoxGLPointSize, qOverload<int>(&QSpinBox::valueChanged), this, &::G3PointDialog::emitGLPointSizeChanged);

	connect(this->ui->radioButtonOnlyOne, &QRadioButton::toggled, this, &G3PointDialog::enableDrawPointsForOnlyOneGrain);
}

G3PointDialog::~G3PointDialog()
{
	delete ui;
}

double G3PointDialog::getMaxAngle1()
{
	return this->ui->doubleSpinBoxMaxAngle1->value();
}

double G3PointDialog::getMaxAngle2()
{
	return this->ui->doubleSpinBoxMaxAngle2->value();
}

double G3PointDialog::getMinFlatness()
{
	return this->ui->doubleSpinBoxMinFlatness->value();
}

int G3PointDialog::getNMin()
{
	return this->ui->spinBoxNMin->value();
}

int G3PointDialog::getkNN()
{
	return this->ui->spinBoxkNN->value();
}

double G3PointDialog::getRadiusFactor()
{
	return ui->doubleSpinBoxRadiusFactor->value();
}

bool G3PointDialog::isSteepestSlope()
{
	return this->ui->radioButtonSteepestSlope->isChecked();
}

void G3PointDialog::enableClusterAndOrClean(bool state)
{
	this->ui->groupBoxClusterAndOrClean->setEnabled(state);
}

bool G3PointDialog::clusterIsChecked()
{
	return this->ui->checkBoxClustering->isChecked();
}

bool G3PointDialog::cleanIsChecked()
{
	return this->ui->checkBoxCleaning->isChecked();
}

void G3PointDialog::emitSignals()
{
	emit allClicked(this->ui->radioButtonAll->isChecked());
	emit onlyOneChanged(this->ui->spinBoxOnlyOne->value());
}

void G3PointDialog::setOnlyOneMax(int max)
{
	this->ui->spinBoxOnlyOne->setMaximum(max);
}

void G3PointDialog::enableDrawPointsForOnlyOneGrain(bool state)
{
	this->ui->checkBoxPoints->setEnabled(state);
	this->ui->spinBoxGLPointSize->setEnabled(state);
}
