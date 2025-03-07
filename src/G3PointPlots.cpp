#include "G3PointPlots.h"
#include "ui_G3PointPlots.h"

// CCPluginAPI
#include <ccPersistentSettings.h>

/// qCC_db
#include <ccFileUtils.h>

// qcc_io
#include <ImageFileFilter.h>

#include <QTextStream>

#include <QSettings>

#include <AnglesCustomPlot.h>

#include <WolmanCustomPlot.h>

G3PointPlots::G3PointPlots(QString title, QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::G3PointPlots)
{
	ui->setupUi(this);

	readSettings();

	setWindowTitle(title);

	connect(this->ui->exportCSVToolButton, &QToolButton::clicked, this, &G3PointPlots::onExportToCSV);
	connect(this->ui->exportImage, &QToolButton::clicked, this, &G3PointPlots::onExportToImage);
}

G3PointPlots::~G3PointPlots()
{
	delete ui;
}

void G3PointPlots::closeEvent(QCloseEvent *event)
{
	QSettings settings("OSERen", "qG3Point");
	settings.setValue("G3PointPlots/geometry", saveGeometry());
	// settings.setValue("windowState", saveState());
	QWidget::closeEvent(event);
}

void G3PointPlots::readSettings()
{
	QSettings settings("OSERen", "qG3Point");
	restoreGeometry(settings.value("G3PointPlots/geometry").toByteArray());
}

void G3PointPlots::addToTabWidget(QWidget* widget)
{
	this->ui->tabWidget->addTab(widget, widget->windowTitle());
	this->ui->tabWidget->setCurrentWidget(widget);

	if (widget->property("TypeOfCustomPlot").toString() == "AnglesCustomPlot")
	{
		connect(static_cast<AnglesCustomPlot*>(widget), &AnglesCustomPlot::closeTab, this, &G3PointPlots::closeCurrentWidget);
	}
	else if (widget->property("TypeOfCustomPlot").toString() == "WolmanCustomPlot")
	{
		connect(static_cast<WolmanCustomPlot*>(widget), &WolmanCustomPlot::closeTab, this, &G3PointPlots::closeCurrentWidget);
	}
}

void G3PointPlots::closeCurrentWidget()
{
	this->ui->tabWidget->currentWidget()->deleteLater();
}

template<typename SharedDataContainer>
bool G3PointPlots::exportToCSV(QString filename, SharedDataContainer container, const Eigen::Array3d* dq_final, const Eigen::Array3d* edq) const
{
	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		ccLog::Warning(QString("[SF/SF] Failed to save plot to file '%1'").arg(filename));
		return false;
	}

	QTextStream stream(&file);
	stream.setRealNumberPrecision(12);
	stream.setRealNumberNotation(QTextStream::FixedNotation);

	//header
	if (dq_final) // WolmanCustomPlot
	{
		stream << "# D10 [mm], D50 [mm], D90 [mm]" << endl;
		stream << (*dq_final)(0) << (*dq_final)(0) << (*dq_final)(0) << endl;
		stream << "# std(D10) [mm], std(D50) [mm], std(D90) [mm]" << endl;
		stream << (*edq)(0) << (*edq)(0) << (*edq)(0) << endl;
		stream << "diameter [m], pdf" << endl;
	}
	else // AnglesCustomPlot
	{
		stream << "angle [degree], counts" << endl;
	}

	//data
	{
		for (auto item : *container)
		{
			stream << item.key << " " << item.value << endl;
		}
	}

	file.close();

	ccLog::Print(QString("[SF/SF] File '%1' saved").arg(filename));

	return true;
}

typedef QSharedPointer<QCPBarsDataContainer> SharedBarsDataContainer;
typedef QSharedPointer<QCPGraphDataContainer> SharedGraphDataContainer;

void G3PointPlots::onExportToCSV()
{
	// get current tab widget
	QWidget* currentWidget = this->ui->tabWidget->currentWidget();

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	currentPath += QString("/") + currentWidget->windowTitle() + ".csv";

	//ask for a filename
	QString filename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, "*.csv");
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(filename).absolutePath());
	settings.endGroup();

	if (currentWidget->property("TypeOfCustomPlot").toString() == "AnglesCustomPlot")
	{
		Eigen::Array3d dq_final;
		Eigen::Array3d edq;
		exportToCSV<SharedBarsDataContainer>(filename,
											 static_cast<AnglesCustomPlot*>(currentWidget)->dataContainer());
	}
	else if (currentWidget->property("TypeOfCustomPlot").toString() == "WolmanCustomPlot")
	{
		WolmanCustomPlot* wolmanCustomPlot = static_cast<WolmanCustomPlot*>(currentWidget);
		exportToCSV<SharedGraphDataContainer>(filename,
											  wolmanCustomPlot->dataContainer(),
											  &wolmanCustomPlot->m_dq_final,
											  &wolmanCustomPlot->m_edq);
	}
}

void G3PointPlots::onExportToImage()
{
	QWidget* currentWidget = this->ui->tabWidget->currentWidget();

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = ImageFileFilter::GetSaveFilename("Select output file",
															  currentWidget->windowTitle(),
															  currentPath,
															  this);

	if (outputFilename.isEmpty())
	{
		//process cancelled by user (or error)
		return;
	}

	//save current export path to persistent settings
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//save the widget as an image file
	QPixmap image = currentWidget->grab();
	if (image.save(outputFilename))
	{
		ccLog::Print(QString("[SF/SF] Image '%1' successfully saved").arg(outputFilename));
	}
	else
	{
		ccLog::Error(QString("Failed to save file '%1'").arg(outputFilename));
	}
}

