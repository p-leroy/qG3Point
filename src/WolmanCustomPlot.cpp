#include "WolmanCustomPlot.h"
#include "ui_WolmanCustomPlot.h"

WolmanCustomPlot::WolmanCustomPlot(const Eigen::ArrayXf &d_sample):
	ui(new Ui::WolmanCustomPlot)
{
	setProperty("TypeOfCustomPlot", "WolmanCustomPlot");

	ui->setupUi(this);

	setWindowTitle("Wolman");

	m_graph = this->ui->customPlot->addGraph();
	QVector<double> x_data(d_sample.size());
	QVector<double> y_data(d_sample.size());
	for (int k = 0; k < d_sample.size(); k++)
	{
		x_data[k] = d_sample(k);
		y_data[k] = (static_cast<double>(k)) / static_cast<double>(d_sample.size());
	}
	std::sort(x_data.begin(), x_data.end());
	m_graph->setData(x_data, y_data);
	m_graph->rescaleAxes();
	// give the axes some labels:
	this->ui->customPlot->xAxis->setScaleType(QCPAxis::stLogarithmic);
	this->ui->customPlot->xAxis->setLabel("Diameter [mm]");
	this->ui->customPlot->yAxis->setLabel("CDF");
}
