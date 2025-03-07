#include "WolmanCustomPlot.h"
#include "ui_WolmanCustomPlot.h"

WolmanCustomPlot::WolmanCustomPlot(const Eigen::ArrayXf &d_sample, const Eigen::Array3d& dq_final, const Eigen::Array3d& edq):
	m_dq_final(dq_final),
	m_edq(edq),
	ui(new Ui::WolmanCustomPlot)
{
	setProperty("TypeOfCustomPlot", "WolmanCustomPlot");

	ui->setupUi(this);

	setWindowTitle("Wolman");

	QPen pen;

	m_graph = this->addGraph();
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
	this->xAxis->setScaleType(QCPAxis::stLogarithmic);
	this->xAxis->setLabel("Diameter [mm]");
	this->yAxis->setLabel("CDF");

	// add error bars
	QCPGraph* errorBarsGraph = this->addGraph();
	errorBarsGraph->setData(QVector<double>({dq_final[0], dq_final[1], dq_final[2]}),
							QVector<double>({0.1, 0.5, 0.9}));
	errorBarsGraph->setLineStyle(QCPGraph::lsNone);
	errorBarsGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
	QCPErrorBars *errorBars = new QCPErrorBars(xAxis, yAxis);
	errorBars->removeFromLegend();
	errorBars->setAntialiased(false);
	errorBars->setDataPlottable(errorBarsGraph);
	errorBars->setErrorType(QCPErrorBars::etKeyError);
	// pen.setWidth(1);
	errorBars->setPen(pen);
	errorBars->setData(QVector<double>({edq[0], edq[1], edq[2]}));
	errorBars->setPen(QPen(QColorConstants::Red));
	errorBars->rescaleAxes(true);

	setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

void WolmanCustomPlot::rescale()
{
	// set ranges appropriate to show data
	if (m_graph)
	{
		m_graph->rescaleAxes();
		replot();
	}
}

void WolmanCustomPlot::mouseDoubleClickEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		rescale();
	}
	QCustomPlot::mouseDoubleClickEvent(event);
}

void WolmanCustomPlot::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::RightButton)
	{
		QMenu* menu = new QMenu(this);
		QAction* action = new QAction("Close tab");
		menu->addAction(action);
		connect(action, &QAction::triggered, this, &WolmanCustomPlot::emitCloseTab);
		menu->popup(event->globalPos());
	}
	QCustomPlot::mousePressEvent(event);
}
