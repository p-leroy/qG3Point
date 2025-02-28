#ifndef ANGLESCUSTOMPLOT_H
#define ANGLESCUSTOMPLOT_H

#include <QWidget>

#include <qcustomplot.h>.h>

namespace Ui {
class AnglesCustomPlot;
}

class AnglesCustomPlot : public QCustomPlot
{

public:
	explicit AnglesCustomPlot(const QVector<double>& data, const QString& xLabel, int nbBins,
							  QWidget *parent = nullptr);
	~AnglesCustomPlot();

	bool computeHistogram(const QVector<double>& data);

	void createQCPBars(const QString &xLabel);

	void rescale();

	void mouseDoubleClickEvent(QMouseEvent *event) override;

	QSharedPointer<QCPBarsDataContainer> dataContainer(){return m_bars->data();}

private:
	Ui::AnglesCustomPlot *ui;

	QVector<double> m_axis;

	QVector<double> m_histogram;

	QString m_label;

	QCPBars* m_bars;

	int m_nbBins;
};

#endif // ANGLESCUSTOMPLOT_H
