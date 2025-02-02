#ifndef WOLMANCUSTOMPLOT_H
#define WOLMANCUSTOMPLOT_H

#include <QWidget>

#include <qcustomplot.h>

namespace Ui {
class WolmanCustomPlot;
}

class WolmanCustomPlot : public QCustomPlot
{
	Q_OBJECT

public:
	explicit WolmanCustomPlot(QWidget *parent = nullptr);
	~WolmanCustomPlot();

private:
	Ui::WolmanCustomPlot *ui;
};

#endif // WOLMANCUSTOMPLOT_H
