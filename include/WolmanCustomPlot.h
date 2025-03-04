#ifndef WOLMANCUSTOMPLOT_H
#define WOLMANCUSTOMPLOT_H

#include <qcustomplot.h>

// Eigen
#include <Eigen/Geometry>

namespace Ui {
class WolmanCustomPlot;
}

//! QCustomPlot: vertical bar with text along side
class QCPBarsWithText : public QCPBars
{
	Q_OBJECT

public:

	QCPBarsWithText(QCPAxis* keyAxis, QCPAxis* valueAxis) : QCPBars(keyAxis,valueAxis), m_textOnTheLeft(false) {}

	void setText(QString text) { m_text = QStringList(text); }
	void appendText(QString text) { m_text.append(text); }
	void setTextAlignment(bool left) { m_textOnTheLeft = left; }

protected:

	QStringList m_text;
	bool m_textOnTheLeft;

	// reimplemented virtual draw method
	virtual void draw(QCPPainter *painter)
	{
		if (!mKeyAxis || !mValueAxis) { qDebug() << Q_FUNC_INFO << "invalid key or value axis"; return; }

		//switch to standard display
		QCPBars::draw(painter);

		int fontHeight = painter->fontMetrics().height();

		if (!data()->isEmpty())
		{
			double& key = data()->begin()->key;
			double& value = data()->begin()->value;
			QPointF P = coordsToPixels(key, value);
			//apply a small shift
			int margin = 5; //in pixels
			if (m_textOnTheLeft)
				margin = -margin;
			P.setX(P.x() + margin);
			//we draw at the 'base' line
			P.setY(P.y() + fontHeight);

			for (int i=0; i<m_text.size(); ++i)
			{
				QPointF Pstart = P;
				if (m_textOnTheLeft)
					Pstart.setX(P.x() - painter->fontMetrics().width(m_text[i]));
				painter->drawText(Pstart,m_text[i]);
				P.setY(P.y() + fontHeight);
			}
		}
	}

};

class WolmanCustomPlot : public QCustomPlot
{
	Q_OBJECT
public:
	WolmanCustomPlot(const Eigen::ArrayXf& d_sample);

	QSharedPointer<QCPGraphDataContainer> dataContainer(){return m_graph->data();}

	QCPGraph* m_graph;

	void emitCloseTab(){emit closeTab();}

	void mousePressEvent(QMouseEvent* event) override;

signals:
	void closeTab();

private:
	Ui::WolmanCustomPlot *ui;
};

#endif // WOLMANCUSTOMPLOT_H
