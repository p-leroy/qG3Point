#ifndef QG3POINTDIALOG_H
#define QG3POINTDIALOG_H

#include <QDialog>

namespace Ui {
class qG3PointDialog;
}

class G3PointDialog : public QDialog
{
	Q_OBJECT

public:
	explicit G3PointDialog(QString cloudName, QWidget *parent = nullptr);
	~G3PointDialog();

	void emitSegment(){emit segment();}
	void emitCluster(){emit cluster();}
	void emitSegmentCluster(){emit segmentCluster();}
	void emitClean(){emit clean();}
	void emitSegmentClusterClean(){emit segmentClusterClean();}
	void emitGetBorders(){emit getBorders();}

	double getMaxAngle1();
	double getMaxAngle2();
	double getMinFlatness();
	int getNMin();
	int getkNN();
	int getPointSize();
	double getRadiusFactor();

	bool isSteepestSlope();
	void enableCluster(bool state);
	void enableClean(bool state);

signals:
	void segment();
	void cluster();
	void segmentCluster();
	void clean();
	void segmentClusterClean();
	void getBorders();

private:
	Ui::qG3PointDialog *ui;
};

#endif // QG3POINTDIALOG_H
