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
	double getMaxAngle1();
	int getkNN();
	int getPointSize();
	double getRadiusFactor();
	bool isSteepestSlope();
	void enableCluster(bool state);

signals:
	void segment();
	void cluster();
	void segmentCluster();

private:
	Ui::qG3PointDialog *ui;
};

#endif // QG3POINTDIALOG_H
