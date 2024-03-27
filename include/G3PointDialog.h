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

	void emitAllClicked(bool state){emit allClicked(state);}
	void emitOnlyOneClicked(bool state){emit onlyOneClicked(state);}
	void emitOnlyOneChanged(int idx){emit onlyOneChanged(idx);}
	void emitFit(){emit fit();}
	void emitTransparencyChanged(double transparency){emit transparencyChanged(transparency);}
	void emitDrawSurfaces(bool state){emit drawSurfaces(state);}
	void emitDrawLines(bool state){emit drawLines(state);}
	void emitDrawPoints(bool state){emit drawPoints(state);}
	void emitGLPointSizeChanged(int size){emit glPointSize(size);}

	void emitSignals();

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

	void setOnlyOneMax(int idx);
	void enableDrawPointsForOnlyOneGrain(bool state);

signals:
	void segment();
	void cluster();
	void segmentCluster();
	void clean();
	void segmentClusterClean();
	void getBorders();
	void fit();

	void allClicked(bool state);
	void onlyOneClicked(bool state);
	void onlyOneChanged(int idx);
	void transparencyChanged(double transparency);
	void drawSurfaces(bool state);
	void drawLines(bool state);
	void drawPoints(bool state);
	void glPointSize(int size);

private:
	Ui::qG3PointDialog *ui;
};

#endif // QG3POINTDIALOG_H
