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
	explicit G3PointDialog(QWidget *parent = nullptr);
	~G3PointDialog();
	void emitRun();
	void emitCluster();
	int getkNN();
	bool isSteepestSlope();
	void enableCluster(bool state);

signals:
	void run();
	void cluster();

private:
	Ui::qG3PointDialog *ui;
};

#endif // QG3POINTDIALOG_H
