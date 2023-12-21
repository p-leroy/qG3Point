#ifndef QG3POINTDIALOG_H
#define QG3POINTDIALOG_H

#include <QDialog>

namespace Ui {
class qG3PointDialog;
}

class qG3PointDialog : public QDialog
{
	Q_OBJECT

public:
	explicit qG3PointDialog(QWidget *parent = nullptr);
	~qG3PointDialog();
	void emitRun();
	int getkNN();

signals:
	void run();

private:
	Ui::qG3PointDialog *ui;
};

#endif // QG3POINTDIALOG_H
