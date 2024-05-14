#ifndef QG3POINTDISCLAIMER_H
#define QG3POINTDISCLAIMER_H

#include <QDialog>

class ccMainAppInterface;

namespace Ui {
class G3PointDisclaimer;
}

class qG3PointDisclaimer : public QDialog
{
	Q_OBJECT

public:
	explicit qG3PointDisclaimer(QWidget *parent = nullptr);
	~qG3PointDisclaimer();

	static bool show(ccMainAppInterface* app);

private:
	//whether disclaimer has already been displayed (and accepted) or not
	static bool s_disclaimerAccepted;

	Ui::G3PointDisclaimer *ui;
};

#endif // QG3POINTDISCLAIMER_H
