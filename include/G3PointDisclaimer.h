#ifndef G3POINTDISCLAIMER_H
#define G3POINTDISCLAIMER_H

#include <QDialog>

class ccMainAppInterface;

namespace Ui {
class G3PointDisclaimer;
}

class G3PointDisclaimer : public QDialog
{
	Q_OBJECT

public:
	explicit G3PointDisclaimer(QWidget *parent = nullptr);
	~G3PointDisclaimer();

	static bool show(ccMainAppInterface* app);

private:
	//whether disclaimer has already been displayed (and accepted) or not
	static bool s_disclaimerAccepted;

	Ui::G3PointDisclaimer *ui;
};

#endif // G3POINTDISCLAIMER_H
