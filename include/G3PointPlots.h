#ifndef G3POINTPLOTS_H
#define G3POINTPLOTS_H

#include <QWidget>

#include <qcustomplot.h>

namespace Ui {
class G3PointPlots;
}

class G3PointPlots : public QWidget
{
	Q_OBJECT

public:
	explicit G3PointPlots(QString title, QWidget *parent = nullptr);
	~G3PointPlots();

	void readSettings();

	void addToTabWidget(QWidget* widget);

	template<typename SharedDataContainer>
	bool exportToCSV(QString filename, SharedDataContainer container) const;

	void onExportToCSV();

	void onExportToImage();

	void closeEvent(QCloseEvent *event) override;

private:
	Ui::G3PointPlots *ui;
};

#endif // G3POINTPLOTS_H
