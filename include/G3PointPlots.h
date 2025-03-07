#ifndef G3POINTPLOTS_H
#define G3POINTPLOTS_H

#include <QWidget>

#include <qcustomplot.h>

// Eigen
#include <Eigen/Geometry>

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

	void closeCurrentWidget();

	template<typename SharedDataContainer>
	bool exportToCSV(QString filename, SharedDataContainer container, const Eigen::Array3d* dq_final=nullptr, const Eigen::Array3d* edq=nullptr) const;

	void onExportToCSV();

	void onExportToImage();

	void closeEvent(QCloseEvent *event) override;

private:
	Ui::G3PointPlots *ui;
};

#endif // G3POINTPLOTS_H
