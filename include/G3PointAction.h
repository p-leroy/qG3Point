#include "Eigen/Dense"

#include <ccOctree.h>
#include <ccScalarField.h>

#include <vector>

#include <QObject>

#include <G3PointDialog.h>

#include <GrainsAsEllipsoids.h>

#include <AnglesCustomPlot.h>

#include <G3PointPlots.h>

#pragma once

class ccMainAppInterface;
class ccPointCloud;

namespace G3Point
{
class G3PointAction : public QObject
{
	Q_OBJECT

	typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> XXb;
	typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> Xb;

public:
	explicit G3PointAction(ccPointCloud *cloud, ccMainAppInterface *app=nullptr);
	~G3PointAction();
	static void createAction(ccMainAppInterface *appInterface);
	static void GetG3PointAction(ccPointCloud *cloud, ccMainAppInterface *app=nullptr);
	void segment();
	void clusterAndOrClean();
	void getBorders();
	bool cluster();
	void fit();
	void exportResults();
	void plots();
	void showWolman(const Eigen::ArrayXf& d_sample);
	bool wolman();
	bool angles();
	bool processNewStacks(std::vector<std::vector<int>>& newStacks, int pointCount);
	bool buildStacksFromG3PointLabelSF(CCCoreLib::ScalarField *g3PointLabel);
	bool merge(XXb& condition);
	bool keep(Xb& condition);
	bool cleanLabels();
	void clean();

	template<typename T> static bool EigenArrayToFile(QString name, T array);

private:
	bool sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent);
	void addToStack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
	int segmentLabels(bool useParallelStrategy=true);
	double angleRot2VecMat(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
	Eigen::ArrayXXd computeMeanAngleBetweenNormalsAtBorders();
	bool exportLocalMaximaAsCloud(const Eigen::ArrayXi &localMaximumIndexes);
	bool updateLocalMaximumIndexes();
	bool updateLabelsAndColors();
	bool checkStacks(const std::vector<std::vector<int>>& stacks, int count);
	void addToStackBraunWillett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi &Di, std::vector<int>& stack, int local_maximum);
	int segmentLabelsBraunWillett(bool useParallelStrategy=true);
	void getNeighborsDistancesSlopes(unsigned index);
	void computeNodeSurfaces();
	bool computeNormalsAndOrientThemWithCloudCompare();
	void orientNormals(const Eigen::Vector3d &sensorCenter);
	bool computeNormalsWithOpen3D();
	bool queryNeighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
	void init();
	void showDlg();
	void resetDlg();
	bool setCloud(ccPointCloud *cloud);
	void setKNN();

	int m_kNN = 20;
	double m_radiusFactor = 0.6;
	double m_maxAngle1 = 60;
	double m_maxAngle2 = 10;
	int m_nMin = 50;
	double m_minFlatness = 0.1;

	ccPointCloud* m_cloud;
	ccMainAppInterface *m_app;
	G3PointDialog* m_dlg;
	
	Eigen::ArrayXXi m_neighborsIndexes;
	Eigen::ArrayXXd m_neighborsDistances;
	Eigen::ArrayXXd m_neighborsSlopes;
	Eigen::ArrayXXd m_normals;

	Eigen::ArrayXi m_initial_labels;
	Eigen::ArrayXi m_initial_labelsnpoint;
	Eigen::ArrayXi m_initial_localMaximumIndexes;
	Eigen::ArrayXi m_labels;
	Eigen::ArrayXi m_labelsnpoint;
	Eigen::ArrayXi m_localMaximumIndexes;
	Eigen::ArrayXi m_ndon;
	Eigen::ArrayXd m_area;

	QSharedPointer<RGBAColorsTableType> m_grainColors;

	std::vector<std::vector<int>> m_initialStacks;
	std::vector<std::vector<int>> m_stacks;

	ccOctree::Shared m_octree;
	unsigned char m_bestOctreeLevel = 0;
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct m_nNSS;

	static std::shared_ptr<G3PointAction> s_g3PointAction;

	static QPointer<G3PointPlots> s_g3PointPlots;

	GrainsAsEllipsoids* m_grainsAsEllipsoids;

	std::unique_ptr<QProgressBar> m_progress;

	int m_currentNumberOfSteps;
};
}
