#include "Eigen/Dense"

#include <ccOctree.h>

#include <vector>

#include <QObject>

#include <G3PointDialog.h>

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
	void segmentAndCluster();
	void segmentAndClusterAndClean();
	void getBorders();
	int cluster();
	bool processNewStacks(std::vector<std::vector<int>>& stacks, int pointCount);
	bool merge(XXb& condition);
	bool keep(Xb& condition);
	bool cleanLabels();
	void clean();

private:
	bool sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent);
	void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
	int segment_labels(bool useParallelStrategy=true);
	double angle_rot_2_vec_mat(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
	Eigen::ArrayXXd computeMeanAngleBetweenNormalsAtBorders();
	bool exportLocalMaximaAsCloud();
	bool updateLocalMaximumIndexes();
	bool updateLabelsAndColors();
	bool checkStacks(const std::vector<std::vector<int>>& stacks, int count);
	int segment_labels_steepest_slope(bool useParallelStrategy=true);
	void add_to_stack_braun_willett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi &Di, std::vector<int>& stack, int local_maximum);
	int segment_labels_braun_willett(bool useParallelStrategy=true);
	void get_neighbors_distances_slopes(unsigned index);
	void compute_node_surfaces();
	bool compute_normals_and_orient_them_cloudcompare();
	void orient_normals(const Eigen::Vector3d &sensorCenter);
	bool compute_normals_with_open3d();
	bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
	void init();
	void showDlg();
	void resetDlg();
	void setCloud(ccPointCloud *cloud);

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
	Eigen::ArrayXXd m_neighbors_distances;
	Eigen::ArrayXXd m_neighbors_slopes;
	Eigen::ArrayXXd m_normals;

	Eigen::ArrayXi m_labels;
	Eigen::ArrayXi m_labelsnpoint;
	Eigen::ArrayXi m_localMaximumIndexes;
	Eigen::ArrayXi m_ndon;
	Eigen::ArrayXd m_area;

	std::vector<std::vector<int>> m_stacks;

	ccOctree::Shared m_octree;
	unsigned char m_bestOctreeLevel = 0;
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct m_nNSS;

	static G3PointAction* s_g3PointAction;
};
}
