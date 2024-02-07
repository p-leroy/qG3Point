#include "Eigen/Dense"

#include <ccOctree.h>

#include <vector>

#include <QObject>

#include <qG3PointDialog.h>

#pragma once

class ccMainAppInterface;
class ccPointCloud;

namespace G3Point
{
class G3PointAction : public QObject
{
	Q_OBJECT

public:
	static void createAction(ccMainAppInterface *appInterface);

private:
	bool sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent);
	void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
	int segment_labels(bool useParallelStrategy=true);
	int compute_mean_angle();
	int cluster_labels();
	int segment_labels_steepest_slope(bool useParallelStrategy=true);
	void add_to_stack_braun_willett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi &Di, std::vector<int>& stack, int local_maximum);
	int segment_labels_braun_willett(bool useParallelStrategy=true);
	void get_neighbors_distances_slopes(unsigned index);
	void compute_node_surfaces();
	void orient_normals();
	void compute_normals_and_orient_them();
	bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
	void run();
	void setkNN(int kNN);

	int m_kNN = 20;
	double rad_factor = 0.6;

	Eigen::ArrayXXi m_neighbors_indexes;
	Eigen::ArrayXXd m_neighbors_distances;
	Eigen::ArrayXXd m_neighbors_slopes;
	Eigen::ArrayXd m_area;
	ccOctree::Shared m_octree;
	ccPointCloud* m_cloud;
	unsigned char m_bestOctreeLevel = 0;
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct m_nNSS;
	ccMainAppInterface *m_app;
	std::vector<std::vector<int>> m_stacks;
	Eigen::ArrayXi m_labels;
	Eigen::ArrayXi m_labelsnpoint;
	Eigen::ArrayXi m_localMaximumIndexes;
	Eigen::ArrayXi m_ndon;
	qG3PointDialog* m_dlg;

	static G3PointAction* s_g3PointAction;
};
}
