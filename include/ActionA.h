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

public:
	static void createAction(ccMainAppInterface *appInterface);

private:
	bool sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent);
	void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
	int segment_labels(bool useParallelStrategy=true);
	int segment_labels_steepest_slope(bool useParallelStrategy=true);
	void add_to_stack_braun_willett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi &Di, std::vector<int>& stack, int local_maximum);
	int segment_labels_braun_willett(bool useParallelStrategy=true);
	void get_neighbors_distances_slopes(unsigned index);
	bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
	void run();
	void setkNN(int kNN);

	Eigen::ArrayXXi m_neighbors_indexes;
	Eigen::ArrayXXd m_neighbors_distances;
	Eigen::ArrayXXd m_neighbors_slopes;
	int m_kNN = 20;
	ccOctree::Shared m_octree;
	ccPointCloud* m_cloud;
	unsigned char m_bestOctreeLevel = 0;
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct m_nNSS;
	ccMainAppInterface *m_app;
	Eigen::ArrayXi m_stack;
	G3PointDialog* m_dlg;

	static G3PointAction* s_g3PointAction;
};
}
