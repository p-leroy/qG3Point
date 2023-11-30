#include "ActionA.h"

#include "DgmOctree.h"

#include "ccMainAppInterface.h"
#include "ccOctree.h"
#include "ccProgressDialog.h"
#include "ccQtHelpers.h"
#include "CCGeom.h"

#include "QMainWindow"
#include "QCoreApplication"
#include "QThreadPool"
#include "QtConcurrent"

#include "algorithm"
#include "iostream"

namespace G3Point
{

Eigen::ArrayXXi neighbors_indexes;
Eigen::ArrayXXf neighbors_distances;
Eigen::ArrayXXf neighbors_slopes;
int kNN = 20;
ccOctree::Shared octree;
ccPointCloud* cloud;
unsigned char bestOctreeLevel = 0;
CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;

void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack)
{
	stack.push_back(index);

	for (int k = 0; k < n_donors(index); k++)
	{
		add_to_stack(donors(index, k), n_donors, donors, stack);
	}
}

int segment_labels(bool useParallelStrategy)
{
	// for each point, find in the neighborhood the point with the minimum slope (the receiver)
	Eigen::ArrayXf min_slopes(neighbors_slopes.rowwise().minCoeff());
	Eigen::ArrayXi index_of_min_slope = Eigen::ArrayXi::Zero(cloud->size());
	Eigen::ArrayXi receivers(cloud->size());

	for (unsigned index = 0; index < cloud->size(); index++)
	{
		float min_slope = min_slopes(index);
		for (int k = 0; k < kNN; k++)
		{
			if (neighbors_slopes(index, k) == min_slope)
			{
				index_of_min_slope(index) = k;
			}
		}
		receivers(index) = neighbors_indexes(index, index_of_min_slope(index));
	}

	// if the minimum slope is positive, the receiver is a local maximum
	int nb_maxima = (min_slopes > 0).count();
	Eigen::ArrayXi localMaximumIndexes = Eigen::ArrayXi::Zero(nb_maxima);
	int l = 0;
	for (unsigned int k = 0; k < cloud->size(); k++)
	{
		if (min_slopes(k) > 0)
		{
			localMaximumIndexes(l) = k;
			receivers(k) = k;
			l++;
		}
	}

	// identify the donors for each receiver
	Eigen::ArrayXi nDonors = Eigen::ArrayXi::Zero(cloud->size());
	Eigen::ArrayXXi donors = Eigen::ArrayXXi::Zero(cloud->size(), kNN);

	for (unsigned int k = 0; k < cloud->size(); k++)
	{
		int receiver = receivers(k);
		if (receiver != k)  // this receiver is not a local maximum
		{
			nDonors(receiver) = nDonors(receiver) + 1;
			donors(receiver, nDonors(receiver) - 1) = k;
		}
	}

	// build the stacks
	Eigen::ArrayXi labels = Eigen::ArrayXi::Zero(cloud->size());
	Eigen::ArrayXi labelsk = Eigen::ArrayXi::Zero(cloud->size());
	Eigen::ArrayXi labelsnpoint = Eigen::ArrayXi::Zero(cloud->size());
	std::vector<std::vector<int>> stacks;

	for (int k = 0; k < localMaximumIndexes.size(); k++)
	{
		int localMaximumIndex = localMaximumIndexes(k);
		std::vector<int> stack;
		add_to_stack(localMaximumIndex, nDonors, donors, stack);
		stacks.push_back(stack);
		// labels
		for (auto i : stack)
		{
			labels(i) = k;
			labelsnpoint(i) = stack.size();
		}
	}

	int nLabels = localMaximumIndexes.size();

	return nLabels;
}

void get_neighbors_distances_slopes(unsigned index)
{
	const CCVector3* P = cloud->getPoint(index);

	nNSS.level = bestOctreeLevel;
	double maxSquareDist = 0;
	int neighborhoodSize = 0;
	CCCoreLib::ReferenceCloud Yk(cloud);

	// get the nearest neighbors
	if (octree->findPointNeighbourhood(P, &Yk, static_cast<unsigned>(kNN + 1), bestOctreeLevel, maxSquareDist, 0, &neighborhoodSize) >= static_cast<unsigned>(kNN))
	{
		for (int k = 0; k < kNN; k++)
		{
			// store the index of the neighbor
			neighbors_indexes(index, k) = Yk.getPointGlobalIndex(k + 1);
			// compute the distance to the neighbor
			const CCVector3* neighbor = Yk.getPoint(k + 1);
			float distance = sqrt((*P - *neighbor).norm2());
			neighbors_distances(index, k) = distance;
			// compute the slope to the neighbor
			neighbors_slopes(index, k) = (P->z - neighbor->z) / distance;
		}
	}
}

bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy)
{
	QString errorStr;

	ccProgressDialog progressDlg(true, appInterface->getMainWindow());
	progressDlg.show();
	progressDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
	CCCoreLib::GenericProgressCallback* progressCb = &progressDlg;
	octree = cloud->getOctree();

	//get the octree
	if (!octree)
	{
		ccLog::Print(QString("Computing octree of cloud %1 (%2 points)").arg(cloud->getName()).arg(cloud->size()));
		if (progressCb)
			progressCb->start();
		QCoreApplication::processEvents();
		octree = cloud->computeOctree(progressCb);
		if (!octree)
		{
			errorStr = "Failed to compute octree (not enough memory?)";
			return false;
		}
	}

	bestOctreeLevel = octree->findBestLevelForAGivenPopulationPerCell(static_cast<unsigned>(std::max(3, kNN)));

	size_t nPoints = cloud->size();
	int maxThreadCount = 0;
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	std::vector<unsigned> pointsIndexes;
	pointsIndexes.resize(nPoints);

	if (useParallelStrategy)
	{
		for (unsigned i = 0; i < nPoints; ++i)
		{
			pointsIndexes[i] = i;
		}
		if (maxThreadCount == 0)
		{
			maxThreadCount = ccQtHelpers::GetMaxThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(pointsIndexes, get_neighbors_distances_slopes);
	}
	else
	{
		//manually call the static per-point method!
		for (unsigned i = 0; i < nPoints; ++i)
		{
			get_neighbors_distances_slopes(i);
		}
	}

	return true;
}

void performActionA( ccMainAppInterface *appInterface )
{
	if ( appInterface == nullptr )
	{
		// The application interface should have already been initialized when the plugin is loaded
		Q_ASSERT( false );

		return;
	}

	//we need one point cloud
	if (!appInterface->haveOneSelection())
	{
		appInterface->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//a real point cloud
	const ccHObject::Container& selectedEntities = appInterface->getSelectedEntities();

	ccHObject* ent = selectedEntities[0];
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
	{
		appInterface->dispToConsole("Select a cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	cloud = ccHObjectCaster::ToPointCloud(ent);

	// initialize the matrix which will contain the results
	neighbors_indexes.resize(cloud->size(), kNN);
	neighbors_distances.resize(cloud->size(), kNN);
	neighbors_slopes.resize(cloud->size(), kNN);

	// Find neighbors of each point of the cloud
	query_neighbors(cloud, appInterface, true);

	// Perform initial segmentation
	int nLabels = segment_labels();

	appInterface->dispToConsole( "[G3Point] initial segmentation: " + QString::number(nLabels) + " labels", ccMainAppInterface::STD_CONSOLE_MESSAGE );

	neighbors_indexes.resize(0, 0);
	neighbors_distances.resize(0, 0);
	neighbors_slopes.resize(0, 0);
}
}
