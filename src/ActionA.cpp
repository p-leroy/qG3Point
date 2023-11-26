#include "ActionA.h"

#include "DgmOctree.h"

#include "ccMainAppInterface.h"
#include "ccOctree.h"
#include "ccProgressDialog.h"
#include "ccQtHelpers.h"

#include "QMainWindow"
#include "QCoreApplication"
#include "QThreadPool"
#include "QtConcurrent"

#include "Eigen/Dense"

#include "algorithm"

namespace G3Point
{

Eigen::MatrixXi neighbors_indexes;
int kNN = 20;
ccOctree::Shared octree;
ccPointCloud* cloud;
unsigned char bestOctreeLevel = 0;
CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;

void get_neighbors(unsigned index)
{
	const CCVector3* P = cloud->getPoint(index);


	nNSS.level = bestOctreeLevel;

	octree->findNearestNeighborsStartingFromCell();
}

bool query_neighbors(ccMainAppInterface* appInterface, bool useParallelStrategy)
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
		QtConcurrent::blockingMap(pointsIndexes, get_neighbors);
	}
	else
	{
		//manually call the static per-point method!
		for (unsigned i = 0; i < nPoints; ++i)
		{
			get_neighbors(i);
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

	/*** HERE STARTS THE ACTION ***/

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

	// Find neighbors of each point of the cloud
	query_neighbors(cloud, appInterface);

	// Perform initial segmentation

	// This is how you can output messages
	// Display a standard message in the console
	appInterface->dispToConsole( "[ExamplePlugin] Hello world!", ccMainAppInterface::STD_CONSOLE_MESSAGE );

	// Display a warning message in the console
	appInterface->dispToConsole( "[ExamplePlugin] Warning: example plugin shouldn't be used as is", ccMainAppInterface::WRN_CONSOLE_MESSAGE );

	// Display an error message in the console AND pop-up an error box
	appInterface->dispToConsole( "Example plugin shouldn't be used - it doesn't do anything!", ccMainAppInterface::ERR_CONSOLE_MESSAGE );

	/*** HERE ENDS THE ACTION ***/
}
}
