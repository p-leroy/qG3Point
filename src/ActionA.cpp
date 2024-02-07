#include "ActionA.h"

#include <DgmOctree.h>

#include <ccMainAppInterface.h>
#include <ccProgressDialog.h>
#include <ccQtHelpers.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <CCGeom.h>

#include <QMainWindow>
#include <QCoreApplication>
#include <QThreadPool>
#include <QtConcurrent>

#include <algorithm>
#include <iostream>
#include <random>

#include <qG3PointDialog.h>
#include <QPushButton>

#include <mlpack.hpp>

namespace G3Point
{
G3PointAction* G3PointAction::s_g3PointAction;

RGBAColorsTableType getRandomColors(int randomColorsNumber)
{
	Q_ASSERT(randomColorsNumber > 1);

	RGBAColorsTableType randomColors;
	if (!randomColors.reserveSafe(static_cast<unsigned>(randomColorsNumber)))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
	}

	//generate random colors
	int max = 255;

	std::mt19937 gen(42);  // to seed mersenne twister.
	std::uniform_int_distribution<uint16_t> dist(0, max); //1-byte types are not allowed

	for (int i = 0; i < randomColorsNumber; ++i)
	{
		ccColor::Rgb col;
		col.r = static_cast<unsigned char>(dist(gen));
		col.g = static_cast<unsigned char>(dist(gen));
//		col.b = static_cast<unsigned char>(dist(gen));
		col.b = max - static_cast<ColorCompType>((static_cast<double>(col.r) + static_cast<double>(col.g)) / 2); //cast to double to avoid overflow (whatever the type of ColorCompType!!!)
		randomColors.addElement(ccColor::Rgba(col, max));
	}

	return randomColors;
}

bool G3PointAction::sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent) // take as is from ccEntityAction.cpp
{
	int s_randomColorsNumber = 256;

	Q_ASSERT(s_randomColorsNumber > 1);

	RGBAColorsTableType* randomColors = new RGBAColorsTableType;
	if (!randomColors->reserveSafe(static_cast<unsigned>(s_randomColorsNumber)))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
		return false;
	}

	//generate random colors
	int max = 255;

	std::mt19937 gen(42);  // to seed mersenne twister.
	std::uniform_int_distribution<uint16_t> dist(0, max); //1-byte types are not allowed

	for (int i = 0; i < s_randomColorsNumber; ++i)
	{
		ccColor::Rgb col;
		col.r = static_cast<unsigned char>(dist(gen));
		col.g = static_cast<unsigned char>(dist(gen));
		col.b = static_cast<unsigned char>(dist(gen));
//		col.b = max - static_cast<ColorCompType>((static_cast<double>(col.r) + static_cast<double>(col.g)) / 2); //cast to double to avoid overflow (whatever the type of ColorCompType!!!)
		randomColors->addElement(ccColor::Rgba(col, max));
	}

	//apply random colors
	for (ccHObject* ent : selectedEntities)
	{
		ccGenericPointCloud* cloud = nullptr;

		bool lockedVertices = false;
		cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
		if (lockedVertices)
		{
			ccLog::Warning("[G3Point::sfConvertToRandomRGB] DisplayLockedVerticesWarning");
			continue;
		}
		if (cloud != nullptr) //TODO
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
			//if there is no displayed SF --> nothing to do!
			if (sf && sf->currentSize() >= pc->size())
			{
				if (!pc->resizeTheRGBTable(false))
				{
					ccLog::Error(QObject::tr("Not enough memory!"));
					break;
				}
				else
				{
					ScalarType minSF = sf->getMin();
					ScalarType maxSF = sf->getMax();

					ScalarType step = (maxSF - minSF) / (s_randomColorsNumber - 1);
					if (step == 0)
						step = static_cast<ScalarType>(1.0);

					for (unsigned i = 0; i < pc->size(); ++i)
					{
						ScalarType val = sf->getValue(i);
						unsigned colIndex = static_cast<unsigned>((val - minSF) / step);
						if (colIndex == s_randomColorsNumber)
							--colIndex;

						pc->setPointColor(i, randomColors->getValue(colIndex));
					}

					pc->showColors(true);
					pc->showSF(false); //just in case
				}
			}

			m_cloud->prepareDisplayForRefresh_recursive();
		}
	}

	return true;
}

void G3PointAction::add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack)
{
	stack.push_back(index);

	for (int k = 0; k < n_donors(index); k++)
	{
		add_to_stack(donors(index, k), n_donors, donors, stack);
	}
}

int G3PointAction::segment_labels(bool useParallelStrategy)
{
	std::cout << "[segment_labels]" << std::endl;
	// for each point, find in the neighborhood the point with the minimum slope (the receiver)
	Eigen::ArrayXd min_slopes(m_neighbors_slopes.rowwise().minCoeff());
	Eigen::ArrayXi index_of_min_slope = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi receivers(m_cloud->size());

	for (unsigned index = 0; index < m_cloud->size(); index++)
	{
		double min_slope = min_slopes(index);
		int count = 0;
		for (int k = 0; k < m_kNN; k++)
		{
			if (m_neighbors_slopes(index, k) == min_slope)
			{
				index_of_min_slope(index) = k;
				count++;
				if (count > 1)
					std::cout << "[segment_labels] slope already seen, index " << index << ", k "<< k << std::endl;
			}
		}
		receivers(index) = m_neighbors_indexes(index, index_of_min_slope(index));
	}

	// if the minimum slope is positive, the receiver is a local maximum
	int nb_maxima = (min_slopes > 0).count();
	Eigen::ArrayXi localMaximumIndexes = Eigen::ArrayXi::Zero(nb_maxima);
	int l = 0;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		if (min_slopes(k) > 0)
		{
			localMaximumIndexes(l) = k;
			receivers(k) = k;
			l++;
		}
	}

	// identify the donors for each receiver
	std::cout << "[segment_labels] identify the donors for each receiver" << std::endl;
	Eigen::ArrayXi nDonors = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXXi donors = Eigen::ArrayXXi::Zero(m_cloud->size(), m_kNN);

	std::cout << "[segment_labels] loop" << std::endl;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		int receiver = receivers(k);
		if (receiver != k)  // this receiver is not a local maximum
		{
			if (nDonors(receiver) < m_kNN - 1)
			{
				nDonors(receiver) = nDonors(receiver) + 1;
				donors(receiver, nDonors(receiver) - 1) = k;
			}
			else
			{
				std::cout << "maximum number of donors reached, k " << k << ", receiver(k) " << receiver << ", nDonors " << nDonors(receiver) << std::endl;
			}
		}
	}

	// build the stacks
	std::cout << "[segment_labels] build the stacks" << std::endl;
	Eigen::ArrayXi labels = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi labelsnpoint = Eigen::ArrayXi::Zero(m_cloud->size());
	std::vector<std::vector<int>> stacks;

	int sfIdx = m_cloud->getScalarFieldIndexByName("g3point_label");
	if (sfIdx == -1)
	{
		sfIdx = m_cloud->addScalarField("g3point_label");
		if (sfIdx == -1)
		{
			ccLog::Error("[G3Point::segment_labels] impossible to create scalar field g3point_label");
		}
	}

	CCCoreLib::ScalarField* g3point_label = m_cloud->getScalarField(sfIdx);
	RGBAColorsTableType randomColors = getRandomColors(localMaximumIndexes.size());

	if (!m_cloud->resizeTheRGBTable(false))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
		return -1;
	}

	for (int k = 0; k < localMaximumIndexes.size(); k++)
	{
		int localMaximumIndex = localMaximumIndexes(k);
		std::vector<int> stack;
		add_to_stack(localMaximumIndex, nDonors, donors, stack);
		// labels
		for (auto i : stack)
		{
			labels(i) = k;
			labelsnpoint(i) = stack.size();
			if (g3point_label)
			{
				g3point_label->setValue(i, k);
				m_cloud->setPointColor(i, randomColors.getValue(k));
			}
		}
		stacks.push_back(stack);
	}

	if (g3point_label)
	{
		g3point_label->computeMinAndMax();
	}

	m_cloud->setCurrentDisplayedScalarField(sfIdx);
	m_cloud->showColors(true);
	m_cloud->showSF(false);

//	m_cloud->redrawDisplay();
//	m_cloud->prepareDisplayForRefresh();

//	ccHObject::Container selectedEntities;
//	selectedEntities.push_back(cloud);

//	if (!sfConvertToRandomRGB(selectedEntities, m_app->getMainWindow()))
//	{
//		ccLog::Error("[G3Point::segment_labels] impossible to convert g3point_label to RGB colors");
//	}

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	int nLabels = localMaximumIndexes.size();

	return nLabels;
}

class mySearch
{
public:
	mySearch() {}

	void Search(const arma::mat& queryPoints,
				const mlpack::math::Range& range,
				std::vector<std::vector<size_t>>& neighbors,
				std::vector<std::vector<double>>& distances);
};

void mySearch::Search(const arma::mat& queryPoints,
					  const mlpack::math::Range& range,
					  std::vector<std::vector<size_t>>& neighbors,
					  std::vector<std::vector<double>>& distances)
{

}

int G3PointAction::compute_mean_angle()
{
	// Find the indexborder nodes (no donor and many other labels in the neighbourhood)
	Eigen::ArrayXXi duplicated_labels(m_cloud->size(), m_kNN);
	for (int n = 0; n < m_kNN; n++)
	{
		duplicated_labels(Eigen::all, n) = m_labels;
	}
	Eigen::ArrayXXi labels_of_neighbors(m_cloud->size(), m_kNN);
	for (int index = 0; index < m_cloud->size(); index++)
	{
		for (int n = 0; n < m_kNN; n++)
		{
			labels_of_neighbors(index, n) = m_labels(m_neighbors_indexes(index, n));
		}
	}

	Eigen::ArrayXi temp = m_kNN - (labels_of_neighbors == duplicated_labels).cast<int>().rowwise().sum();
	auto condition = ((temp >= m_kNN / 4) && (m_ndon == 0));
	Eigen::ArrayXi indborder(condition.count());
	std::cout << "condition.count() " << condition.count() << std::endl;
	int l = 0;
	for (int c = 0; c < condition.size(); c++)
	{
		if (condition(c))
		{
			indborder(l) = c;
			l++;
		}
	}

	std::cout << "temp" << std::endl;
	std::cout << condition.block(10, 0, 20, 1) << std::endl;

	std::cout << "indborder" << std::endl;
	std::cout << indborder.block(0, 0, 10, 1) << std::endl;

	// Compute the angle of the normal vector between the neighbours of each grain / label
	int nlabels = m_stacks.size();
	Eigen::ArrayXXi A = Eigen::ArrayXXi::Zero(nlabels, nlabels);
	Eigen::ArrayXXi N = Eigen::ArrayXXi::Zero(nlabels, nlabels);

	for (auto i : indborder)
	{
		auto j = m_neighbors_indexes(i, Eigen::all);  // indexes of the neighbourhood of i
		// Take the normals vector for i and j (duplicate the normal vector of i to have the same size as for j)
//		P1 = numpy.tile(normals[i, :], (params.knn, 1));
//		P2 = m_normals(j, Eigen::all);
		// Compute the angle between the normal of i and the normals of j
		// Add this angle to the angle matrix between each label
//		A[labels[i], labels[j]] = A[labels[i], labels[j]] + angle_rot_2_vec_mat(P1, P2)
		// Number of occurrences
//		N[labels[i], labels[j]] = N[labels[i], labels[j]] + 1
	}

	mlpack::DBSCAN(1, 1);
}

int G3PointAction::cluster_labels()
{
	ccLog::Print("[cluster_labels]");
	int nlabels = m_stacks.size();

	std::cout << "COMPARE VALUES " << nlabels << " " << m_localMaximumIndexes.size() << std::endl;

	// Compute the distances between the sinks associated to each label
	Eigen::ArrayXXd D1(nlabels, nlabels);
	for (int i = 0; i < nlabels; i++)
	{
		for (int j = 0; j < nlabels; j++)
		{
			D1(i, j) = (*m_cloud->getPoint(m_localMaximumIndexes(i)) - *m_cloud->getPoint(m_localMaximumIndexes(j))).norm();
		}
	}

	// Estimate the distances between labels using the areas
	int k = 0;
	Eigen::ArrayXXd D2 = Eigen::ArrayXXd::Zero(nlabels, nlabels);
	Eigen::ArrayXd radius = Eigen::ArrayXd::Zero(nlabels);
	for (auto &stack : m_stacks)  // Radius of each label (assuming the surface corresponds to a disk)
	{
		radius(k) = sqrt(m_area(stack).sum() / M_PI);
		k++;
	}
	for(int i = 0; i < nlabels; i++)  // Compute inter-distances by summing radius
	{
		for(int j = 0; j < nlabels; j++)
		{
			D2(i, j) = radius(i) + radius(j);
		}
	}

	// If the radius of the sink is above the distance to the other sink (by a factor of rad_factor), set Dist to 1
	Eigen::ArrayXXi Dist = Eigen::ArrayXXi::Zero(nlabels, nlabels);
	Dist = (rad_factor * D2 > D1).select(1, Dist);
	std::cout << "Dist" << std::endl;
	for (int i = 0; i < 10; i++)  // set the values of the diagonal to 0
	{
		Dist(i, i) = 0;
	}

	// If labels are neighbours, set Nneigh to 1
	Eigen::ArrayXXi Nneigh = Eigen::ArrayXXi::Zero(nlabels, nlabels);
	k = 0;
	for (auto &stack : m_stacks)
	{
		Eigen::ArrayXXi labels(stack.size(), m_kNN);
		for (int index = 0; index < stack.size(); index++)
		{
			for (int n = 0; n < m_kNN; n++)
			{
				labels(index, n) = m_labels(m_neighbors_indexes(stack[index], n));
			}
		}
		auto reshaped = labels.reshaped();
		std::set<int> unique_elements(reshaped.begin(), reshaped.end());
		for (auto unique : unique_elements)
		{
			Nneigh(k, unique) = 1;
		}
		k++;
	}

	compute_mean_angle();

	return 0;
}

void G3PointAction::add_to_stack_braun_willett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi& Di, std::vector<int>& stack, int local_maximum)
{
	stack.push_back(index);

	for (int k = delta[index]; k < delta[index + 1]; k++)
	{
		if (Di[k] != local_maximum) // avoid infinite loop
		{
			add_to_stack_braun_willett(Di[k], delta, Di, stack, local_maximum);
		}
	}
}

int G3PointAction::segment_labels_braun_willett(bool useParallelStrategy)
{
	std::cout << "[segment_labels_braun_willett]" << std::endl;
	// for each point, find in the neighborhood the point with the minimum slope (the receiver)
	Eigen::ArrayXd min_slopes(m_neighbors_slopes.rowwise().minCoeff());
	Eigen::ArrayXi index_of_min_slope = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi receivers(m_cloud->size());

	for (unsigned index = 0; index < m_cloud->size(); index++)
	{
		double min_slope = min_slopes(index);
		for (int k = 0; k < m_kNN; k++)
		{
			if (m_neighbors_slopes(index, k) == min_slope)
			{
				index_of_min_slope(index) = k;
				break;
			}
		}
		receivers(index) = m_neighbors_indexes(index, index_of_min_slope(index));
	}

	// if the minimum slope is positive, the receiver is a local maximum
	int nb_maxima = (min_slopes > 0).count();
	m_localMaximumIndexes = Eigen::ArrayXi::Zero(nb_maxima);
	int l = 0;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		if (min_slopes(k) > 0)
		{
			m_localMaximumIndexes(l) = k;
			receivers(k) = k;
			l++;
		}
	}

	// get the number of donors per receiver (di) and build the list of donors per receiver (Dij)
	std::cout << "[segment_labels_braun_willett] identify the donors for each receiver" << std::endl;
	Eigen::ArrayXi di = Eigen::ArrayXi::Zero(m_cloud->size()); // number of donors
	std::vector<std::vector<int>> Dij; // lists of donors (one list per point)
	for (unsigned int k = 0; k < m_cloud->size(); k++) // initialize the lists of donors
	{
		std::vector<int> list_of_donors;
		Dij.push_back(list_of_donors);
	}
	std::cout << "[segment_labels_braun_willett] create di and Dij" << std::endl;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		int receiver = receivers(k);
		di[receiver] = di[receiver] + 1; // increment the number of donors of the receiver
		Dij[receiver].push_back(k); // add the donor to the list of donors of the receiver
	}

	m_ndon = di;

	// build Di, the list of donors
	Eigen::ArrayXi Di = Eigen::ArrayXi::Zero(m_cloud->size()); // list of donors
	int idx = 0;
	for (auto& list_ : Dij) // build the list of donors
	{
		for (int point : list_)
		{
			Di[idx] = point;
			idx++;
		}
	}

	// build delta, the index array
	Eigen::ArrayXi delta = Eigen::ArrayXi::Zero(m_cloud->size() + 1); // index of the first donor
	delta[m_cloud->size()] = m_cloud->size();
	std::vector<int> delta_vec(m_cloud->size());
	for (int i = m_cloud->size() - 1; i >= 0; i--)
	{
		delta[i] = delta[i + 1] - di[i];
		delta_vec[i] = delta[i];
	}


	// build the stacks
	std::cout << "[segment_labels_braun_willett] build the stacks" << std::endl;

	int sfIdx = m_cloud->getScalarFieldIndexByName("g3point_label");
	if (sfIdx == -1)
	{
		sfIdx = m_cloud->addScalarField("g3point_label");
		if (sfIdx == -1)
		{
			ccLog::Error("[G3Point::segment_labels] impossible to create scalar field g3point_label");
		}
	}
	CCCoreLib::ScalarField* g3point_label = m_cloud->getScalarField(sfIdx);

	RGBAColorsTableType randomColors = getRandomColors(m_localMaximumIndexes.size());

	if (!m_cloud->resizeTheRGBTable(false))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
		return -1;
	}

	for (int k = 0; k < m_localMaximumIndexes.size(); k++)
	{
		int localMaximumIndex = m_localMaximumIndexes(k);
		std::vector<int> stack;
		add_to_stack_braun_willett(localMaximumIndex, delta, Di, stack, localMaximumIndex);
		// labels
		for (auto i : stack)
		{
			m_labels(i) = k;
			m_labelsnpoint(i) = stack.size();
			if (g3point_label)
			{
				g3point_label->setValue(i, k);
				m_cloud->setPointColor(i, randomColors.getValue(k));
			}
		}
		m_stacks.push_back(stack);
	}

	if (g3point_label)
	{
		g3point_label->computeMinAndMax();
	}

	m_cloud->setCurrentDisplayedScalarField(sfIdx);
	m_cloud->showColors(true);
	m_cloud->showSF(false);

	m_cloud->redrawDisplay();
	//	m_cloud->prepareDisplayForRefresh();

	//	ccHObject::Container selectedEntities;
	//	selectedEntities.push_back(cloud);

	//	if (!sfConvertToRandomRGB(selectedEntities, m_app->getMainWindow()))
	//	{
	//		ccLog::Error("[G3Point::segment_labels] impossible to convert g3point_label to RGB colors");
	//	}

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	int nLabels = m_localMaximumIndexes.size();

	return nLabels;
}

int G3PointAction::segment_labels_steepest_slope(bool useParallelStrategy)
{
	std::cout << "[segment_labels]" << std::endl;
	// for each point, find in the neighborhood the point with the steepest slope (the receiver)
	Eigen::ArrayXd steepest_slopes(m_neighbors_slopes.rowwise().maxCoeff());
	Eigen::ArrayXi index_of_steepest_slope = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi receivers(m_cloud->size());

	for (unsigned index = 0; index < m_cloud->size(); index++)
	{
		double steepest_slope = steepest_slopes(index);
		int count = 0;
		for (int k = 0; k < m_kNN; k++)
		{
			if (m_neighbors_slopes(index, k) == steepest_slope)
			{
				index_of_steepest_slope(index) = k;
				count++;
				if (count > 1)
					std::cout << "[segment_labels] slope already seen, index " << index << ", k "<< k << std::endl;
			}
		}
		receivers(index) = m_neighbors_indexes(index, index_of_steepest_slope(index));
	}

	// if the minimum slope is positive, the receiver is a base level node
	int nb_maxima = (steepest_slopes < 0).count();
	Eigen::ArrayXi localMaximumIndexes = Eigen::ArrayXi::Zero(nb_maxima);
	int l = 0;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		if (steepest_slopes(k) < 0)
		{
			localMaximumIndexes(l) = k;
			receivers(k) = k;
			l++;
		}
	}

	// identify the donors for each receiver
	std::cout << "[segment_labels] identify the donors for each receiver" << std::endl;
	Eigen::ArrayXi nDonors = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXXi donors = Eigen::ArrayXXi::Zero(m_cloud->size(), m_kNN);

	std::cout << "[segment_labels] loop" << std::endl;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		int receiver = receivers(k);
		if (receiver != k)  // this receiver is not a local maximum
		{
			if (nDonors(receiver) < m_kNN - 1)
			{
				nDonors(receiver) = nDonors(receiver) + 1;
				donors(receiver, nDonors(receiver) - 1) = k;
			}
			else
			{
				std::cout << "maximum number of donors reached, k " << k << ", receiver(k) " << receiver << ", nDonors " << nDonors(receiver) << std::endl;
			}
		}
	}

	// build the stacks
	std::cout << "[segment_labels] build the stacks" << std::endl;
	Eigen::ArrayXi labels = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi labelsnpoint = Eigen::ArrayXi::Zero(m_cloud->size());
	std::vector<std::vector<int>> stacks;

	int sfIdx = m_cloud->getScalarFieldIndexByName("g3point_label");
	if (sfIdx == -1)
	{
		sfIdx = m_cloud->addScalarField("g3point_label");
		if (sfIdx == -1)
		{
			ccLog::Error("[G3Point::segment_labels] impossible to create scalar field g3point_label");
		}
	}

	CCCoreLib::ScalarField* g3point_label = m_cloud->getScalarField(sfIdx);
	RGBAColorsTableType randomColors = getRandomColors(localMaximumIndexes.size());

	if (!m_cloud->resizeTheRGBTable(false))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
		return -1;
	}

	for (int k = 0; k < localMaximumIndexes.size(); k++)
	{
		int localMaximumIndex = localMaximumIndexes(k);
		std::vector<int> stack;
		add_to_stack(localMaximumIndex, nDonors, donors, stack);
		// labels
		for (auto i : stack)
		{
			labels(i) = k;
			labelsnpoint(i) = stack.size();
			if (g3point_label)
			{
				g3point_label->setValue(i, k);
				m_cloud->setPointColor(i, randomColors.getValue(k));
			}
		}
		stacks.push_back(stack);
	}

	if (g3point_label)
	{
		g3point_label->computeMinAndMax();
	}

	m_cloud->setCurrentDisplayedScalarField(sfIdx);
	m_cloud->showColors(true);
	m_cloud->showSF(false);

	//	m_cloud->redrawDisplay();
	//	m_cloud->prepareDisplayForRefresh();

	//	ccHObject::Container selectedEntities;
	//	selectedEntities.push_back(cloud);

	//	if (!sfConvertToRandomRGB(selectedEntities, m_app->getMainWindow()))
	//	{
	//		ccLog::Error("[G3Point::segment_labels] impossible to convert g3point_label to RGB colors");
	//	}

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	int nLabels = localMaximumIndexes.size();

	return nLabels;
}

void G3PointAction::get_neighbors_distances_slopes(unsigned index)
{
	const CCVector3* P = m_cloud->getPoint(index);

	m_nNSS.level = m_bestOctreeLevel;
	double maxSquareDist = 0;
	int neighborhoodSize = 0;
	CCCoreLib::ReferenceCloud Yk(m_cloud);

	// get the nearest neighbors
	if (m_octree->findPointNeighbourhood(P, &Yk, static_cast<unsigned>(m_kNN + 1), m_bestOctreeLevel, maxSquareDist, 0, &neighborhoodSize) >= static_cast<unsigned>(m_kNN))
	{
		for (int k = 0; k < m_kNN; k++)
		{
			// store the index of the neighbor
			m_neighbors_indexes(index, k) = Yk.getPointGlobalIndex(k + 1);
			// compute the distance to the neighbor
			const CCVector3* neighbor = Yk.getPoint(k + 1);
			float distance = (*P - *neighbor).norm();
			m_neighbors_distances(index, k) = distance;
			// compute the slope to the neighbor
			m_neighbors_slopes(index, k) = (P->z - neighbor->z) / distance;
		}
	}
}

void G3PointAction::compute_node_surfaces()
{
	m_area = M_PI * m_neighbors_distances.rowwise().minCoeff().square();
}

void G3PointAction::orient_normals()
{
	// Flip the normals so they are oriented towards the sensor center
	// x,y,z: points
	// u,v,w: normals
	// ox,oy,oz: sensor center

//	p1 = sensor_center - points
//	p2 = normals

// Flip the normals if they are not pointing towards the sensor
//angle = np.arctan2(np.linalg.norm(np.cross(p1, p2), axis=1), np.sum(p1 * p2, axis=1))
//index = (angle > np.pi / 2) | (angle < -np.pi / 2)
//normals[index] = -normals[index]  # invert normal

//return normals
}

void G3PointAction::compute_normals_and_orient_them()
{
//	pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(params.knn))
//	centroid = np.mean(xyz, axis=0)
//	sensor_center = np.array([centroid[0], centroid[1], 1000])
//	normals = orient_normals(xyz, np.asarray(pcd.normals), sensor_center)
}

bool G3PointAction::query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy)
{
	std::cout << "[query_neighbor]" << std::endl;
	QString errorStr;

	ccProgressDialog progressDlg(true, appInterface->getMainWindow());
	progressDlg.show();
	progressDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
	CCCoreLib::GenericProgressCallback* progressCb = &progressDlg;
	m_octree = m_cloud->getOctree();

	//get the octree
	if (!m_octree)
	{
		ccLog::Print(QString("Computing octree of cloud %1 (%2 points)").arg(m_cloud->getName()).arg(m_cloud->size()));
		if (progressCb)
			progressCb->start();
		QCoreApplication::processEvents();
		m_octree = m_cloud->computeOctree(progressCb);
		if (!m_octree)
		{
			errorStr = "Failed to compute octree (not enough memory?)";
			return false;
		}
	}

	m_bestOctreeLevel = m_octree->findBestLevelForAGivenPopulationPerCell(static_cast<unsigned>(std::max(3, m_kNN)));

	size_t nPoints = m_cloud->size();
	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	std::vector<unsigned> pointsIndexes;
	pointsIndexes.resize(nPoints);

	if (useParallelStrategy)
	{
		for (unsigned i = 0; i < nPoints; ++i)
		{
			pointsIndexes[i] = i;
		}
		int threadCount = std::max(1, ccQtHelpers::GetMaxThreadCount() - 2);
		std::cout << "[query_neighbor] parallel strategy, thread count " << threadCount <<  std::endl;
		QThreadPool::globalInstance()->setMaxThreadCount(threadCount);
		QtConcurrent::blockingMap(pointsIndexes, [=](int index){get_neighbors_distances_slopes(index);});
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

void G3PointAction::run()
{
	m_kNN = m_dlg->getkNN();

	// initialize the matrix which will contain the results
	m_neighbors_indexes.resize(m_cloud->size(), m_kNN);
	m_neighbors_distances.resize(m_cloud->size(), m_kNN);
	m_neighbors_slopes.resize(m_cloud->size(), m_kNN);
	m_labels = Eigen::ArrayXi::Zero(m_cloud->size());
	m_labelsnpoint = Eigen::ArrayXi::Zero(m_cloud->size());
	m_stacks.clear();  // needed in case of several runs

	// Find neighbors of each point of the cloud
	query_neighbors(m_cloud, m_app, true);

	compute_node_surfaces();

	// Perform initial segmentation
	// int nLabels = segment_labels();
	int nLabels = segment_labels_braun_willett();
	//	int nLabels = segment_labels_steepest_slope();

	cluster_labels();

	m_app->dispToConsole( "[G3Point] initial segmentation: " + QString::number(nLabels) + " labels", ccMainAppInterface::STD_CONSOLE_MESSAGE );

	m_neighbors_indexes.resize(0, 0);
	m_neighbors_distances.resize(0, 0);
	m_neighbors_slopes.resize(0, 0);
}

void G3PointAction::createAction(ccMainAppInterface *appInterface)
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

	if (s_g3PointAction == nullptr) // create the singleton if needed
	{
		s_g3PointAction = new G3PointAction();
	}

	s_g3PointAction->m_app = appInterface;
	//display dialog
	s_g3PointAction->m_dlg = new qG3PointDialog();
	s_g3PointAction->m_dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	s_g3PointAction->m_dlg->setWindowFlag(Qt::WindowStaysOnTopHint, true);
	s_g3PointAction->m_dlg->setWindowTitle("G3Point");

	connect(s_g3PointAction->m_dlg, &qG3PointDialog::run, s_g3PointAction, &G3PointAction::run);
	s_g3PointAction->m_cloud = ccHObjectCaster::ToPointCloud(ent);
	s_g3PointAction->m_dlg->show();
}
}
