#include "ActionA.h"

#include <DgmOctree.h>

#include <ccMainAppInterface.h>
#include <ccProgressDialog.h>
#include <ccQtHelpers.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <CCGeom.h>
#include <ccNormalVectors.h>
#include <ccGLWindowInterface.h>

#include <QMainWindow>
#include <QCoreApplication>
#include <QThreadPool>
#include <QtConcurrent>
#include <QApplication>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <random>

#include <G3PointDialog.h>
#include <QPushButton>
#include <QOpenGLShaderProgram>

#include <open3d/geometry/PointCloud.h>

#include <Eigen/Geometry>

#include <math.h>

#include <set>

#include <ccPointCloud.h>

namespace G3Point
{
G3PointAction* G3PointAction::s_g3PointAction = nullptr;

G3PointAction::G3PointAction(ccPointCloud *cloud, ccMainAppInterface *app)
	: m_cloud(cloud)
	, m_app(app)
	, m_dlg(nullptr)
{
}

G3PointAction::~G3PointAction()
{
	if (s_g3PointAction)
	{
		if (m_dlg)
		{
			delete m_dlg;
		}
	}
}

void G3PointAction::GetG3PointAction(ccPointCloud *cloud, ccMainAppInterface *app)
{
	if (!s_g3PointAction) // create the singleton if needed
	{
		s_g3PointAction = new G3PointAction(cloud, app);
	}
	else
	{
		s_g3PointAction->setCloud(cloud);
	}
	s_g3PointAction->showDlg();
}

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

void G3PointAction::addToStack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack)
{
	stack.push_back(index);

	for (int k = 0; k < n_donors(index); k++)
	{
		addToStack(donors(index, k), n_donors, donors, stack);
	}
}

int G3PointAction::segmentLabels(bool useParallelStrategy)
{
	std::cout << "[segment_labels]" << std::endl;
	// for each point, find in the neighborhood the point with the minimum slope (the receiver)
	Eigen::ArrayXd min_slopes(m_neighborsSlopes.rowwise().minCoeff());
	Eigen::ArrayXi index_of_min_slope = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi receivers(m_cloud->size());

	for (unsigned index = 0; index < m_cloud->size(); index++)
	{
		double min_slope = min_slopes(index);
		int count = 0;
		for (int k = 0; k < m_kNN; k++)
		{
			if (m_neighborsSlopes(index, k) == min_slope)
			{
				index_of_min_slope(index) = k;
				count++;
				if (count > 1)
					std::cout << "[segment_labels] slope already seen, index " << index << ", k "<< k << std::endl;
			}
		}
		receivers(index) = m_neighborsIndexes(index, index_of_min_slope(index));
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
		addToStack(localMaximumIndex, nDonors, donors, stack);
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

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	int nLabels = localMaximumIndexes.size();

	return nLabels;
}

double G3PointAction::angleRot2VecMat(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	double angle;
	Eigen::Vector3d c;
	double d;

	c = a.cross(b);

	d = a.dot(b);

	angle = atan2(c.norm(), d) * 180 / M_PI;

	return angle;
}

Eigen::ArrayXXd G3PointAction::computeMeanAngleBetweenNormalsAtBorders()
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
			labels_of_neighbors(index, n) = m_labels(m_neighborsIndexes(index, n));
		}
	}

	Eigen::ArrayXi temp = m_kNN - (labels_of_neighbors == duplicated_labels).cast<int>().rowwise().sum();
	auto condition = ((temp >= m_kNN / 4) && (m_ndon == 0));
	Eigen::ArrayXi indborder(condition.count());
	// std::cout << "condition.count() " << condition.count() << std::endl;
	int l = 0;
	for (int c = 0; c < condition.size(); c++)
	{
		if (condition(c))
		{
			indborder(l) = c;
			l++;
		}
	}

	// std::cout << "indborder" << std::endl;
	// std::cout << indborder.block(0, 0, 10, 1) << std::endl;

	// Compute the angle of the normal vector between the neighbours of each grain / label
	size_t nlabels = m_stacks.size();
	Eigen::ArrayXXd A = Eigen::ArrayXXd::Zero(nlabels, nlabels);
	Eigen::ArrayXXd Aangle(nlabels, nlabels);
	Aangle.fill(NAN);
	Eigen::ArrayXXi N = Eigen::ArrayXXi::Zero(nlabels, nlabels);

	for (auto i : indborder)
	{
		auto neighbors = m_neighborsIndexes(i, Eigen::all);  // indexes of the neighbors of i
		Eigen::Vector3d N1(m_normals(i, Eigen::all)); // normal at i
		for (auto j : neighbors)
		{
			// Take the normals vector for i and j
			Eigen::Vector3d N2(m_normals(j, Eigen::all)); // normal at j
			double angle = angleRot2VecMat(N1, N2);
			if (i < 27)
			{
				// std::cout << "INDBORDER i " << i << " j " "" << j << std::endl;
				// std::cout << angle << std::endl;
			}
			if ((m_labels(i) != -1) && (m_labels(j) != -1))  // points which belong to the discarded grains have the -1 label
			{
				if ((m_labels(i) > A.rows()) || (m_labels(j) > A.rows()))
				{
					std::cout << "ERROR " << m_labels(i) << " " << m_labels(j) << "(nlabels " << nlabels << ")" << std::endl;
				}
				else
				{
					A(m_labels(i), m_labels(j)) = A(m_labels(i), m_labels(j)) + angle;
					N(m_labels(i), m_labels(j)) = N(m_labels(i), m_labels(j)) + 1;
				}
			}
		}
		if (i < 31)
		{
			// std::cout << "INDBORDER i " << i << std::endl;
			// std::cout << A.block(0, 0, 10, 10) << std::endl;
		}
	}

	// std::cout << "A" << std::endl;
	// std::cout << A.block(0, 0, 10, 10) << std::endl;

	/// compute the means
for (int r = 0; r < nlabels; r++)
	{
		for(int c = 0; c < nlabels; c++)
		{
			if (N(r, c) != 0)
			{
				Aangle(r, c) = A(r, c) / N(r, c);
			}
		}
	}

	// std::cout << "A mean" << std::endl;
	// std::cout << A.block(0, 0, 10, 10) << std::endl;

	return Aangle;
}

bool G3PointAction::checkStacks(const std::vector<std::vector<int>>& stacks, int count)
{
	std::set<int> indexes;
	bool ret = true;
	int errorCount = 0;

	// the stacks shall contain each point, only one time
	for (auto& stack : stacks)
	{
		for (int index : stack)
		{
			if (indexes.count(index))
			{
				ccLog::Warning("[G3PointAction::check_stacks] index already in the set " + QString::number(index));
				ret = false;
				errorCount++;
			}
			indexes.insert(index);
		}
	}

	if (errorCount)
	{
		ccLog::Error("[G3PointAction::check_stacks] number of duplicates " + QString::number(errorCount));
	}

	// the number of points in the stacks shall be equal to count
	if(indexes.size() != count)
	{
		ccLog::Warning("[G3PointAction::check_stacks] size of indexes " + QString::number(indexes.size()) + ", point count " + QString::number(m_cloud->size()));
		ret = false;
	}

	return ret;
}

bool G3PointAction::updateLocalMaximumIndexes()
{
	size_t nlabels = m_stacks.size();

	Eigen::ArrayXi localMaximumIndexes = Eigen::ArrayXi::Ones(nlabels) * (-1);

	for (int k = 0; k < nlabels; k++)
	{
		auto& stack = m_stacks[k];
		size_t nPoints = stack.size();
		Eigen::ArrayXd elevations(nPoints);
		for (int index = 0; index < nPoints; index++)
		{
			const CCVector3* point = m_cloud->getPoint(stack[index]);
			elevations(index) = point->z;
		}
		Eigen::Index maxIndex;
		elevations.maxCoeff(&maxIndex);
		localMaximumIndexes(k) = stack[maxIndex];
	}

	if ((localMaximumIndexes == -1).any())
	{
		ccLog::Error("[G3PointAction::updateLocalMaximumIndexes] CANCEL value error in the indexes of the local maximima");
		return false;
	}

	m_localMaximumIndexes = localMaximumIndexes;

	return true;
}

bool G3PointAction::updateLabelsAndColors()
{
	std::cout << "[G3PointAction::update_labels_and_colors]" << std::endl;

	// g3point_label scalar field
	int sfIdx = m_cloud->getScalarFieldIndexByName("g3point_label");
	if (sfIdx == -1)
	{
		sfIdx = m_cloud->addScalarField("g3point_label");
		if (sfIdx == -1)
		{
			ccLog::Error("[G3PointAction::update_labels_and_colors] impossible to create scalar field g3point_label");
		}
	}
	CCCoreLib::ScalarField* g3point_label = m_cloud->getScalarField(sfIdx);

	RGBAColorsTableType randomColors = getRandomColors(m_stacks.size());

	if (!m_cloud->resizeTheRGBTable(false))
	{
		ccLog::Error(QObject::tr("[G3PointAction::update_labels_and_colors] Not enough memory!"));
		return false;
	}

	for (int index = 0; index < m_cloud->size(); index++)  // points which are not in the stacks will have the label -1
	{
		g3point_label->setValue(index, -1);
		m_labels = -1;
	}

	for (int k = 0; k < m_stacks.size(); k++)
	{
		const std::vector<int>& stack = m_stacks[k];
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
	}

	if (g3point_label)
	{
		g3point_label->computeMinAndMax();
	}

	m_cloud->setCurrentDisplayedScalarField(sfIdx);
	m_cloud->showColors(true);
	m_cloud->showSF(false);

	m_cloud->redrawDisplay();
	m_cloud->prepareDisplayForRefresh();

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	return true;
}

bool G3PointAction::exportLocalMaximaAsCloud(const Eigen::ArrayXi& localMaximumIndexes)
{
	// create cloud
	// QString cloudName = m_cloud->getName() + "_g3point";
	QString cloudName = "g3point_summits";
	ccPointCloud *cloud = new ccPointCloud(cloudName);

	RGBAColorsTableType randomColors = getRandomColors(localMaximumIndexes.size());

	for (auto index : localMaximumIndexes)
	{
		cloud->addPoint(*m_cloud->getPoint(index));
	}

	//allocate colors if necessary
	if (cloud->resizeTheRGBTable())
	{
		for (int index = 0; index < cloud->size(); index++)
		{
			cloud->setPointColor(index, randomColors.getValue(index));
		}
	}

	cloud->showColors(true);

	ccHObject* parent = m_cloud->getParent();
	int nbChildren = parent->getChildrenNumber();
	std::vector<ccHObject *> toDelete;
	for (int k = 0; k < nbChildren; k++)
	{
		auto child = parent->getChild(k);

		if (child->getName() == cloudName)
		{
			toDelete.push_back(child);
		}
	}
	for (auto& child : toDelete)
	{
		m_app->removeFromDB(child, true);
	}

	parent->addChild(cloud, ccHObject::DP_PARENT_OF_OTHER, 0);
	m_app->addToDB(cloud);
	cloud->setEnabled(false);

	return true;
}

bool G3PointAction::processNewStacks(std::vector<std::vector<int>>& newStacks, int pointCount)
{
	if (!checkStacks(newStacks, pointCount))
	{
		ccLog::Error("[G3PointAction::processNewStacks] newStacks is not valid");
		return false;
	}

	// new stacks are valid, set the class attribute
	m_stacks = newStacks;

	updateLocalMaximumIndexes();

	updateLabelsAndColors();

	exportLocalMaximaAsCloud(m_localMaximumIndexes);

	return true;
}

bool G3PointAction::merge(XXb& condition)
{
	std::vector<std::vector<int>> newStacks;
	Eigen::ArrayXi newLabels = Eigen::ArrayXi::Ones(m_labels.size()) * (-1);
	int countNewLabels = 0;
	size_t nlabels = m_stacks.size();

	if (condition.rows() != m_stacks.size()) // check that condition is validd
	{
		ccLog::Error("[G3PointAction::merge] the shape of the condition (" + QString::number(condition.rows())
					 + ", " + QString::number(condition.cols())
					 + ") is not coherent with the stacks size " + QString::number(m_stacks.size()));
		return false;
	}

	for (int label = 0; label < nlabels; label++)
	{

		if (newLabels(label) == -1) // the label has not already been merged
		{
			newLabels(label) = countNewLabels;
			newStacks.push_back(m_stacks[label]); // initialize a newStack with the stack of the current label
			countNewLabels++;
		}

		for (int otherLabel = 0; otherLabel < nlabels; otherLabel++)
		{

			if (otherLabel == label)
			{
				continue; // do not try to merge a label with itself
			}

			// shall we merge otherLabel with label?
			if (!condition(label, otherLabel))
			{

				std::vector<int>& labelStack = newStacks[newLabels(label)];

				if (newLabels(otherLabel) != -1) // the other label has already been merged
				{
					std::vector<int>& otherLabelStack = newStacks[newLabels(otherLabel)];
					if (newLabels(label) > newLabels(otherLabel)) // merge label in otherLabel
					{
						// add the label stack to the otherLabel stack
						otherLabelStack.insert(otherLabelStack.end(), labelStack.begin(), labelStack.end()); // add the stack to the label stack
						// empty the label stack
						labelStack.clear();
						// update the label
						newLabels(label) = newLabels(otherLabel);
					}
					if (newLabels(label) < newLabels(otherLabel)) // merge otherLabel in label
					{
						// add the otherLabel stack to the label stack
						labelStack.insert(labelStack.end(), otherLabelStack.begin(), otherLabelStack.end()); // add the stack to the label stack
						// empty the otherLabel stack
						otherLabelStack.clear();
						// update the otherLabel
						newLabels(otherLabel) = newLabels(label);
					}
				}
				else  // merge otherLabel and label
				{
					std::vector<int>& otherLabelStack = m_stacks[otherLabel];
					// add the otherLabel stack to the label stack
					labelStack.insert(labelStack.end(), otherLabelStack.begin(), otherLabelStack.end()); // add the stack to the label stack
					// update the otherLabel
					newLabels(otherLabel) = newLabels(label);
				}
			}
		}
	}

	// remove empty stacks
	std::vector<std::vector<int>> newStacksWithoutEmpty;
	for (auto& stack : newStacks)
	{
		if (!stack.empty())
		{
			newStacksWithoutEmpty.push_back(stack);
		}
	}

	newStacks = newStacksWithoutEmpty;

	ccLog::Print("[G3PointAction::merge] keep " + QString::number(newStacks.size())
				 + "/" + QString::number(m_stacks.size()) + " labels ("
				 + QString::number(m_stacks.size() - newStacks.size()) + " removed)");

	if (!processNewStacks(newStacks, m_cloud->size()))
	{
		ccLog::Error("[G3PointAction::merge] processing newStacks failed");
	}

	return true;
}

bool G3PointAction::keep(Xb& condition)
{
	std::vector<std::vector<int>> newStacks;
	size_t pointCount = 0;

	for (int index = 0; index < condition.size(); index++)
	{
		std::vector<int>& stack = m_stacks[index];
		if (condition(index)) // if the condition is met, we keep the stack
		{
			newStacks.push_back(stack);
			pointCount = pointCount + stack.size();
		}
	}

	ccLog::Print("[G3PointAction::keep] keep " + QString::number(newStacks.size())
				 + "/" + QString::number(m_stacks.size()) + " gains ("
				 + QString::number(m_stacks.size() - newStacks.size()) + " removed)");

	if (!processNewStacks(newStacks, pointCount))
	{
		ccLog::Error("[G3PointAction::keep] processing newStacks failed");
	}

	return true;
}

template<typename T>
bool eigenArrayToFile(QString name, T array)
{
	const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
	std::ofstream file(name.toLatin1());
	file << array.format(CSVFormat);
	return true;
}

bool G3PointAction::cluster()
{
	ccLog::Print("[cluster_labels]");
	size_t nlabels = m_stacks.size();

	m_maxAngle1 = m_dlg->getMaxAngle1();
	m_radiusFactor = m_dlg->getRadiusFactor();

	// Compute the distances between the sinks associated to each label
	Eigen::ArrayXXd D1(nlabels, nlabels);
	if (m_localMaximumIndexes.size() != nlabels)
	{
		ccLog::Error("[G3PointAction::cluster] m_localMaximumIndexes size ("
					 + QString::number(m_localMaximumIndexes.size())
					 + ") different from nlabels "
					 + QString::number(nlabels));
		return false;
	}
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
	Dist = (m_radiusFactor * D2 > D1).select(1, Dist);
	std::cout << "Dist" << std::endl;
	for (int i = 0; i < 10; i++)  // set the values of the diagonal to 0
	{
		Dist(i, i) = 0;
	}

	// If labels are neighbours, set Nneigh to 1
	Eigen::ArrayXXi Nneigh = Eigen::ArrayXXi::Zero(nlabels, nlabels);
	k = 0;
	for (auto& stack : m_stacks)
	{
		Eigen::ArrayXXi labels(stack.size(), m_kNN);
		for (int index = 0; index < stack.size(); index++)
		{
			for (int n = 0; n < m_kNN; n++)
			{
				labels(index, n) = m_labels(m_neighborsIndexes(stack[index], n));
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

	Eigen::ArrayXXd A = computeMeanAngleBetweenNormalsAtBorders();

	// merge labels if sinks are
	// => close to each other (Dist == 1)
	// => neighbours (Nneigh == 1)
	// => normals are similar

	int start = 15;
	int size = 5;

	std::cout << "\n\nDist" << std::endl;
	std::cout << Dist.block(start, start, size, size) << std::endl;
	std::cout << "\n\nNneigh" << std::endl;
	std::cout << Nneigh.block(start, start, size, size) << std::endl;
	std::cout << "\n\nA" << std::endl;
	std::cout << A.block(start, start, size, size) << std::endl;

	if (!checkStacks(m_stacks, m_cloud->size()))
	{
		ccLog::Error("m_stacks is not valid");
	}

	// create the condition matrix and force the symmetry of the matrix
	XXb condition = (Dist < 1) || (Nneigh < 1) || (A > m_maxAngle1) || (A != A);
	XXb symmetrical_condition = (condition == condition.transpose()).select(condition, true);
	symmetrical_condition.count();
	condition = symmetrical_condition;

	std::cout << "\n\nsymmetrical_condition" << std::endl;
	std::cout << symmetrical_condition.block(start, start, size, size) << std::endl;

	// <SAVE>
	eigenArrayToFile("C:/dev/python/g3point_python/data/debug/Dist.csv", Dist);
	eigenArrayToFile("C:/dev/python/g3point_python/data/debug/Nneigh.csv", Nneigh);
	eigenArrayToFile("C:/dev/python/g3point_python/data/debug/A.csv", A);
	eigenArrayToFile("C:/dev/python/g3point_python/data/debug/symmetrical_condition.csv", symmetrical_condition);
	// </SAVE>

	std::vector<std::vector<int>> newStacks;
	Eigen::ArrayXi newLabels = Eigen::ArrayXi::Ones(m_labels.size()) * (-1);
	int countNewLabels = 0;

	for (int label = 0; label < nlabels; label++)
	{

		if (newLabels(label) == -1) // the label has not already been merged
		{
			newLabels(label) = countNewLabels;
			newStacks.push_back(m_stacks[label]); // initialize a newStack with the stack of the current label
			countNewLabels++;
		}

		for (int otherLabel = 0; otherLabel < nlabels; otherLabel++)
		{

			if (otherLabel == label)
			{
				continue; // do not try to merge a label with itself
			}

			// shall we merge otherLabel with label?
			if (!condition(label, otherLabel))
			{
				std::vector<int>& labelStack = newStacks[newLabels(label)];

				if (newLabels(otherLabel) != -1) // the other label has already been merged
				{
					std::vector<int>& otherLabelStack = newStacks[newLabels(otherLabel)];
					if (newLabels(label) > newLabels(otherLabel)) // merge label in otherLabel
					{
						// add the label stack to the otherLabel stack
						otherLabelStack.insert(otherLabelStack.end(), labelStack.begin(), labelStack.end()); // add the stack to the label stack
						// empty the label stack
						labelStack.clear();
						// update the label
						newLabels(label) = newLabels(otherLabel);
					}
					if (newLabels(label) < newLabels(otherLabel)) // merge otherLabel in label
					{
						// add the otherLabel stack to the label stack
						labelStack.insert(labelStack.end(), otherLabelStack.begin(), otherLabelStack.end()); // add the stack to the label stack
						// empty the otherLabel stack
						otherLabelStack.clear();
						// update the otherLabel
						newLabels(otherLabel) = newLabels(label);
					}
				}
				else  // merge otherLabel and label
				{
					std::vector<int>& otherLabelStack = m_stacks[otherLabel];
					// add the otherLabel stack to the label stack
					labelStack.insert(labelStack.end(), otherLabelStack.begin(), otherLabelStack.end()); // add the stack to the label stack
					// update the otherLabel
					newLabels(otherLabel) = newLabels(label);
				}
			}
		}
	}

	// remove empty stacks
	std::vector<std::vector<int>> newStacksWithoutEmpty;
	for (auto& stack : newStacks)
	{
		if (!stack.empty())
		{
			newStacksWithoutEmpty.push_back(stack);
		}
	}
	std::cout << "m_stacks.size() " << m_stacks.size()
			  << " newStacks.size() " << newStacks.size()
			  << " newStacksWithoutEmpty.size() " << newStacksWithoutEmpty.size() << std::endl;

	newStacks = newStacksWithoutEmpty;

	std::cout << "(a) m_stacks.size() " << m_stacks.size() << std::endl;
	std::cout << "(a) m_labels.size() " << m_labels.size() << std::endl;
	for (int k = 0; k < 10; k++)
	{
		std::cout << m_stacks[k].size() << " " << newStacks[k].size() << std::endl;
	}

	ccLog::Print("[G3PointAction::merge] keep " + QString::number(newStacks.size())
				 + "/" + QString::number(m_stacks.size()) + " labels ("
				 + QString::number(m_stacks.size() - newStacks.size()) + " removed)");

	if (!processNewStacks(newStacks, m_cloud->size()))
	{
		ccLog::Error("[G3PointAction::cluster] new stacks are not valid");
		return false;
	}

	std::cout << "(b) m_stacks.size() " << m_stacks.size() << std::endl;
	std::cout << "(b) m_labels.size() " << m_labels.size() << std::endl;

	return true;
}

void G3PointAction::fit()
{
	// plot display grains as ellipsoids
	m_grainColors.reset(new RGBAColorsTableType(getRandomColors(m_localMaximumIndexes.size())));
	m_grainsAsEllipsoids = new GrainsAsEllipsoids(m_cloud, m_app, m_stacks, *m_grainColors);
	m_grainsAsEllipsoids->setLocalMaximumIndexes(m_localMaximumIndexes);
	m_grainsAsEllipsoids->setName("grains as ellipsoids");

	// add connections with the dialog
	connect(m_dlg, &G3PointDialog::onlyOneClicked, m_grainsAsEllipsoids, &GrainsAsEllipsoids::showOnlyOne);
	connect(m_dlg, &G3PointDialog::allClicked, m_grainsAsEllipsoids, &GrainsAsEllipsoids::showAll);
	connect(m_dlg, &G3PointDialog::onlyOneChanged, m_grainsAsEllipsoids, &GrainsAsEllipsoids::setOnlyOne);
	connect(m_dlg, &G3PointDialog::transparencyChanged, m_grainsAsEllipsoids, &GrainsAsEllipsoids::setTransparency);
	connect(m_dlg, &G3PointDialog::drawSurfaces, m_grainsAsEllipsoids, &GrainsAsEllipsoids::drawSurfaces);
	connect(m_dlg, &G3PointDialog::drawLines, m_grainsAsEllipsoids, &GrainsAsEllipsoids::drawLines);
	connect(m_dlg, &G3PointDialog::drawPoints, m_grainsAsEllipsoids, &GrainsAsEllipsoids::drawPoints);
	connect(m_dlg, &G3PointDialog::glPointSize, m_grainsAsEllipsoids, &GrainsAsEllipsoids::setGLPointSize);

	m_dlg->setOnlyOneMax(m_stacks.size());
	m_dlg->emitSignals(); // force to send parameters to m_grainsAsEllipsoids

	m_cloud->getParent()->addChild(m_grainsAsEllipsoids);
	m_app->addToDB(m_cloud);

	m_app->updateUI();
}

bool G3PointAction::cleanLabels()
{
	ccLog::Print("[cleanLabels]");

	m_maxAngle2 = m_dlg->getMaxAngle2();
	m_nMin = m_dlg->getNMin();
	m_minFlatness = m_dlg->getMinFlatness();

	// merge points considering the normals at the border
	{
		ccLog::Print("[cleanLabels] merge points considering the normals at the border");
		Eigen::ArrayXXd A = computeMeanAngleBetweenNormalsAtBorders();
		size_t nGrains = m_stacks.size();
		XXb condition = (A > m_maxAngle2) || (A != A)  || (Eigen::MatrixXi::Identity(nGrains, nGrains).array() == 1); // add true on the diagonal (important for the if hereafter)
		XXb symmetrical_condition = (condition == condition.transpose()).select(condition, true);
		if (condition.all())
		{
			ccLog::Print("[cleanLabels] nothing to merge, continue");
		}
		else
		{
			merge(condition);
		}

		QApplication::processEvents();
	}

	//remove small labels
	{
		ccLog::Print("[cleanLabels] remove small labels");
		Eigen::ArrayXi stackSize(m_stacks.size());
		for (size_t k = 0; k < m_stacks.size(); k++)
		{
			stackSize(k) = m_stacks[k].size();
		}
		Xb condition = (stackSize > m_nMin);
		size_t numberOfGrainsToKeep = condition.count();
		if (numberOfGrainsToKeep == m_stacks.size())
		{
			ccLog::Print("[cleanLabels] all grains are larger than " + QString::number(m_nMin) + " points, nothing to remove");
		}
		else if (numberOfGrainsToKeep)
		{
			keep(condition);
		}
		else
		{
			ccLog::Error("[cleanLabels] no remaining grain after removing those with less than " + QString::number(m_nMin) + " points");
			return false;
		}
		QApplication::processEvents();
	}

	// remove flattish labels
	{
		ccLog::Print("[cleanLabels] remove flattish labels");
		Eigen::ArrayX3d s(m_stacks.size(), 3);
		for (size_t k = 0; k < m_stacks.size(); k++)
		{
			std::vector<int>& stack = m_stacks[k];
			// get the points of the label
			size_t nPoints = stack.size();
			Eigen::MatrixX3d points(nPoints, 3);
			for (int index = 0; index < nPoints; index++)
			{
				const CCVector3* point = m_cloud->getPoint(stack[index]);
				points(index, 0) = point->x;
				points(index, 1) = point->y;
				points(index, 2) = point->z;
			}
			// compute the centroid of the label
			Eigen::RowVector3d centroid = points.colwise().mean();
			points.rowwise() -= centroid;
			// SVD decomposition A = U S V∗
			s(k, Eigen::all) = points.jacobiSvd().singularValues();
		}
		// filtering condition: (l2 / l0 > min_flatness) or (l1 / l0 > 2 * min_flatness)
		Xb condition = (s(Eigen::all, 2) / s(Eigen::all, 0) > m_minFlatness)
					   || (s(Eigen::all, 1) / s(Eigen::all, 0) > 2. * m_minFlatness);
		size_t numberOfGrainsToKeep = condition.count();
		if (numberOfGrainsToKeep == m_stacks.size())
		{
			ccLog::Print("[cleanLabels] no flattish grain, nothing to remove");
		}
		else if (numberOfGrainsToKeep)
		{
			keep(condition);
		}
		else
		{
			ccLog::Error("[cleanLabels] no remaining grain after removing the flattish ones");
			return false;
		}
	}

	return true;
}

void G3PointAction::addToStackBraunWillett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi& Di, std::vector<int>& stack, int local_maximum)
{
	stack.push_back(index);

	for (int k = delta[index]; k < delta[index + 1]; k++)
	{
		if (Di[k] != local_maximum) // avoid infinite loop
		{
			addToStackBraunWillett(Di[k], delta, Di, stack, local_maximum);
		}
	}
}

int G3PointAction::segmentLabelsBraunWillett(bool useParallelStrategy)
{
	std::cout << "[segment_labels]" << std::endl;

	bool steepestSlope = m_dlg->isSteepestSlope();

	// for each point, find in the neighborhood the point with the extreme slope, depending on the mode (the receiver)
	Eigen::ArrayXd extreme_slopes;
	if (steepestSlope)
	{
		std::cout << "[segment_labels] classical steepest slope algorithm [Braun, Willett 2013]" << std::endl;
		extreme_slopes = m_neighborsSlopes.rowwise().maxCoeff();
	}
	else
	{
		std::cout << "[segment_labels] reversed version of the steepest slope algorithm [Braun, Willett 2013]" << std::endl;
		extreme_slopes = m_neighborsSlopes.rowwise().minCoeff();
	}
	Eigen::ArrayXi index_of_extreme_slope = Eigen::ArrayXi::Zero(m_cloud->size());
	Eigen::ArrayXi receivers(m_cloud->size());

	for (unsigned index = 0; index < m_cloud->size(); index++)
	{
		double extreme_slope = extreme_slopes(index);
		for (int k = 0; k < m_kNN; k++)
		{
			if (m_neighborsSlopes(index, k) == extreme_slope)
			{
				index_of_extreme_slope(index) = k;
				break;
			}
		}
		receivers(index) = m_neighborsIndexes(index, index_of_extreme_slope(index));
	}

	// if the minimum slope is positive, the receiver is a local maximum
	int nb_maxima;
	if (steepestSlope)
	{
		nb_maxima = (extreme_slopes <= 0).count();
	}
	else
	{
		nb_maxima = (extreme_slopes >= 0).count();
	}
	m_initial_localMaximumIndexes = Eigen::ArrayXi::Zero(nb_maxima); // we need nb_maxima for the initialization
	int l = 0;
	for (unsigned int k = 0; k < m_cloud->size(); k++)
	{
		if (steepestSlope)
		{
			if (extreme_slopes(k) <= 0)
			{
				m_initial_localMaximumIndexes(l) = k;
				receivers(k) = k;
				l++;
			}
		}
		else
		{
			if (extreme_slopes(k) >= 0)
			{
				m_initial_localMaximumIndexes(l) = k;
				receivers(k) = k;
				l++;
			}
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

	int sfIdx = m_cloud->getScalarFieldIndexByName("g3point_initial_segmentation");
	if (sfIdx == -1)
	{
		sfIdx = m_cloud->addScalarField("g3point_initial_segmentation");
		if (sfIdx == -1)
		{
			ccLog::Error("[G3Point::segment_labels] impossible to create scalar field g3point_initial_segmentation");
		}
	}
	CCCoreLib::ScalarField* g3point_label = m_cloud->getScalarField(sfIdx);

	RGBAColorsTableType randomColors = getRandomColors(m_initial_localMaximumIndexes.size());

	if (!m_cloud->resizeTheRGBTable(false))
	{
		ccLog::Error(QObject::tr("Not enough memory!"));
		return -1;
	}

	for (int k = 0; k < m_initial_localMaximumIndexes.size(); k++)
	{
		int localMaximumIndex = m_initial_localMaximumIndexes(k);
		std::vector<int> stack;
		addToStackBraunWillett(localMaximumIndex, delta, Di, stack, localMaximumIndex);
		// labels
		for (auto i : stack)
		{
			m_initial_labels(i) = k;
			m_initial_labelsnpoint(i) = stack.size();
			if (g3point_label)
			{
				g3point_label->setValue(i, k);
				m_cloud->setPointColor(i, randomColors.getValue(k));
			}
		}
		m_initialStacks.push_back(stack);
	}

	if (g3point_label)
	{
		g3point_label->computeMinAndMax();
	}

	m_cloud->setCurrentDisplayedScalarField(sfIdx);
	m_cloud->showColors(true);
	m_cloud->showSF(false);

	m_cloud->redrawDisplay();
	m_cloud->prepareDisplayForRefresh();

	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}

	int nLabels = m_initial_localMaximumIndexes.size();

	return nLabels;
}

void G3PointAction::getNeighborsDistancesSlopes(unsigned index)
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
			m_neighborsIndexes(index, k) = Yk.getPointGlobalIndex(k + 1);
			// compute the distance to the neighbor
			const CCVector3* neighbor = Yk.getPoint(k + 1);
			float distance = (*P - *neighbor).norm();
			m_neighborsDistances(index, k) = distance;
			// compute the slope to the neighbor
			m_neighborsSlopes(index, k) = (P->z - neighbor->z) / distance;
		}
	}
}

void G3PointAction::computeNodeSurfaces()
{
	m_area = M_PI * m_neighborsDistances.rowwise().minCoeff().square();
}

bool G3PointAction::computeNormalsAndOrientThemWithCloudCompare()
{
	double radius = 1.; // one should set the radius value

	CCCoreLib::LOCAL_MODEL_TYPES model = CCCoreLib::QUADRIC;
	ccNormalVectors::Orientation  orientation = ccNormalVectors::Orientation::UNDEFINED;

	orientation = ccNormalVectors::Orientation::PLUS_Z;
	model = CCCoreLib::LOCAL_MODEL_TYPES::LS;

	QScopedPointer<ccProgressDialog> progressDialog(nullptr);

	if (!m_cloud->getOctree())
	{
		if (!m_cloud->computeOctree(progressDialog.data()))
		{
			ccLog::Error("Failed to compute octree for cloud " + m_cloud->getName());
			return false;
		}
	}

		float thisCloudRadius = radius;
		if (std::isnan(thisCloudRadius))
		{
			ccOctree::BestRadiusParams params;
			{
				params.aimedPopulationPerCell = 16;
				params.aimedPopulationRange = 4;
				params.minCellPopulation = 6;
				params.minAboveMinRatio = 0.97;
			}
			thisCloudRadius = ccOctree::GuessBestRadius(m_cloud, params, m_cloud->getOctree().data());
			if (thisCloudRadius == 0)
			{
				return ccLog::Error("Failed to determine best normal radius for cloud " + m_cloud->getName());
			}
			ccLog::Print("Cloud " + m_cloud->getName() + " radius = " + QString::number(thisCloudRadius));
		}

		ccLog::Print("computeNormalsWithOctree started...");
		bool success = m_cloud->computeNormalsWithOctree(model, orientation, thisCloudRadius, progressDialog.data());
		if(success)
		{
			ccLog::Print("computeNormalsWithOctree success");
		}
		else
		{
			ccLog::Error("computeNormalsWithOctree failed");
			return false;
		}

	return true;
}

void G3PointAction::orientNormals(const Eigen::Vector3d& sensorCenter)
{
	int pointCloud = m_cloud->size();

	for (int i = 0; i < pointCloud; i++)
	{
		const CCVector3 *point = m_cloud->getPoint(i);
		Eigen::Vector3d P1 = sensorCenter - Eigen::Vector3d(point->x, point->y, point->z);
		Eigen::Vector3d P2 = m_normals(i, Eigen::all);
		double angle = atan2(P1.cross(P2).norm(), P1.dot(P2));
		if ((angle < - M_PI / 2) || (angle > M_PI / 2))
		{
			m_normals(i, 0) = -m_normals(i, 0);
			m_normals(i, 1) = -m_normals(i, 1);
			m_normals(i, 2) = -m_normals(i, 2);
		}
	}
}

bool G3PointAction::computeNormalsWithOpen3D()
{
	// create an open3D point cloud from the original point cloud
	std::vector<Eigen::Vector3d> points(m_cloud->size());
	for (int index =0; index < points.size(); index++) // copy all points
	{
		points[index] = Eigen::Vector3d(m_cloud->getPoint(index)->x, m_cloud->getPoint(index)->y, m_cloud->getPoint(index)->z);
	}
	open3d::geometry::PointCloud pcd(points); // create the cloud

	// compute the normals
	pcd.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(m_kNN));

	for (int i = 0; i < 10; i++)
	{
		std::cout << pcd.normals_[i].x() << " " << pcd.normals_[i].y() << " " << pcd.normals_[i].z() << std::endl;
	}

	// we 'compress' each normal
	int pointCount = m_cloud->size();
	NormsIndexesTableType theNormsCodes = NormsIndexesTableType();
	if (!theNormsCodes.resizeSafe(pointCount))
	{
		ccLog::Error("[G3PointAction::compute_normals_and_orient_them_open3d] failed to allocate theNormCodes");
		return false;
	}
	std::fill(theNormsCodes.begin(), theNormsCodes.end(), 0);
	for (int index = 0; index < pointCount; index++)
	{
		CCVector3 N(pcd.normals_[index].x(), pcd.normals_[index].y(), pcd.normals_[index].z());
		CompressedNormType nCode = ccNormalVectors::GetNormIndex(N);
		theNormsCodes.setValue(index, nCode);
	}

	// preferred orientation
	ccNormalVectors::Orientation preferredOrientation = ccNormalVectors::PLUS_Z;
	ccNormalVectors::UpdateNormalOrientations(m_cloud, theNormsCodes, preferredOrientation);

	ccLog::Print("[G3PointAction::compute_normals_with_open3d] set the normals computed with Open3D to the point cloud");
	m_cloud->resizeTheNormsTable();

	//we hide normals during process
	m_cloud->showNormals(false);

	// set the normals
	for (unsigned j = 0; j < theNormsCodes.currentSize(); j++)
	{
		m_cloud->setPointNormalIndex(j, theNormsCodes.getValue(j));
	}

	for (int i = 0; i < 10; i++)
	{
		std::cout << m_cloud->getNormal(i)->x << " " << m_cloud->getNormal(i)->y << " " << m_cloud->getNormal(i)->z << std::endl;
	}

	for (int i = 0; i < pointCount; i++)
	{
		m_normals(i, 0) = pcd.normals_[i].x();
		m_normals(i, 1) = pcd.normals_[i].y();
		m_normals(i, 2) = pcd.normals_[i].z();
	}

	return true;
}

bool G3PointAction::queryNeighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy)
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
		QtConcurrent::blockingMap(pointsIndexes, [=](int index){getNeighborsDistancesSlopes(index);});
	}
	else
	{
		//manually call the static per-point method!
		for (unsigned i = 0; i < nPoints; ++i)
		{
			getNeighborsDistancesSlopes(i);
		}
	}

	return true;
}

void G3PointAction::segment()
{
	init();

	// Find neighbors of each point of the cloud
	queryNeighbors(m_cloud, m_app, true);

	computeNodeSurfaces();

	computeNormalsWithOpen3D();

	// compute the centroid
	int pointCount = m_cloud->size();
	CCVector3d G(0, 0, 0);
	{
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = m_cloud->getPoint(i);
			G += *P;
		}
		G /= pointCount;
	}

	Eigen::Vector3d sensorCenter(G.x, G.y, 1000);
	orientNormals(sensorCenter);

	// Perform initial segmentation
	int nLabels = segmentLabelsBraunWillett();

	exportLocalMaximaAsCloud(m_initial_localMaximumIndexes);

	m_app->dispToConsole( "[G3Point] initial segmentation: " + QString::number(nLabels) + " labels", ccMainAppInterface::STD_CONSOLE_MESSAGE );

	m_dlg->enableClusterAndOrClean(true);

	// in case we want to test the fitting of ellipsoids now, without clustering nor cleaning
	m_labels = m_initial_labels;
	m_labelsnpoint = m_initial_labelsnpoint;
	m_localMaximumIndexes = m_initial_localMaximumIndexes;
	m_stacks = m_initialStacks;
}

void G3PointAction::clusterAndOrClean()
{
	// initialize variables for clustering and / or cleaning  to the initial segmentation
	m_labels = m_initial_labels;
	m_labelsnpoint = m_initial_labelsnpoint;
	m_localMaximumIndexes = m_initial_localMaximumIndexes;
	m_stacks = m_initialStacks;

	if (m_dlg->clusterIsChecked())
	{
		if (!cluster())
		{
			ccLog::Error("[G3PointAction::clusterAndOrClean] clustering failed");
		}
	}

	if (m_dlg->cleanIsChecked())
	{
		if (!cleanLabels())
		{
			ccLog::Error("[G3PointAction::clusterAndOrClean] cleaning failed");
		}
	}
}

void G3PointAction::getBorders()
{
	// Find the indexborder nodes (no donor and many other labels in the neighbourhood)
	Eigen::ArrayXXi duplicatedLabelsInColumns(m_cloud->size(), m_kNN);
	for (int n = 0; n < m_kNN; n++)
	{
		duplicatedLabelsInColumns(Eigen::all, n) = m_labels;
	}
	Eigen::ArrayXXi labelsOfNeighbors(m_cloud->size(), m_kNN);
	for (int index = 0; index < m_cloud->size(); index++)
	{
		for (int n = 0; n < m_kNN; n++)
		{
			labelsOfNeighbors(index, n) = m_labels(m_neighborsIndexes(index, n));
		}
	}

	Eigen::ArrayXi temp = m_kNN - (labelsOfNeighbors == duplicatedLabelsInColumns).cast<int>().rowwise().sum();
	auto condition = ((temp >= m_kNN / 4) && (m_ndon == 0));

	// create cloud
	CCCoreLib::ReferenceCloud referenceCloud(m_cloud);
	for (int index = 0; index < condition.size(); index++)
	{
		if (condition(index))
		{
			referenceCloud.addPointIndex(index);
		}
	}

	ccPointCloud* borderCloud = m_cloud->partialClone(&referenceCloud);

	// add cloud to the database
	borderCloud->setName(m_cloud->getName() + "_borders");
	m_cloud->getParent()->addChild(borderCloud, ccHObject::DP_PARENT_OF_OTHER, 0);
	m_app->addToDB(borderCloud);
}

void G3PointAction::init()
{
	m_kNN = m_dlg->getkNN();

	// initialize the matrices which will contain the results
	m_neighborsIndexes = Eigen::ArrayXXi::Zero(m_cloud->size(), m_kNN);
	m_neighborsDistances = Eigen::ArrayXXd::Zero(m_cloud->size(), m_kNN);
	m_neighborsSlopes = Eigen::ArrayXXd::Zero(m_cloud->size(), m_kNN);
	m_normals = Eigen::ArrayXXd::Zero(m_cloud->size(), 3);

	m_labels = Eigen::ArrayXi::Zero(m_cloud->size());
	m_labelsnpoint = Eigen::ArrayXi::Zero(m_cloud->size());
	m_initial_labels = Eigen::ArrayXi::Zero(m_cloud->size());
	m_initial_labelsnpoint = Eigen::ArrayXi::Zero(m_cloud->size());

	m_stacks.clear(); // needed in case of several runs
	m_initialStacks.clear(); // needed in case of several runs
}

void G3PointAction::showDlg()
{
	if (!m_dlg)
	{
		m_dlg = new G3PointDialog(m_cloud->getName());

		connect(m_dlg, &G3PointDialog::segment, s_g3PointAction, &G3Point::G3PointAction::segment);
		connect(m_dlg, &G3PointDialog::clusterAndOrClean, s_g3PointAction, &G3Point::G3PointAction::clusterAndOrClean);
		connect(m_dlg, &G3PointDialog::getBorders, s_g3PointAction, &G3Point::G3PointAction::getBorders);

		connect(m_dlg, &G3PointDialog::fit, s_g3PointAction, &G3Point::G3PointAction::fit);

		connect(m_dlg, &QDialog::finished, s_g3PointAction, &G3Point::G3PointAction::clean);
		connect(m_dlg, &QDialog::finished, s_g3PointAction, &G3Point::G3PointAction::resetDlg); // dialog is defined with Qt::WA_DeleteOnClose
	}
	else
	{
		m_dlg->enableClusterAndOrClean(false);
	}

	m_dlg->show();
}

void G3PointAction::resetDlg()
{
	// dialog is defined with Qt::WA_DeleteOnClose, you have to reset the pointer when it's closed by the user
	m_dlg = nullptr;
}

void G3PointAction::clean()
{
	m_neighborsIndexes.resize(0, 0);
	m_neighborsDistances.resize(0, 0);
	m_neighborsSlopes.resize(0, 0);
	m_normals.resize(0, 0);

	m_labels.resize(0);
	m_labelsnpoint.resize(0);
	m_localMaximumIndexes.resize(0);
	m_ndon.resize(0);
	m_area.resize(0);

	m_stacks.clear();

	m_octree.clear();
}

void G3PointAction::setCloud(ccPointCloud *cloud)
{
	m_cloud = cloud;
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

	GetG3PointAction(ccHObjectCaster::ToPointCloud(ent), appInterface);
}

}
