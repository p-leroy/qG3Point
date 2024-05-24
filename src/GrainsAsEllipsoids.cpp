#include "GrainsAsEllipsoids.h"

/// qCC_db
#include <ccPointCloud.h>
#include <ccGLMatrix.h>
#include <ccSerializableObject.h>

#include <QCoreApplication>
#include <QDir>
#include <QOpenGLShaderProgram>

#include <iostream>

// from GeometricTools/GTE
// #include <Mathematics/DistPointHyperellipsoid.h>
// #include <Mathematics/Vector2.h>
// #include <Mathematics/Vector3.h>

GrainsAsEllipsoids::GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface *app, const std::vector<std::vector<int> >& stacks, const RGBAColorsTableType& colors)
	: m_cloud(cloud)
	, m_app(app)
	, m_stacks(stacks)
{
	this->setMetaData("class_name", "GrainsAsEllipsoids");
	this->setMetaData("plugin_name", "G3PointPlugin");

	setShaderPath();
	setGrainColorsTable(colors);

	m_center.resize(m_stacks.size());
	m_radii.resize(m_stacks.size());
	m_rotationMatrix.resize(m_stacks.size());

	// fit all ellipsoids
	std::cout << "[GrainsAsEllipsoids::GrainsAsEllipsoids] fit " << stacks.size() << " ellipsoids" << std::endl;

	lockVisibility(false);
	setVisible(true);

	m_ccBBoxAll.setValidity(false);
	m_ccBBoxAll.clear();

	for (int idx = 0; idx < m_stacks.size(); idx++)
	{
		if (!fitEllipsoidToGrain(idx, m_center[idx], m_radii[idx], m_rotationMatrix[idx]))
		{
			m_fitNotOK.insert(idx);
			ccLog::Warning("[GrainsAsEllipsoids::GrainsAsEllipsoids] fit not possible for grain " + QString::number(idx)
						   + " of size " + QString::number(m_stacks[idx].size()));
		}
		else // update the bounding box
		{
			float maxRadius = m_radii[idx].maxCoeff();
			CCVector3 center(m_center[idx](0), m_center[idx](1), m_center[idx](2));
			m_ccBBoxAll.add(CCVector3(center.x + maxRadius, center.y + maxRadius, center.z + maxRadius));
			m_ccBBoxAll.add(CCVector3(center.x - maxRadius, center.y - maxRadius, center.z - maxRadius));
		}
	}

	m_ccBBoxAll.setValidity(true);
}

void GrainsAsEllipsoids::setShaderPath()
{
	QDir appDir = QCoreApplication::applicationDirPath();
	m_shaderPath = (appDir.absolutePath() + "/shaders/G3Point");
}

void GrainsAsEllipsoids::setGrainColorsTable(const RGBAColorsTableType& colorTable)
{
	m_grainColors.resize(colorTable.size());

	for (int k = 0; k < colorTable.size(); k++)
	{
		ccColor::Rgba color = colorTable[k];
		m_grainColors[k] = CCVector3f(static_cast<float>(color.r) / ccColor::MAX,
									  static_cast<float>(color.g) / ccColor::MAX,
									  static_cast<float>(color.b) / ccColor::MAX);
	}
}

bool GrainsAsEllipsoids::exportResultsAsCloud()
{
	// create cloud
	QString cloudName = "g3point_results";
	ccPointCloud *cloud = new ccPointCloud(cloudName);

	for (int idx = 0; idx < m_center.size(); idx++)
	{
		if (m_fitNotOK.count(idx))
		{
			continue;
		}
		Eigen::Vector3f center {m_center[idx].x(), m_center[idx].y(), m_center[idx].z()};
		Eigen::Vector3f point = center;
		CCVector3 ccPoint(point(0), point(1), point(2));
		cloud->addPoint(ccPoint);
	}

	//allocate colors if necessary
	if (cloud->resizeTheRGBTable())
	{
		for (unsigned int index = 0; index < cloud->size(); index++)
		{
			ccColor::Rgb color(m_grainColors[index].x * ccColor::MAX * 0.8,
							   m_grainColors[index].y * ccColor::MAX * 0.8,
							   m_grainColors[index].z * ccColor::MAX * 0.8);
			cloud->setPointColor(index, color);
		}
	}

	int sfIdx;
	CCCoreLib::ScalarField* sf;

	// EXPORT g3point_index
	sfIdx = cloud->addScalarField("g3point_index");
	if (sfIdx == -1)
	{
		ccLog::Error("[GrainsAsEllipsoids::exportResultsAsCloud] impossible to allocate g3point_index scalar field");
		return false;
	}
	sf = cloud->getScalarField(sfIdx);
	int indexInResults = 0;
	for (int index = 0; index < m_center.size(); index++)
	{
		if (m_fitNotOK.count(index)) // when the fit was not successful, the point is not exported
		{
			continue;
		}
		sf->setValue(indexInResults, index);
		indexInResults++;
	}
	sf->computeMinAndMax();

	// <EXPORT RADII>
	int sfIdxRadiusX = cloud->addScalarField("g3point_radius_x");
	int sfIdxRadiusY = cloud->addScalarField("g3point_radius_y");
	int sfIdxRadiusZ = cloud->addScalarField("g3point_radius_z");
	if (sfIdxRadiusX == -1 || sfIdxRadiusY == -1 || sfIdxRadiusZ == -1)
	{
		ccLog::Error("[GrainsAsEllipsoids::exportResultsAsCloud] impossible to allocate scalar fields to export the radii");
		return false;
	}
	CCCoreLib::ScalarField* sfRadiusX = cloud->getScalarField(sfIdxRadiusX);
	CCCoreLib::ScalarField* sfRadiusY = cloud->getScalarField(sfIdxRadiusY);
	CCCoreLib::ScalarField* sfRadiusZ = cloud->getScalarField(sfIdxRadiusZ);
	for (unsigned int index = 0; index < cloud->size(); index++)
	{
		if (m_fitNotOK.count(index))
		{
			continue;
		}
		sfRadiusX->setValue(index, m_radii[index].x());
		sfRadiusY->setValue(index, m_radii[index].y());
		sfRadiusZ->setValue(index, m_radii[index].z());
	}
	sfRadiusX->computeMinAndMax();
	sfRadiusY->computeMinAndMax();
	sfRadiusZ->computeMinAndMax();
	// </EXPORT RADII>

	// <EXPORT ROTATION>
	int sfIdxR00 = cloud->addScalarField("g3point_r00");
	int sfIdxR01 = cloud->addScalarField("g3point_r01");
	int sfIdxR02 = cloud->addScalarField("g3point_r02");
	int sfIdxR10 = cloud->addScalarField("g3point_r10");
	int sfIdxR11 = cloud->addScalarField("g3point_r11");
	int sfIdxR21 = cloud->addScalarField("g3point_r12");
	int sfIdxR20 = cloud->addScalarField("g3point_r20");
	int sfIdxR12 = cloud->addScalarField("g3point_r21");
	int sfIdxR22 = cloud->addScalarField("g3point_r22");
	if (sfIdxR00 == -1 || sfIdxR01 == -1 || sfIdxR02 == -1
		|| sfIdxR10 == -1 || sfIdxR11 == -1 || sfIdxR12 == -1
		|| sfIdxR20 == -1 || sfIdxR21 == -1 || sfIdxR22 == -1)
	{
		ccLog::Error("[GrainsAsEllipsoids::exportResultsAsCloud] impossible to allocate scalar fields to export the rotation");
		return false;
	}
	CCCoreLib::ScalarField* sfR00 = cloud->getScalarField(sfIdxR00);
	CCCoreLib::ScalarField* sfR01 = cloud->getScalarField(sfIdxR01);
	CCCoreLib::ScalarField* sfR02 = cloud->getScalarField(sfIdxR02);
	CCCoreLib::ScalarField* sfR10 = cloud->getScalarField(sfIdxR10);
	CCCoreLib::ScalarField* sfR11 = cloud->getScalarField(sfIdxR11);
	CCCoreLib::ScalarField* sfR12 = cloud->getScalarField(sfIdxR12);
	CCCoreLib::ScalarField* sfR20 = cloud->getScalarField(sfIdxR20);
	CCCoreLib::ScalarField* sfR21 = cloud->getScalarField(sfIdxR21);
	CCCoreLib::ScalarField* sfR22 = cloud->getScalarField(sfIdxR22);
	for (unsigned int index = 0; index < cloud->size(); index++)
	{
		if (m_fitNotOK.count(index))
		{
			continue;
		}
		sfR00->setValue(index, m_rotationMatrix[index](0, 0));
		sfR01->setValue(index, m_rotationMatrix[index](0, 1));
		sfR02->setValue(index, m_rotationMatrix[index](0, 2));
		sfR10->setValue(index, m_rotationMatrix[index](1, 0));
		sfR11->setValue(index, m_rotationMatrix[index](1, 1));
		sfR12->setValue(index, m_rotationMatrix[index](1, 2));
		sfR20->setValue(index, m_rotationMatrix[index](2, 0));
		sfR21->setValue(index, m_rotationMatrix[index](2, 1));
		sfR22->setValue(index, m_rotationMatrix[index](2, 2));
	}
	sfR00->computeMinAndMax();
	sfR01->computeMinAndMax();
	sfR02->computeMinAndMax();
	sfR10->computeMinAndMax();
	sfR11->computeMinAndMax();
	sfR12->computeMinAndMax();
	sfR20->computeMinAndMax();
	sfR21->computeMinAndMax();
	sfR22->computeMinAndMax();
	// </EXPORT ROTATION>

	cloud->showColors(true);
	cloud->setPointSize(9);

	m_cloud->getParent()->addChild(cloud, ccHObject::DP_PARENT_OF_OTHER, 0);
	m_app->addToDB(cloud);

	return true;
}

// INIT ORIGINAL SPHERE

void GrainsAsEllipsoids::initSphereVertices()
{
	// clear memory of prev arrays
	std::vector<float>().swap(vertices);
	std::vector<float>().swap(normals);
	std::vector<float>().swap(texCoords);

	float x, y, z, xy;                              // vertex position
	float nx, ny, nz;    // vertex normal
	float s, t;                                     // vertex texCoord

	float sectorStep = 2 * M_PI / sectorCount;
	float stackStep = M_PI / stackCount;
	float sectorAngle, stackAngle;

	for(int i = 0; i <= stackCount; ++i)
	{
		stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
		xy = cosf(stackAngle);             // r * cos(u)
		z = sinf(stackAngle);              // r * sin(u)

		// add (sectorCount+1) vertices per stack
		// first and last vertices have same position and normal, but different tex coords
		for(int j = 0; j <= sectorCount; ++j)
		{
			sectorAngle = j * sectorStep;           // starting from 0 to 2pi

			// vertex position (x, y, z)
			x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
			y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z);

			// normalized vertex normal (nx, ny, nz)
			nx = x;
			ny = y;
			nz = z;
			normals.push_back(nx);
			normals.push_back(ny);
			normals.push_back(nz);

			// vertex tex coord (s, t) range between [0, 1]
			s = (float)j / sectorCount;
			t = (float)i / stackCount;
			texCoords.push_back(s);
			texCoords.push_back(t);
		}
	}
}

void GrainsAsEllipsoids::initSphereIndexes()
{
	// generate CCW index list of sphere triangles
	// k1--k1+1
	// |  / |
	// | /  |
	// k2--k2+1

	int k1, k2;
	for(int i = 0; i < stackCount; ++i)
	{
		k1 = i * (sectorCount + 1);     // beginning of current stack
		k2 = k1 + sectorCount + 1;      // beginning of next stack

		for(int j = 0; j < sectorCount; ++j, ++k1, ++k2)
		{
			// 2 triangles per sector excluding first and last stacks
			// k1 => k2 => k1+1
			if(i != 0)
			{
				indices.push_back(k1);
				indices.push_back(k2);
				indices.push_back(k1 + 1);
			}

			// k1+1 => k2 => k2+1
			if(i != (stackCount-1))
			{
				indices.push_back(k1 + 1);
				indices.push_back(k2);
				indices.push_back(k2 + 1);
			}

			// store indices for lines
			// vertical lines for all stacks, k1 => k2
			lineIndices.push_back(k1);
			lineIndices.push_back(k2);
			if(i != 0)  // horizontal lines except 1st stack, k1 => k+1
			{
				lineIndices.push_back(k1);
				lineIndices.push_back(k1 + 1);
			}
		}
	}
}

// ELLIPSOID FITTING

double GrainsAsEllipsoids::ellipsoidDistance(const Eigen::ArrayXd& p, int idx)
{
	// // Compute the mean distance between the points of the grain and the ellipsoid

	// // GTE Geometric Tools Engine

	// // center
	// gte::Vector3<double> center = {m_center[idx](0), m_center[idx](1), m_center[idx](2)};

	// // axis
	// std::array<gte::Vector3<double>, 3> axis;
	// axis[0] = {m_rotationMatrix[idx](0, 0), m_rotationMatrix[idx](1, 0), m_rotationMatrix[idx](2, 0)};
	// axis[1] = {m_rotationMatrix[idx](0, 1), m_rotationMatrix[idx](1, 1), m_rotationMatrix[idx](2, 1)};
	// axis[2] = {m_rotationMatrix[idx](0, 2), m_rotationMatrix[idx](1, 2), m_rotationMatrix[idx](2, 2)};

	// // extent
	// gte::Vector3<double> extent = {m_radii[idx](0), m_radii[idx](1), m_radii[idx](2)};

	// // create the ellipsoid
	// gte::Ellipsoid3<double> ellipsoid(center, axis, extent);

	// gte::DCPQuery<double, gte::Vector3<double>, gte::Ellipsoid3<double>> query;

	// std::vector<int> stack = m_stacks[idx];
	// double sum_a = 0; // distances with respect to the ellipsoid
	// double sum_b = 0; // distances with respect to the mean

	// //compute gravity center
	// size_t count = m_cloud->size();
	// CCVector3 mean(0, 0, 0);
	// for (int index : m_stacks[idx])
	// {
	// 	const CCVector3* P = m_cloud->getPoint(index);
	// 	mean.x += P->x;
	// 	mean.y += P->y;
	// 	mean.z += P->z;
	// }
	// mean.x = mean.x / count;
	// mean.y = mean.y / count;
	// mean.z = mean.z / count;

	// for (int index : stack)
	// {
	// 	const CCVector3 *P = m_cloud->getPoint(index);
	// 	gte::Vector3<double> P_gte = {P->x, P->y, P->z};
	// 	auto result = query(P_gte, ellipsoid);
	// 	sum_a = sum_a + pow(result.distance, 2);

	// 	CCVector3 P_minus_min = *P - CCVector3(mean.x, mean.y, mean.z);
	// 	sum_b = sum_b + P_minus_min.norm2d();
	// }

	// double r2 = 1 - sum_a / sum_b;

	// // in Matlab
	// // d = (x-xp).^2 + (y-yp).^2 + (z-zp).^2;
	// // r2 = 1 - sum((x-xp).^2 + (y-yp).^2 + (z-zp).^2)./sum((x-mean(x)).^2 + (y-mean(y)).^2 + (z-mean(z)).^2);

	// return r2;
	return 0.;
}

void GrainsAsEllipsoids::updateBBoxOnlyOne(int index)
{
	m_ccBBoxOnlyOne.setValidity(false);
	m_ccBBoxOnlyOne.clear();
	if (index < m_stacks.size())
	{
		if (m_fitNotOK.count(index) == 0)
		{
			float maxRadius = m_radii[index].maxCoeff();
			CCVector3 center(m_center[index](0), m_center[index](1), m_center[index](2));
			m_ccBBoxOnlyOne.add(CCVector3(center.x + maxRadius, center.y + maxRadius, center.z + maxRadius));
			m_ccBBoxOnlyOne.add(CCVector3(center.x - maxRadius, center.y - maxRadius, center.z - maxRadius));
			m_ccBBoxOnlyOne.setValidity(true);
		}
	}
	else
	{
		ccLog::Error("[GrainsAsEllipsoids::updateBBox] asking for the bounding of index " + QString::number(index) + " out of range");
	}
}

bool GrainsAsEllipsoids::explicitToImplicit(const Eigen::Array3f& center,
													  const Eigen::Array3f& radii,
													  const Eigen::Matrix3f& rotationMatrix,
													  Eigen::ArrayXd& parameters)
{
	float xrr = 1 / radii(0);
	float yrr = 1 / radii(1);
	float zrr = 1 / radii(2);

	float r11 = rotationMatrix.data()[0];
	float r21 = rotationMatrix.data()[1];
	float r31 = rotationMatrix.data()[2];
	float r12 = rotationMatrix.data()[3];
	float r22 = rotationMatrix.data()[4];
	float r32 = rotationMatrix.data()[5];
	float r13 = rotationMatrix.data()[6];
	float r23 = rotationMatrix.data()[7];
	float r33 = rotationMatrix.data()[8];

	float xc = center(0);
	float yc = center(1);
	float zc = center(2);

	// terms collected from symbolic expression

	parameters << pow(r11, 2) * pow(xrr, 2) + pow(r21, 2) * pow(yrr, 2) + pow(r31, 2) * pow(zrr, 2),
		pow(r12, 2) * pow(xrr, 2) + pow(r22, 2) * pow(yrr, 2) + pow(r32, 2) * pow(zrr, 2),
		pow(r13, 2) * pow(xrr, 2) + pow(r23, 2) * pow(yrr, 2) + pow(r33, 2) * pow(zrr, 2),
		2 * r11 * r12 * pow(xrr, 2) + 2 * r21 * r22 * pow(yrr, 2) + 2 * r31 * r32 * pow(zrr, 2),
		2 * r11 * r13 * pow(xrr, 2) + 2 * r21 * r23 * pow(yrr, 2) + 2 * r31 * r33 * pow(zrr, 2),
		2 * r12 * r13 * pow(xrr, 2) + 2 * r22 * r23 * pow(yrr, 2) + 2 * r32 * r33 * pow(zrr, 2),
		(-2) * (pow(r11, 2) * xc * pow(xrr, 2) + pow(r21, 2) * xc * pow(yrr, 2) + pow(r31, 2) * xc * pow(zrr, 2)
				+ r11 * r12 * pow(xrr, 2) * yc
				+ r11 * r13 * pow(xrr, 2) * zc
				+ r21 * r22 * yc * pow(yrr, 2)
				+ r21 * r23 * pow(yrr, 2) * zc
				+ r31 * r32 * yc * pow(zrr, 2)
				+ r31 * r33 * zc * pow(zrr, 2)),
		(-2) * (pow(r12, 2) * pow(xrr, 2) * yc + pow(r22, 2) * yc * pow(yrr, 2) + pow(r32, 2) * yc * pow(zrr, 2)
				+ r11 * r12 * xc * pow(xrr, 2)
				+ r21 * r22 * xc * pow(yrr, 2)
				+ r12 * r13 * pow(xrr, 2) * zc
				+ r31 * r32 * xc * pow(zrr, 2)
				+ r22 * r23 * pow(yrr, 2) * zc
				+ r32 * r33 * zc * pow(zrr, 2)),
		(-2) * (pow(r13, 2)*pow(xrr, 2) * zc + pow(r23, 2) * pow(yrr, 2) * zc + pow(r33, 2) * zc * pow(zrr, 2)
				+ r11 * r13 * xc * pow(xrr, 2)
				+ r12 * r13 * pow(xrr, 2) * yc
				+ r21 * r23 * xc * pow(yrr, 2)
				+ r22 * r23 * yc * pow(yrr, 2)
				+ r31 * r33 * xc * pow(zrr, 2)
				+ r32 * r33 * yc * pow(zrr, 2)),
		pow(r11, 2) * pow(xc, 2) * pow(xrr, 2)
			+ 2 * r11 * r12 * xc * pow(xrr, 2) * yc
			+ 2 * r11 * r13 * xc * pow(xrr, 2) * zc
			+ pow(r12, 2) * pow(xrr, 2) * pow(yc, 2)
			+ 2 * r12 * r13 * pow(xrr, 2) * yc * zc
			+ pow(r13, 2) * pow(xrr, 2) * pow(zc, 2)
			+ pow(r21, 2) *pow(xc, 2) * pow(yrr, 2)
			+ 2 * r21 * r22 * xc * yc * pow(yrr, 2)
			+ 2 * r21 * r23 * xc * pow(yrr, 2) * zc
			+ pow(r22, 2) * pow(yc, 2) * pow(yrr, 2)
			+ 2 * r22 * r23 * yc * pow(yrr, 2) * zc
			+ pow(r23, 2) * pow(yrr, 2) * pow(zc, 2)
			+ pow(r31, 2) * pow(xc, 2) * pow(zrr, 2)
			+ 2 * r31 * r32 * xc * yc * pow(zrr, 2)
			+ 2 * r31 * r33 * xc * zc * pow(zrr, 2)
			+ pow(r32, 2) * pow(yc, 2) * pow(zrr, 2)
			+ 2 * r32 * r33 * yc * zc * pow(zrr, 2)
			+ pow(r33, 2) * pow(zc, 2) * pow(zrr, 2) - 1;

	return true;
}

bool GrainsAsEllipsoids::implicitToExplicit(const Eigen::ArrayXd& parameters,
											Eigen::Array3f& center,
											Eigen::Array3f& radii,
											Eigen::Matrix3f& rotationMatrix)
{
	Eigen::ArrayXd p = parameters;

	p(3) = 0.5 * p(3);
	p(4) = 0.5 * p(4);
	p(5) = 0.5 * p(5);
	p(6) = 0.5 * p(6);
	p(7) = 0.5 * p(7);
	p(8) = 0.5 * p(8);

	Eigen::MatrixXd q(4, 4);

	q << p(0), p(3), p(4), p(6)
		, p(3), p(1), p(5), p(7)
		, p(4), p(5), p(2), p(8)
		, p(6), p(7), p(8), p(9);

	center = q.block(0, 0, 3, 3).colPivHouseholderQr().solve(-p(Eigen::seq(6, 8)).matrix()).cast<float>();

	Eigen::MatrixXd t(4, 4);
	t = Eigen::MatrixXd::Identity(4, 4);
	t(3, 0) = center(0);
	t(3, 1) = center(1);
	t(3, 2) = center(2);

	Eigen::MatrixXd s(4, 4);
	s = t * q * t.transpose();

	// check for positive definiteness
	Eigen::LLT<Eigen::MatrixXd> lltOfA((-s(3, 3) * s.block(0, 0, 3, 3).array()));
	if (lltOfA.info() != Eigen::Success)
	{
		return false;
	}

	Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(s.block(0, 0, 3, 3));
	if (eigensolver.info() != Eigen::Success)
	{
		return false;
	}

	radii = (-s(3, 3) / eigensolver.eigenvalues().array().real()).sqrt().cast<float>();
	rotationMatrix = eigensolver.eigenvectors().transpose().real().cast<float>();

	return true;
}

bool GrainsAsEllipsoids::directFit(const Eigen::ArrayX3d& xyz, Eigen::ArrayXd& parameters)
{
	Eigen::MatrixXd d(xyz.rows(), 10);

	d << xyz(Eigen::all, 0).pow(2).matrix()
		, xyz(Eigen::all, 1).pow(2).matrix()
		, xyz(Eigen::all, 2).pow(2).matrix()
		, (2 * xyz(Eigen::all, 1) * xyz(Eigen::all, 2)).matrix()
		, (2 * xyz(Eigen::all, 0) * xyz(Eigen::all, 2)).matrix()
		, (2 * xyz(Eigen::all, 0) * xyz(Eigen::all, 1)).matrix()
		, (2 * xyz(Eigen::all, 0)).matrix()
		, (2 * xyz(Eigen::all, 1)).matrix()
		, (2 * xyz(Eigen::all, 2)).matrix()
		, Eigen::MatrixXd::Ones(xyz.rows(), 1);

	Eigen::MatrixXd s = d.transpose() * d;

	int k = 4;
	Eigen::Matrix3d c1;
	Eigen::Matrix3d c2;
	Eigen::MatrixXd c;
	c = Eigen::MatrixXd::Zero(10, 10);
	c1 << 0 , k , k
		, k, 0, k
		, k , k , 0;
	c1 = c1.array() / 2 - 1;
	c2 = - k * Eigen::Matrix3d::Identity();
	c.block(0, 0, 3, 3) = c1;
	c.block(3, 3, 3, 3) = c2;

	Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> eigensolver(s, c);
	if (eigensolver.info() != Eigen::Success)
	{
		return false;
	}

	Eigen::ArrayXd eigenValues(10);
	Eigen::VectorXd eigenValuesAsAMatrix(10);
	eigenValues = eigensolver.eigenvalues().real();
	eigenValuesAsAMatrix = eigensolver.eigenvalues().real().matrix();
	Xb condition = (eigenValues > 0) && (!eigenValues.isInf());

	int flt = condition.count();
//	std::cout << "flt " << flt << std::endl;
	Eigen::ArrayXd finiteValues(flt);
	finiteValues = Eigen::ArrayXd::Zero(flt);
	int finiteValuesCounter = 0;
	for (int k = 0; k < eigenValues.size(); k++)
	{
		if (condition(k))
		{
			finiteValues(finiteValuesCounter++) = eigenValues(k);
		}
	}

	double eigenValue;
	Eigen::MatrixXd v;
	switch (flt) {
	case 1: // regular case
		eigenValue = finiteValues(0); // there is only one positive finite value
		for (k = 0; k < 10; k++)
		{
			if (eigenValues(k) == eigenValue)
			{
				v = eigensolver.eigenvectors()(Eigen::all, k).real();
				break;
			}
		}
		break;
	case 0: // degenerate case
		// # single positive eigenvalue becomes near-zero negative eigenvalue due to round-off error
		eigenValue = eigenValues.abs().minCoeff();
		for (k = 0; k < 10; k++)
		{
			if (abs(eigenValues(k)) == eigenValue)
			{
				v = eigensolver.eigenvectors()(Eigen::all, k).real();
				break;
			}
		}
		break;
	default: // degenerate case
		// several positive eigenvalues appear
		eigenValue = finiteValues.abs().minCoeff();
		for (k = 0; k < 10; k++)
		{
			if (eigenValues(k) == eigenValue)
			{
				v = eigensolver.eigenvectors()(Eigen::all, k).real();
				break;
			}
		}
		break;
	}

	parameters.resize(10);

	parameters << v(0), v(1), v(2)
		, 2 * v(5), 2 * v(4), 2* v(3)
		, 2 * v(6), 2 * v(7), 2 * v(8)
		, v(9);

	return true;
}

bool GrainsAsEllipsoids::fitEllipsoidToGrain(const int grainIndex,
											 Eigen::Array3f& center,
											 Eigen::Array3f& radii,
											 Eigen::Matrix3f& rotationMatrix,
											 const Method& method)
{
	// Shift point cloud to have only positive coordinates
	// (problem with quadfit if the point cloud is far from the coordinates of the origin (0,0,0))

	bool ret = true;

	// extract the point cloud related to the current index
	CCCoreLib::ReferenceCloud referenceCloud(m_cloud);
	for (int index : m_stacks[grainIndex])
	{
		referenceCloud.addPointIndex(index);
	}

	ccPointCloud* grainCloud = m_cloud->partialClone(&referenceCloud);
	Eigen::Map<const Eigen::MatrixX3f, Eigen::Unaligned, Eigen::Stride<1, 3>>
		grainPoints(static_cast<const float*>(grainCloud->getPoint(0)->u), grainCloud->size(), 3);

	CCVector3 bbMin;
	CCVector3 bbMax;
	grainCloud->getBoundingBox(bbMin, bbMax);
	CCVector3 bb(bbMax - bbMin);
	Eigen::Vector3d scales(bb.x, bb.y, bb.z);
	double scale = 1 / scales.maxCoeff();
	Eigen::RowVector3d means = grainPoints.cast<double>().colwise().mean();

	Eigen::ArrayXd p(10);

	switch (method) {
	case DIRECT:
		// Direct least squares fitting of ellipsoids under the constraint 4J - I**2 > 0.
		// The constraint confines the class of ellipsoids to fit to those whose smallest radius is at least half of the
		// largest radius.

		if(!directFit(scale * (grainPoints.cast<double>().rowwise() - means), p)) // Ellipsoid fit
		{
			return false;
		}

		if (!implicitToExplicit(p, center, radii, rotationMatrix)) // Get the explicit parameters
		{
			return false;
		}

		break;
	default:
		break;
	}

	// Rescale the explicit parameters (the rotation matrix is unchanged by the scaling)
	center = center / scale + Eigen::Array3f(means.cast<float>());
	radii = radii / scale;

	// re-order the radii
	std::vector<float> sortedRadii{radii(0), radii(1), radii(2)};
	std::sort(sortedRadii.begin(), sortedRadii.end());
	Eigen::Array3f updatedRadii = {sortedRadii[0], sortedRadii[1], sortedRadii[2]}; // from the smallest to the largest
	Eigen::Matrix3f updatedRotationMatrix;
	for (int k = 0; k < 3; k++)
	{
		float radius = updatedRadii(k);
		int col = 0;
		for (int idx = 0; idx < 3; idx++)
		{
			if (radii[idx] == radius)
			{
				break;
			}
			col++;
		}
		updatedRotationMatrix(k, 0) = rotationMatrix(col, 0);
		updatedRotationMatrix(k, 1) = rotationMatrix(col, 1);
		updatedRotationMatrix(k, 2) = rotationMatrix(col, 2);
	}

	radii = updatedRadii;
	rotationMatrix = updatedRotationMatrix;

	ret = explicitToImplicit(center, radii, rotationMatrix, p);

	return ret;
}

// DRAW

void GrainsAsEllipsoids::releaseShaders()
{
	m_program.clear();
}

void GrainsAsEllipsoids::setUniformValueColor(const ccColor::Rgba &color)
{
	m_program->setUniformValue("color", color.r, color.g, color.b, color.a);
}

bool GrainsAsEllipsoids::initProgram(QOpenGLContext* context)
{
	if (m_program.isNull())
	{
		QString error;

		if (!context)
		{
			assert(false);
			return false;
		}

		m_program.reset(new QOpenGLShaderProgram(context));

		// create vertex shader
		QString vertexShaderFile(m_shaderPath + "/DrawGrains.vs");
		if (!m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, vertexShaderFile))
		{
			error = m_program->log();
			ccLog::Error(error);
			return false;
		}

		// create fragment shader
		QString fragmentShaderFile(m_shaderPath + "/DrawGrains.fs");
		if (!m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, fragmentShaderFile))
		{
			error = m_program->log();
			ccLog::Error(error);
			return false;
		}

		if (!m_program->link())
		{
			error = m_program->log();
			ccLog::Error(error);
			return false;
		}

		initSphereVertices();
		initSphereIndexes();
	}

	return true;
}

void GrainsAsEllipsoids::drawEllipsoid(CC_DRAW_CONTEXT& context, int idx)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (!m_fitNotOK.count(idx))
	{
		QMatrix4x4 projection;
		QMatrix4x4 modelView;
		QMatrix4x4 model;
		QMatrix4x4 matrixNormal;

		Eigen::Matrix3f rotation(m_rotationMatrix[idx].transpose());
		QMatrix4x4 matrixFromFit(rotation(0, 0), rotation(0, 1), rotation(0, 2), m_center[idx](0),
								 rotation(1, 0), rotation(1, 1), rotation(1, 2), m_center[idx](1),
								 rotation(2, 0), rotation(2, 1), rotation(2, 2), m_center[idx](2),
								 0, 0, 0, 1);

		CCVector3f color;

		color = m_grainColors[idx];
		glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glFunc->glEnable(GL_BLEND);
		m_program->setUniformValue("materialAmbient", color.x, color.y, color.z, 1.);
		m_program->setUniformValue("materialDiffuse", color.x, color.y, color.z, 1.);
		m_program->setUniformValue("objectColor", color.x, color.y, color.z);

		// prepare translation, rotation and scaling
		glFunc->glPushMatrix(); // save the current matrix

		// rotation and translation from the ellipsoid fitting
		glFunc->glMultMatrixf(matrixFromFit.data());
		// scale from the ellipsoid fitting
		glFunc->glScalef(m_radii[idx](0), m_radii[idx](1), m_radii[idx](2));

		// get matrices
		glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
		glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());
		matrixNormal = modelView;
		matrixNormal.setColumn(3, QVector4D(0,0,0,1));
		m_program->setUniformValue("modelViewMatrix", modelView);
		m_program->setUniformValue("normalMatrix", matrixNormal);
		m_program->setUniformValue("modelViewProjectionMatrix", projection * modelView);

		// draw triangles
		if (m_drawSurfaces)
		{
			m_program->setUniformValue("drawLines", 0);
			m_program->setUniformValue("drawPoints", 0);

			glFunc->glEnable(GL_POLYGON_OFFSET_FILL);

			glFunc->glPolygonOffset(1.0, 1.0f); // move polygon backward

			glFunc->glDrawElements(GL_TRIANGLES, (unsigned int) indices.size(), GL_UNSIGNED_INT, indices.data());

			glFunc->glDisable(GL_POLYGON_OFFSET_FILL);
		}

		// draw lines
		if (m_drawLines)
		{
			m_program->setUniformValue("drawLines", 1);
			m_program->setUniformValue("drawPoints", 0);

			glFunc->glDrawElements(GL_LINES, (unsigned int)lineIndices.size(), GL_UNSIGNED_INT, lineIndices.data());
		}

		glFunc->glPopMatrix();
	}
}

bool GrainsAsEllipsoids::drawEllipsoids(CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	CCVector3f color;

	// set uniforms
	QVector4D lightPosition(2 * m_ccBBoxAll.maxCorner().x, 2 * m_ccBBoxAll.maxCorner().y, 10 * m_ccBBoxAll.maxCorner().z, 0);
	QVector4D lightAmbient(0.8f, 0.8f, 0.8f, 1); // grey
	QVector4D lightDiffuse(0.8f, 0.8f, 0.8f, 1); // light grey
	QVector4D lightSpecular(1.0f, 1.0f, 1.0f, 1); // white
	QVector4D materialDiffuse(0.7f, 0.7f, 0.7f, m_transparency);
	QVector4D materialSpecular(0.4f, 0.4f, 0.4f, 1);
	float materialShininess  = 16;

	m_program->setUniformValue("lightPosition", lightPosition);
	m_program->setUniformValue("lightAmbient", lightAmbient);
	m_program->setUniformValue("lightDiffuse", lightDiffuse);
	m_program->setUniformValue("lightSpecular", lightSpecular);
	m_program->setUniformValue("materialSpecular", materialSpecular);
	m_program->setUniformValue("materialShininess", materialShininess);

	m_program->setAttributeArray("vertexPosition", static_cast<GLfloat*>(vertices.data()), 3);
	m_program->setAttributeArray("vertexNormal", static_cast<GLfloat*>(normals.data()), 3);
	m_program->setAttributeArray("vertexTexCoord", static_cast<GLfloat*>(texCoords.data()), 2);

	m_program->enableAttributeArray("vertexPosition");
	m_program->enableAttributeArray("vertexNormal");
	m_program->enableAttributeArray("vertexTexCoord");

	QMatrix4x4 projection;
	QMatrix4x4 modelView;

	if (m_showAll)
	{
		for (int idx = 0; idx < m_stacks.size(); idx++)
		{
			drawEllipsoid(context, idx);
		}
	}
	else
	{
		drawEllipsoid(context, m_onlyOne);

		// draw points
		if (m_drawPoints)
		{
			// get matrices
			glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
			glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());

			m_program->setUniformValue("modelViewProjectionMatrix", projection * modelView);
			m_program->setUniformValue("drawLines", 0);
			m_program->setUniformValue("drawPoints", 1);
			m_program->setUniformValue("materialAmbient", 1, 1, 1, 1.);

			std::vector<int> stack = m_stacks[m_onlyOne];
			std::vector<std::array<GLfloat, 3>> points(stack.size());
			std::vector<int> indices;
			for (int k = 0; k < stack.size(); k++)
			{
				const CCVector3* P = m_cloud->getPoint(stack[k]);
				points[k] = {P->x, P->y, P->z};
			}

			// change the vertex positions to the points of the current grain
			m_program->setAttributeArray("vertexPosition", static_cast<GLfloat*>(points[0].data()), 3);

			m_program->setUniformValue("pointSize", m_glPointSize);

			glFunc->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
			glFunc->glDisable(GL_LIGHTING);
			glFunc->glDisable(GL_TEXTURE_2D);

			glFunc->glDrawArrays(GL_POINTS, 0, stack.size());

			// reset the vertex positions to the template sphere
			m_program->setAttributeArray("vertexPosition", static_cast<GLfloat*>(vertices.data()), 3);
		}
	}

	m_program->disableAttributeArray("vertexPosition");
	m_program->disableAttributeArray("vertexNormal");
	m_program->disableAttributeArray("vertexTexCoord");

	return true;
}

void GrainsAsEllipsoids::drawGrains(CC_DRAW_CONTEXT& context)
{
	if (!initProgram(context.qGLContext))
	{
		ccLog::Warning("[GrainsAsEllipsoids::drawGrains] impossible to init shader program");
		return;
	}

	m_program->bind();

	drawEllipsoids(context);

	m_program->release();

	m_app->redrawAll();
}

void GrainsAsEllipsoids::setOnlyOne(int i)
{
	m_onlyOne = i;
	updateBBoxOnlyOne(i);
	redrawDisplay();
}

void GrainsAsEllipsoids::showOnlyOne(bool state)
{
	m_showAll =!state;
	m_ccBBox = m_ccBBoxOnlyOne;
	redrawDisplay();
}

void GrainsAsEllipsoids::showAll(bool state)
{
	m_showAll = state;
	m_ccBBox = m_ccBBoxAll;
	redrawDisplay();
}

void GrainsAsEllipsoids::draw(CC_DRAW_CONTEXT& context)
{
	 if (isVisible() && isEnabled())
	 {
		if (MACRO_Draw3D(context))
			drawGrains(context);
	 }
}

ccBBox GrainsAsEllipsoids::getOwnBB(bool withGLFeatures)
{
	return m_ccBBox;
}

bool GrainsAsEllipsoids::toFile(QFile& out, short dataVersion) const
{
	ccLog::Print("[GrainsAsEllipsoids::toFile]");
	ccSerializationHelper::GenericArrayToFile<Eigen::Array3f, 1, float>(m_center, out);
	ccSerializationHelper::GenericArrayToFile<Eigen::Array3f, 1, float>(m_radii, out);
	ccSerializationHelper::GenericArrayToFile<Eigen::Matrix3f, 1, float>(m_rotationMatrix, out);
	return true;
}

bool GrainsAsEllipsoids::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	ccLog::Print("[GrainsAsEllipsoids::fromFile");
	ccSerializationHelper::GenericArrayFromFile<Eigen::Array3f, 1, float>(m_center, in, dataVersion);
	ccSerializationHelper::GenericArrayFromFile<Eigen::Array3f, 1, float>(m_radii, in, dataVersion);
	ccSerializationHelper::GenericArrayFromFile<Eigen::Matrix3f, 1, float>(m_rotationMatrix, in, dataVersion);

	m_ccBBoxAll.setValidity(false);
	m_ccBBoxAll.clear();

	// recompute bounding box
	for (int idx = 0; idx < m_center.size(); idx++)
	{
			float maxRadius = m_radii[idx].maxCoeff();
			CCVector3 center(m_center[idx](0), m_center[idx](1), m_center[idx](2));
			m_ccBBoxAll.add(CCVector3(center.x + maxRadius, center.y + maxRadius, center.z + maxRadius));
			m_ccBBoxAll.add(CCVector3(center.x - maxRadius, center.y - maxRadius, center.z - maxRadius));
	}

	m_ccBBoxAll.setValidity(true);

	return true;
}
