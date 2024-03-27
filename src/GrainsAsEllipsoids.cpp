#include "GrainsAsEllipsoids.h"

#include <ccPointCloud.h>
#include <ccGLMatrix.h>

#include <QOpenGLShaderProgram>

#include <iostream>
#include <random>

// from GeometricTools/GTE
#include <Mathematics/DistPointHyperellipsoid.h>
#include <Mathematics/Vector2.h>
#include <Mathematics/Vector3.h>

GrainsAsEllipsoids::GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface *app, const std::vector<std::vector<int> >& stacks)
	: m_cloud(cloud)
	, m_app(app)
	, m_stacks(stacks)
{
	setShaderPath("C:/dev/CloudCompare/plugins/private/qG3POINT/shaders");

	m_center.resize(m_stacks.size());
	m_radii.resize(m_stacks.size());
	m_rotationMatrix.resize(m_stacks.size());

	// fit all ellipsoids
	std::cout << "[GrainsAsEllipsoids::GrainsAsEllipsoids] fit " << stacks.size() << " ellipsoids" << std::endl;

	lockVisibility(false);

	for (int idx = 0; idx < m_stacks.size(); idx++)
	{
		if (!fitEllipsoidToGrain(idx, m_center[idx], m_radii[idx], m_rotationMatrix[idx]))
		{
			m_fitNotOK.insert(idx);
			ccLog::Warning("[GrainsAsEllipsoids::GrainsAsEllipsoids] fit not possible for grain " + QString::number(idx)
						   + " of size " + QString::number(m_stacks[idx].size()));
		}
	}
}

void GrainsAsEllipsoids::setShaderPath(const QString& path)
{
	m_shaderPath = path;
}

void GrainsAsEllipsoids::setLocalMaximumIndexes(const Eigen::ArrayXi &localMaximumIndexes)
{
	m_localMaximumIndexes = localMaximumIndexes;
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

// INIT ORIGINAL SPHERE

void GrainsAsEllipsoids::initSphereVertices()
{
	float radius  = 1.;

	// clear memory of prev arrays
	std::vector<float>().swap(vertices);
	std::vector<float>().swap(normals);
	std::vector<float>().swap(texCoords);

	float x, y, z, xy;                              // vertex position
	float nx, ny, nz, lengthInv = 1.0f / radius;    // vertex normal
	float s, t;                                     // vertex texCoord

	float sectorStep = 2 * M_PI / sectorCount;
	float stackStep = M_PI / stackCount;
	float sectorAngle, stackAngle;

	for(int i = 0; i <= stackCount; ++i)
	{
		stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
		xy = radius * cosf(stackAngle);             // r * cos(u)
		z = radius * sinf(stackAngle);              // r * sin(u)

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
			nx = x * lengthInv;
			ny = y * lengthInv;
			nz = z * lengthInv;
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
	ccLog::Print("vertices.size() " + QString::number(vertices.size()));
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

void GrainsAsEllipsoids::buildInterleavedVertices()
{
	std::vector<float>().swap(interleavedVertices);

	std::size_t i, j;
	std::size_t count = vertices.size();
	for(i = 0, j = 0; i < count; i += 3, j += 2)
	{
		interleavedVertices.push_back(vertices[i]);
		interleavedVertices.push_back(vertices[i+1]);
		interleavedVertices.push_back(vertices[i+2]);

		interleavedVertices.push_back(normals[i]);
		interleavedVertices.push_back(normals[i+1]);
		interleavedVertices.push_back(normals[i+2]);

		interleavedVertices.push_back(texCoords[j]);
		interleavedVertices.push_back(texCoords[j+1]);
	}
}

// ELLIPSOID FITTING

double GrainsAsEllipsoids::ellipsoidDistance(const Eigen::ArrayXd& p, int idx)
{
	// Compute the mean distance between the points of the grain and the ellipsoid

	// GTE Geometric Tools Engine

	// center
	gte::Vector3<double> center = {m_center[idx](0), m_center[idx](1), m_center[idx](2)};

	// axis
	std::array<gte::Vector3<double>, 3> axis;
	axis[0] = {m_rotationMatrix[idx](0, 0), m_rotationMatrix[idx](1, 0), m_rotationMatrix[idx](2, 0)};
	axis[1] = {m_rotationMatrix[idx](0, 1), m_rotationMatrix[idx](1, 1), m_rotationMatrix[idx](2, 1)};
	axis[2] = {m_rotationMatrix[idx](0, 2), m_rotationMatrix[idx](1, 2), m_rotationMatrix[idx](2, 2)};

	// extent
	gte::Vector3<double> extent = {m_radii[idx](0), m_radii[idx](1), m_radii[idx](2)};

	// create the ellipsoid
	gte::Ellipsoid3<double> ellipsoid(center, axis, extent);

	gte::DCPQuery<double, gte::Vector3<double>, gte::Ellipsoid3<double>> query;

	std::vector<int> stack = m_stacks[idx];
	double sum_a = 0; // distances with respect to the ellipsoid
	double sum_b = 0; // distances with respect to the mean

	//compute gravity center
	size_t count = m_cloud->size();
	CCVector3 mean(0, 0, 0);
	for (int index : m_stacks[idx])
	{
		const CCVector3* P = m_cloud->getPoint(index);
		mean.x += P->x;
		mean.y += P->y;
		mean.z += P->z;
	}
	mean.x = mean.x / count;
	mean.y = mean.y / count;
	mean.z = mean.z / count;

	for (int index : stack)
	{
		const CCVector3 *P = m_cloud->getPoint(index);
		gte::Vector3<double> P_gte = {P->x, P->y, P->z};
		auto result = query(P_gte, ellipsoid);
		sum_a = sum_a + pow(result.distance, 2);

		CCVector3 P_minus_min = *P - CCVector3(mean.x, mean.y, mean.z);
		sum_b = sum_b + P_minus_min.norm2d();
	}

	double r2 = 1 - sum_a / sum_b;

	// in Matlab
	// d = (x-xp).^2 + (y-yp).^2 + (z-zp).^2;
	// r2 = 1 - sum((x-xp).^2 + (y-yp).^2 + (z-zp).^2)./sum((x-mean(x)).^2 + (y-mean(y)).^2 + (z-mean(z)).^2);

	return r2;
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

//	std::cout << "p " << std::endl << p << std::endl;
//	std::cout << "q " << std::endl << q << std::endl;
//	std::cout << "t " << std::endl << t << std::endl;
//	std::cout << "s " << std::endl << s << std::endl;

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

		// create geometry shader
		//		QString geometryShaderFile(m_shaderPath + "/DrawGrains.gs");
		//		if (!m_program->addShaderFromSourceFile(QOpenGLShader::Geometry, geometryShaderFile))
		//		{
		//			error = m_program->log();
		//			ccLog::Error(error);
		//			return false;
		//		}

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

		// initialize the ellipsoid instance
		m_ellipsoidInstance.resize(401);
		float stepTheta = M_PI / 21;
		float stepPhi = 2 * M_PI / 21;
		int count = 0;
		CCVector3f center = *m_cloud->getPoint(m_localMaximumIndexes[0]);
		m_ellipsoidInstance[count++] = CCVector3f(0, 0, 1);
		for (int thetaI = 1; thetaI < 20; thetaI++)
		{
			for (int phiI = 0; phiI < 21; phiI++)
			{
				m_ellipsoidInstance[count++] = CCVector3f(center.x + sin(thetaI * stepTheta) * cos(phiI * stepPhi) * 1,
														  center.y + sin(thetaI * stepTheta) * sin(phiI * stepPhi) * 1,
														  center.z + cos(thetaI * stepTheta) * 1);
			}
		}
		m_ellipsoidInstance[count++] = CCVector3f(0, 0, -1);

		// initialize sphere
		initSphereVertices();
		initSphereIndexes();
		buildInterleavedVertices();
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

		// prepare translation, rotation and scaling
		glFunc->glPushMatrix(); // save the current matrix

		// rotation and translation from the ellipsoid fitting
		glFunc->glMultMatrixf(matrixFromFit.data());
		// scale from the ellipsoid fitting
		glFunc->glScalef(m_radii[idx](0), m_radii[idx](1), m_radii[idx](2));

		// get matrices
		glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
		glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());
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
			glFunc->glDisable(GL_LIGHTING);
			glFunc->glDisable(GL_TEXTURE_2D);
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
	QVector4D lightPosition(0, 0, 1, 0);
	QVector4D lightAmbient(0.3f, 0.3f, 0.3f, 1); // grey
	QVector4D lightDiffuse(0.7f, 0.7f, 0.7f, 1); // light grey
	QVector4D lightSpecular(1.0f, 1.0f, 1.0f, 1); // RGB white
	//	QVector4D materialAmbient(0.5f, 0.5f, 0.5f, 1);
	QVector4D materialDiffuse(0.7f, 0.7f, 0.7f, m_transparency);
	QVector4D materialSpecular(0.4f, 0.4f, 0.4f, 1);
	QVector4D materialAmbient(color.x, color.y, color.z, 1);
	//	QVector4D materialDiffuse(color.r / ccColor::MAX, color.g / ccColor::MAX, color.b / ccColor::MAX, 1);
	//	QVector4D materialSpecular(color.r / ccColor::MAX, color.g / ccColor::MAX, color.b / ccColor::MAX, 1);
	float materialShininess  = 16;

	m_program->setUniformValue("lightPosition", lightPosition);
	m_program->setUniformValue("lightAmbient", lightAmbient);
	m_program->setUniformValue("lightDiffuse", lightDiffuse);
	m_program->setUniformValue("lightSpecular", lightSpecular);
	m_program->setUniformValue("materialDiffuse", materialDiffuse);
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

	if (false) // Matlab fit of grain 351
	{
		Eigen::Matrix3f rotation;
		rotation << -0.1424, -0.8175, -0.5580,
			-0.9102, -0.1133,  0.3983,
			0.3889, -0.5646, 0.7280;
		rotation.transposeInPlace();
		QMatrix4x4 matrixFromFit(rotation(0, 0), rotation(0, 1), rotation(0, 2), 22.36920946,
								 rotation(1, 0), rotation(1, 1), rotation(1, 2), 17.15421478,
								 rotation(2, 0), rotation(2, 1), rotation(2, 2), -11.2193826,
								 0, 0, 0, 1);

		color = CCVector3f(static_cast<float>(255) / ccColor::MAX,
						   static_cast<float>(255) / ccColor::MAX,
						   static_cast<float>(255) / ccColor::MAX);
		m_program->setUniformValue("materialAmbient", color.x, color.y, color.z, 1);

		// prepare translation, rotation and scaling
		glFunc->glPushMatrix(); // save the current matrix

		// rotation and translation from the ellipsoid fitting
		glFunc->glMultMatrixf(matrixFromFit.data());
		// scale from the ellipsoid fitting
		glFunc->glScalef(0.6154, 0.5055, 0.3647);

		// get matrices
		glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
		glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());
		m_program->setUniformValue("modelViewProjectionMatrix", projection * modelView);

		// draw triangles
		m_program->setUniformValue("drawLines", 0);
		glFunc->glEnable(GL_POLYGON_OFFSET_FILL);
		glFunc->glPolygonOffset(1.0, 1.0f); // move polygon backward
		glFunc->glDrawElements(GL_TRIANGLES, (unsigned int) indices.size(), GL_UNSIGNED_INT, indices.data());
		glFunc->glDisable(GL_POLYGON_OFFSET_FILL);

		// draw lines
		m_program->setUniformValue("drawLines", 1);
		glFunc->glDisable(GL_LIGHTING);
		glFunc->glDisable(GL_TEXTURE_2D);
		glFunc->glDrawElements(GL_LINES, (unsigned int) lineIndices.size(), GL_UNSIGNED_INT, lineIndices.data());

		glFunc->glPopMatrix();
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

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	QMatrix4x4 projection;
	QMatrix4x4 modelView;
	glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
	glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());
	QMatrix4x4 projectionModelView = projection * modelView; // QMatrix4x4 expects row major data

	m_program->bind();
	// set uniforms
	m_program->setUniformValue("modelViewProjectionMatrix", projectionModelView);

	QMatrix4x4 matrixNormal = modelView;
	matrixNormal.setColumn(3, QVector4D(0,0,0,1));
	m_program->setUniformValue("modelViewMatrix", modelView);
	m_program->setUniformValue("normalMatrix", matrixNormal);
	drawEllipsoids(context);

	m_program->release();

	m_app->redrawAll();
}

void GrainsAsEllipsoids::draw(CC_DRAW_CONTEXT& context)
{
	drawGrains(context);
}
