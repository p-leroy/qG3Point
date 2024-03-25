#include "GrainsAsEllipsoids.h"

#include <ccPointCloud.h>
#include <ccGLMatrix.h>

#include <QOpenGLShaderProgram>

#include <iostream>
#include <random>

std::vector<int> indexes;

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

	indexes.push_back(268);
	indexes.push_back(351);

	lockVisibility(false);

	for (int idx : indexes)
	{
		fitEllipsoidToGrain(idx, m_center[idx], m_radii[idx], m_rotationMatrix[idx]);
		std::cout << "grain " << idx << " stack size " << m_stacks[idx].size() << std::endl;
		std::cout << "center " << std::endl << m_center[idx] << std::endl;
		std::cout << "radii " << std::endl << m_radii[idx] << std::endl;
		std::cout << "rotation matrix " << std::endl << m_rotationMatrix[idx] << std::endl;
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
		abort();
	}

	radii = (-s(3, 3) / eigensolver.eigenvalues().array().real()).sqrt().cast<float>();
	rotationMatrix = eigensolver.eigenvectors().transpose().real().cast<float>();

	return true;
}

Eigen::ArrayXd GrainsAsEllipsoids::directFit(const Eigen::ArrayX3d& xyz)
{
	std::cout << "[GrainsAsEllipsoids::directFit]" << std::endl;

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
		abort();
	}

	Eigen::ArrayXd eigenValues(10);
	eigenValues = eigensolver.eigenvalues().real();
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

	Eigen::ArrayXd p(10);

	p << v(0), v(1), v(2)
		, 2 * v(5), 2 * v(4), 2* v(3)
		, 2 * v(6), 2 * v(7), 2 * v(8)
		, v(9);

	return p;
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

		p = directFit(scale * (grainPoints.cast<double>().rowwise() - means)); // Ellipsoid fit

		implicitToExplicit(p, center, radii, rotationMatrix); // Get the explicit parameters

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
	QVector4D materialDiffuse(0.7f, 0.7f, 0.7f, 1);
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

	//generate random colors
	std::mt19937 gen(42);  // to seed mersenne twister.
	std::uniform_real_distribution<> dis(0.5, 1.0);
	std::uniform_real_distribution<> dis2(-1., 1.0);

	for (int idx : indexes)
	{
		Eigen::Matrix3f rotation(m_rotationMatrix[idx].transpose());
		QMatrix4x4 matrixFromFit(rotation(0, 0), rotation(0, 1), rotation(0, 2), m_center[idx](0),
								 rotation(1, 0), rotation(1, 1), rotation(1, 2), m_center[idx](1),
								 rotation(2, 0), rotation(2, 1), rotation(2, 2), m_center[idx](2),
								 0, 0, 0, 1);

		color = m_grainColors[idx];
		glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glFunc->glEnable( GL_BLEND );
		m_program->setUniformValue("materialAmbient", color.x, color.y, color.z, 0.5);

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
		m_program->setUniformValue("drawLines", 0);
		glFunc->glEnable(GL_POLYGON_OFFSET_FILL);
		glFunc->glPolygonOffset(1.0, 1.0f); // move polygon backward
		glFunc->glDrawElements(GL_TRIANGLES, (unsigned int)indices.size(), GL_UNSIGNED_INT, indices.data());
		glFunc->glDisable(GL_POLYGON_OFFSET_FILL);

		// draw lines
		m_program->setUniformValue("drawLines", 1);
		glFunc->glDisable(GL_LIGHTING);
		glFunc->glDisable(GL_TEXTURE_2D);
		glFunc->glDrawElements(GL_LINES, (unsigned int)lineIndices.size(), GL_UNSIGNED_INT, lineIndices.data());

		glFunc->glPopMatrix();
	}

	if (false)
	{ // Matlab 22.36920946,  17.15421478, -11.2193826
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
		glFunc->glDrawElements(GL_TRIANGLES, (unsigned int)indices.size(), GL_UNSIGNED_INT, indices.data());
		glFunc->glDisable(GL_POLYGON_OFFSET_FILL);

		// draw lines
		m_program->setUniformValue("drawLines", 1);
		glFunc->glDisable(GL_LIGHTING);
		glFunc->glDisable(GL_TEXTURE_2D);
		glFunc->glDrawElements(GL_LINES, (unsigned int)lineIndices.size(), GL_UNSIGNED_INT, lineIndices.data());

		glFunc->glPopMatrix();
	}

//	for (int k = 1; k < m_localMaximumIndexes.size(); k++)
//	{
//		if (k==indexOfGrainToFit)
//			continue;

//		const CCVector3f* center = m_cloud->getPoint(m_localMaximumIndexes[k]);
//		CCVector3f color = m_grainColors[k];
//		m_program->setUniformValue("materialAmbient", color.x, color.y, color.z, 1);

//		// prepare translation, rotation and scaling
//		glFunc->glPushMatrix(); // save the current matrix
//		// translate
//		glFunc->glTranslatef(center->x, center->y, center->z);
//		// rotate
//		glFunc->glRotatef(90, static_cast<float>(dis2(gen)), static_cast<float>(dis2(gen)), static_cast<float>(dis2(gen)));
//		// scale
//		glFunc->glScalef(static_cast<float>(dis(gen)), static_cast<float>(dis(gen)), static_cast<float>(dis(gen)));
//		// get matrices
//		glFunc->glGetFloatv(GL_PROJECTION_MATRIX, projection.data());
//		glFunc->glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());
//		m_program->setUniformValue("modelViewProjectionMatrix", projection * modelView);

//		// draw triangles
//		m_program->setUniformValue("drawLines", 0);
//		glFunc->glEnable(GL_POLYGON_OFFSET_FILL);
//		glFunc->glPolygonOffset(1.0, 1.0f); // move polygon backward
//		glFunc->glDrawElements(GL_TRIANGLES, (unsigned int)indices.size(), GL_UNSIGNED_INT, indices.data());
//		glFunc->glDisable(GL_POLYGON_OFFSET_FILL);

//		// draw lines
//		m_program->setUniformValue("drawLines", 1);
//		glFunc->glDisable(GL_LIGHTING);
//		glFunc->glDisable(GL_TEXTURE_2D);
//		glFunc->glDrawElements(GL_LINES, (unsigned int)lineIndices.size(), GL_UNSIGNED_INT, lineIndices.data());

//		glFunc->glPopMatrix();
//	}

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

	// get the coordinates of the local maxima
	std::vector<CCVector3> semiAxisLengths(m_localMaximumIndexes.size(), CCVector3(1, 1, 1));
	std::vector<CCVector3f> centers(m_localMaximumIndexes.size());

	// initialize the vector containing the centers of the grains
	for (int index = 0; index < m_localMaximumIndexes.size(); index++)
	{
		centers[index] = *m_cloud->getPoint(m_localMaximumIndexes[index]);
	}

	if (false)
	{
		m_program->setAttributeArray("vertexIn", static_cast<GLfloat*>(vertices.data()), 3);
		m_program->setAttributeArray("semiAxisLengths", static_cast<GLfloat*>(semiAxisLengths.front().u), 3);

		m_program->enableAttributeArray("vertexIn"); // enable the vertex locations array
		m_program->enableAttributeArray("semiAxisLengths"); // enable the semi-axis lenghts array

		m_program->setUniformValue("center", centers[0].x, centers[0].y, centers[0].z);

		glFunc->glEnable(GL_PROGRAM_POINT_SIZE);
		setUniformValueColor(ccColor::yellow);
		glFunc->glDrawElements(GL_TRIANGLES, (unsigned int)indices.size(), GL_UNSIGNED_INT, indices.data());

		m_program->disableAttributeArray("vertexIn");
		m_program->disableAttributeArray("semiAxesLengths");
	}
	else
	{
		QMatrix4x4 matrixNormal = modelView;
		matrixNormal.setColumn(3, QVector4D(0,0,0,1));
		m_program->setUniformValue("modelViewMatrix", modelView);
		m_program->setUniformValue("normalMatrix", matrixNormal);
		drawEllipsoids(context);
	}

	m_program->release();

	m_app->redrawAll();
}

void GrainsAsEllipsoids::draw(CC_DRAW_CONTEXT& context)
{
	drawGrains(context);
}
