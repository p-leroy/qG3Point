#include "GrainsAsEllipsoids.h"

#include <ccPointCloud.h>

#include <QOpenGLShaderProgram>

#include <iostream>
#include <random>

GrainsAsEllipsoids::GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface *app)
	: m_cloud(cloud)
	, m_app(app)
{
	setShaderPath("C:/dev/CloudCompare/plugins/private/qG3POINT/shaders");
}

void GrainsAsEllipsoids::setShaderPath(const QString& path)
{
	m_shaderPath = path;
}

void GrainsAsEllipsoids::releaseShaders()
{
	m_program.clear();
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
//		QString vertexShaderFile(m_shaderPath + "/DrawGrains.vs");
		QString vertexShaderFile(m_shaderPath + "/ellipsoid.vs");
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
//		QString fragmentShaderFile(m_shaderPath + "/DrawGrains.fs");
		QString fragmentShaderFile(m_shaderPath + "/ellipsoid.fs");
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

void GrainsAsEllipsoids::initSphereVertices()
{
	float radius  = 0.2;

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

bool GrainsAsEllipsoids::drawSphere(CC_DRAW_CONTEXT& context, int colorIndex)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	CCVector3f color;
	if (colorIndex < m_grainColors.size())
	{
		color = m_grainColors[colorIndex];
	}
	else
	{
		ccLog::Error("[GrainsAsEllipsoids::drawSphere] index is larger than the color table");
		return false;
	}

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

	for (int k = 0; k < m_localMaximumIndexes.size(); k++)
	{
		const CCVector3f* center = m_cloud->getPoint(m_localMaximumIndexes[k]);
		m_program->setUniformValue("center", center->x, center->y, center->z);
		CCVector3f color = m_grainColors[k];
		m_program->setUniformValue("materialAmbient", color.x, color.y, color.z, 1);

		glFunc->glPushMatrix(); // save the current matrix
		// translate
		glFunc->glTranslatef(center->x, center->y, center->z);
		// rotate
		glFunc->glRotatef(90, static_cast<float>(dis2(gen)), static_cast<float>(dis2(gen)), static_cast<float>(dis2(gen)));
		// scale
		glFunc->glScalef(static_cast<float>(dis(gen)), static_cast<float>(dis(gen)), static_cast<float>(dis(gen)));
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

	m_program->disableAttributeArray("vertexPosition");
	m_program->disableAttributeArray("vertexNormal");
	m_program->disableAttributeArray("vertexTexCoord");

	return true;
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

void GrainsAsEllipsoids::setUniformValueColor(const ccColor::Rgba &color)
{
	m_program->setUniformValue("color", color.r, color.g, color.b, color.a);
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
	m_program->setUniformValue("normalLength", GLfloat(m_normalLineParameters.length));

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
		drawSphere(context, 0);
	}

	m_program->release();

	m_app->redrawAll();
}

void GrainsAsEllipsoids::setLocalMaximumIndexes(const Eigen::ArrayXi &localMaximumIndexes)
{
	m_localMaximumIndexes = localMaximumIndexes;
}

void GrainsAsEllipsoids::draw(CC_DRAW_CONTEXT& context)
{
	drawGrains(context);
}
