#ifndef GRAINSASELLIPSOIDS_H
#define GRAINSASELLIPSOIDS_H

#include "Eigen/Dense"
#include "ccAdvancedTypes.h"

#include <QObject>
#include <QOpenGLShaderProgram>

#include <ccGLWindowInterface.h>
#include <ccHObject.h>
#include <ccMainAppInterface.h>
#include <ccColorTypes.h>

class ccPointCloud;

class GrainsAsEllipsoids : public ccHObject
{
public:
	GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface* app);

	//! Set the path for shaders
	void setShaderPath(const QString &path);

	//! Release shaders (if any)
	/** Must be called before the OpenGL context is released.
	**/
	void releaseShaders();

	bool initProgram(QOpenGLContext* context);

	// <DRAW_SPHERE>

	void initSphereVertices();

	void initSphereIndexes();

	void buildInterleavedVertices();

	bool drawSphere(CC_DRAW_CONTEXT &context, int colorIndex=0);

	void setGrainColorsTable(const RGBAColorsTableType& colorTable);

	std::vector<float> vertices;
	std::vector<float> normals;
	std::vector<float> texCoords;
	std::vector<float> interleavedVertices;

	std::vector<int> indices;
	std::vector<int> lineIndices;

	int sectorCount{21};
	int stackCount{21};

	int interleavedStride = 32;

	// </DRAW_SPHERE>

	void setUniformValueColor(const ccColor::Rgba &color);

	//! Draw grains as ellipsoids
	void drawGrains(CC_DRAW_CONTEXT &context);

	void setLocalMaximumIndexes(const Eigen::ArrayXi &localMaximumIndexes);

	//! Colors of the normals when they are displayed
	enum NormalLineColors{
		YELLOW,
		RED,
		GREEN,
		BLUE,
		BLACK
	};

	struct NormalLineParameters
	{
		float length = 1.0f;
		ccColor::Rgba color = ccColor::yellow;
		int colorIdx = YELLOW;
	} m_normalLineParameters;

	ccPointCloud* m_cloud;
	ccMainAppInterface* m_app;
	Eigen::ArrayXi m_localMaximumIndexes;
	std::vector<CCVector3f> m_grainColors;

	std::vector<CCVector3f> m_ellipsoidInstance;
	Eigen::ArrayX3f ellipsoidInstance;

	QSharedPointer<QOpenGLShaderProgram> m_program;

	// Default path to the shader files
	QString m_shaderPath;

	// from ccHObject
	void draw(CC_DRAW_CONTEXT& context);
};

#endif // GRAINSASELLIPSOIDS_H
