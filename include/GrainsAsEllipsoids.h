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
	typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> Xb;

	GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface* app, const std::vector<std::vector<int> >& stacks);

	//! Set the path for shaders
	void setShaderPath(const QString &path);

	void setLocalMaximumIndexes(const Eigen::ArrayXi &localMaximumIndexes);

	void setGrainColorsTable(const RGBAColorsTableType& colorTable);

	// INIT SPHERE

	void initSphereVertices();

	void initSphereIndexes();

	void buildInterleavedVertices();

	std::vector<float> vertices;
	std::vector<float> normals;
	std::vector<float> texCoords;
	std::vector<float> interleavedVertices;

	std::vector<int> indices;
	std::vector<int> lineIndices;

	int sectorCount{21};
	int stackCount{21};

	int interleavedStride = 32;

	// ELLIPSOID FITTING

	enum Method{
		DIRECT = 0};

	bool explicitToImplicit();

	bool implicitToExplicit();

	Eigen::ArrayXf directFit(const Eigen::ArrayX3f &xyz);

	bool fitEllipsoidToGrain(int grainIndex, const Method& method=DIRECT);

	// DRAW

	//! Release shaders (if any)
	/** Must be called before the OpenGL context is released.
	**/
	void releaseShaders();

	void setUniformValueColor(const ccColor::Rgba &color);

	bool drawSphere(CC_DRAW_CONTEXT &context, int colorIndex=0);

	bool initProgram(QOpenGLContext* context);

	//! Draw grains as ellipsoids
	void drawGrains(CC_DRAW_CONTEXT &context);

	// from ccHObject
	void draw(CC_DRAW_CONTEXT& context);

	ccPointCloud* m_cloud;
	ccMainAppInterface* m_app;
	Eigen::ArrayXi m_localMaximumIndexes;
	std::vector<std::vector<int>> m_stacks;
	std::vector<CCVector3f> m_grainColors;

	std::vector<CCVector3f> m_ellipsoidInstance;
	Eigen::ArrayX3f ellipsoidInstance;

	QSharedPointer<QOpenGLShaderProgram> m_program;

	// Default path to the shader files
	QString m_shaderPath;
};

#endif // GRAINSASELLIPSOIDS_H
