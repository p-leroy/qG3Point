#ifndef GRAINSASELLIPSOIDS_H
#define GRAINSASELLIPSOIDS_H

#include "Eigen/Dense"
#include "ccAdvancedTypes.h"

#include <QObject>
#include <QOpenGLShaderProgram>

#include <ccGLWindowInterface.h>
#include <ccHObject.h>
#include <ccCustomObject.h>
#include <ccMainAppInterface.h>
#include <ccColorTypes.h>

#include <set>

class ccPointCloud;

class GrainsAsEllipsoids : public QObject, public ccCustomHObject
{
	Q_OBJECT

public:
	typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> Xb;

	GrainsAsEllipsoids(ccPointCloud *cloud, ccMainAppInterface* app, const std::vector<std::vector<int> >& stacks, const RGBAColorsTableType& colors);

	//! Set the path for shaders
	void setShaderPath(const QString &path);

	void setLocalMaximumIndexes(const Eigen::ArrayXi &localMaximumIndexes);

	void setGrainColorsTable(const RGBAColorsTableType& colorTable);

	bool exportResultsAsCloud();

	// INIT SPHERE

	void initSphereVertices();

	void initSphereIndexes();

	std::vector<float> vertices;
	std::vector<float> normals;
	std::vector<float> texCoords;

	std::vector<int> indices;
	std::vector<int> lineIndices;

	int sectorCount{21};
	int stackCount{21};

	// ELLIPSOID FITTING

	enum Method{
		DIRECT = 0};

	double ellipsoidDistance(const Eigen::ArrayXd& p, int idx);

	void updateBBoxOnlyOne(int index);

	bool explicitToImplicit(const Eigen::Array3f& center, const Eigen::Array3f& radii, const Eigen::Matrix3f &rotationMatrix, Eigen::ArrayXd& parameters);

	bool implicitToExplicit(const Eigen::ArrayXd& parameters, Eigen::Array3f& center, Eigen::Array3f& radii, Eigen::Matrix3f& rotationMatrix);

	bool directFit(const Eigen::ArrayX3d& xyz, Eigen::ArrayXd &parameters);

	bool fitEllipsoidToGrain(const int grainIndex, Eigen::Array3f& center, Eigen::Array3f& radii, Eigen::Matrix3f& rotationMatrix, const Method& method=DIRECT);

	// DRAW

	//! Release shaders (if any)
	/** Must be called before the OpenGL context is released.
	**/
	void releaseShaders();

	void setUniformValueColor(const ccColor::Rgba &color);

	void drawEllipsoid(CC_DRAW_CONTEXT &context, int idx);

	bool drawEllipsoids(CC_DRAW_CONTEXT &context);

	bool initProgram(QOpenGLContext* context);

	//! Draw grains as ellipsoids
	void drawGrains(CC_DRAW_CONTEXT &context);

	void setOnlyOne(int i);

	void showOnlyOne(bool state);

	void showAll(bool state);

	void setTransparency(double transparency){m_transparency = transparency; redrawDisplay();}
	void drawSurfaces(bool state){m_drawSurfaces = state; redrawDisplay();}
	void drawLines(bool state){m_drawLines = state; redrawDisplay();}
	void drawPoints(bool state){m_drawPoints = state; redrawDisplay();}
	void setGLPointSize(int size){m_glPointSize = size; redrawDisplay();}

	//Inherited from ccHObject

	void draw(CC_DRAW_CONTEXT& context) override;

	ccBBox getOwnBB(bool withGLFeatures = false) override;

	ccPointCloud* m_cloud;
	ccMainAppInterface* m_app;
	Eigen::ArrayXi m_localMaximumIndexes;
	std::vector<std::vector<int>> m_stacks;
	std::vector<CCVector3f> m_grainColors;
	ccBBox m_ccBBoxOnlyOne;
	ccBBox m_ccBBoxAll;
	ccBBox m_ccBBox;

	std::vector<CCVector3f> m_ellipsoidInstance;
	Eigen::ArrayX3f ellipsoidInstance;

	QSharedPointer<QOpenGLShaderProgram> m_program;

	// Default path to the shader files
	QString m_shaderPath;

	std::vector<Eigen::Array3f> m_center;
	std::vector<Eigen::Array3f> m_radii;
	std::vector<Eigen::Matrix3f> m_rotationMatrix;
	std::set<int> m_fitNotOK;
	GLfloat m_transparency = 1.0;
	bool m_drawSurfaces = true;
	bool m_drawLines = true;
	bool m_drawPoints = false;

	int m_onlyOne;
	bool m_showAll{true};
	int m_glPointSize = 3;
};

#endif // GRAINSASELLIPSOIDS_H
