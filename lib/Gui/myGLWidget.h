#ifndef GLWIDGET_H
#define GLWIDGET_H

#ifdef USE_OPENSIM
#define GL_GLEXT_PROTOTYPES


#include <Simbody.h>
#include <OpenSim/OpenSim.h>

/*/ /!\ Warning do not put 
#include <Simbody.h>
#include <OpenSim/OpenSim.h>

after 
#include <QtOpenGL/QtOpenGL>
#include <QtOpenGL/QGLWidget> or other opensim simtk library

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>

will not compile because of redinition but work fine on Linux (go figure !)

*/
#ifdef WIN32
#include <windows.h>
#include <GL/glew.h>
//#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <QtOpenGL/QtOpenGL>
#include <QtOpenGL/QGLWidget>

#include "Mesh.h"
#include "RenderedMesh.h"


#ifdef UNIX
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>
#endif
#ifdef APPLE
#include <gl.h>
#include <glu.h>
#include <glext.h>
#include <Glut/glut.h>
#endif




#if QT_VERSION > QT_VERSION_CHECK(5, 0, 0)
#undef qInfo
#endif


#include <cstring>
#include <utility>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif


namespace gl
{
class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(const std::string modelFileName, QWidget *parent = 0, int nbOfModel = 1);
    ~GLWidget();

//    QSize minimumSizeHint() const;
//    QSize sizeHint() const;

    void setPositionModel1(const std::vector<double>& position);
    void setPositionModel2(const std::vector<double>& position);
    void setModel(const std::string& filename) {_modelFileName = filename;}
    void setPositionfromMap1(const std::map<std::string, double>& positionMap);
    void ComputeRandPositionForModel2();
    double computeErrorBetweenModel();

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setYTranslation(double position);
    void setXTranslation(double position);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
    void yTranslationChanged(double position);
    void xTranslationChanged(double position);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent* event);

private slots:
    void testModel();

private:

    void computeSceneBounds(float& radius, fVec3& center);

    bool findGeometryFile(const std::string& geoFile, bool& geoFileIsAbsolute,
       		SimTK::Array_<std::string>& attempts);

    void createModel1();
    void createModel2();
    void setColorFromError(double error);

    std::string _modelFileName;

	float _radius;
	fVec3 _center;

    int xRot;
    int yRot;
    int zRot;
    double xTrans;
    double yTrans;
    QPoint lastPos;
    QColor qtGreen;
    QColor qtPurple;

	GLfloat light_diffuse[4];
	GLfloat light_position[4];
	GLfloat light_ambient[4];

	int _nbOfModel;

//    boost::thread* thread;
    std::vector<bool> increment1;

    std::vector<bool> increment2;

    SimTK::fVec4 _bgColor;
    std::string _errorStr;

    float fovyPerspective;
    QTimer timer;
    bool _firtsPass; // first pass for the position of the thorax or pelvis.

//    boost::mutex _mutex;
    OpenSim::Model* _model1;
    SimTK::State* _si1;
    OpenSim::Model* _model2;
    SimTK::State* _si2;
    std::vector<RenderedMesh> _renderMeshVectModel1;

    std::vector<RenderedMesh> _renderMeshVectModel2;
    fTransform X_GC;
};
}
#endif

#endif
