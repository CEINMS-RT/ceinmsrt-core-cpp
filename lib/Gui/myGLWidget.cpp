#include "myGLWidget.h"

#ifdef USE_OPENSIM

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif
#ifdef WIN32
#include <windows.h>
#endif
#define timer   timer_class
#include <boost/timer.hpp>
#undef timer
#include <boost/timer/timer.hpp>

using namespace gl;
GLWidget::GLWidget(const std::string modelFileName, QWidget *parent, int nbOfModel)
	: QGLWidget(QGLFormat(QGL::SampleBuffers), parent), _nbOfModel(nbOfModel), _modelFileName(modelFileName)
{
	_firtsPass = true;
	srand(time(NULL));
	_bgColor = SimTK::fVec4(0, 0, 0, 1);
	_errorStr = std::string(" ");
}

GLWidget::~GLWidget()
{
}

//QSize GLWidget::minimumSizeHint() const
//{
//    return QSize(50, 50);
//}
//
//QSize GLWidget::sizeHint() const
//{
//    return QSize(400, 400);
//}
//
static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void GLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != xRot) {
		xRot = angle;
		emit xRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != yRot) {
		yRot = angle;
		emit yRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != zRot) {
		zRot = angle;
		emit zRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setYTranslation(double position)
{
	if (position != yTrans) {
		yTrans = position;
		emit yTranslationChanged(position);
		updateGL();
	}
}

void GLWidget::setXTranslation(double position)
{
	if (position != xTrans) {
		xTrans = position;
		emit yTranslationChanged(position);
		updateGL();
	}
}

void GLWidget::initializeGL()
{
#ifdef WIN32
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed */
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}

#endif
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient);

	// Initialize miscellaneous OpenGL state.

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_MULTISAMPLE);

	fovyPerspective = 30;
	yTrans = 0;
	xTrans = 0;
	xRot = 0;
	yRot = 0;
	zRot = 0;

	if (_nbOfModel == 2)
	{
		createModel1();
		createModel2();
	}
	else
	{
		createModel1();
	}

	light_diffuse[0] = 0.8f;
	light_diffuse[1] = 0.8f;
	light_diffuse[2] = 0.8f;
	light_diffuse[3] = 1.0f;
	light_position[0] = 1.0f;
	light_position[1] = 1.0f;
	light_position[2] = 1.0f;
	light_position[3] = 0.0f;
	light_ambient[0] = 0.2f;
	light_ambient[1] = 0.2f;
	light_ambient[2] = 0.2f;
	light_ambient[3] = 1.0f;

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient);

	glClearColor(_bgColor[0], _bgColor[1], _bgColor[2], _bgColor[3]);

	computeSceneBounds(_radius, _center);

	//	connect(&timer, SIGNAL(timeout()), this, SLOT(testModel()));
	//	timer.start(100);
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glClearColor(_bgColor[0], _bgColor[1], _bgColor[2], _bgColor[3]);

	//computeSceneBounds(_radius, _center);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//	glFrustum(-1, 1, -1, 1, 3, 9); // near and far match your triangle Z distance
	gluPerspective(fovyPerspective, float(width()) / float(height()), 2 - _radius, 2 + 4 * _radius);
	//	gluLookAt(xTrans,yTrans,1,_center[0],_center[1],_center[2],0,1,0);
	gluLookAt(xTrans, yTrans, 1, 0, 0, 0, 0, 1, 0);

	glTranslatef(-_center[0], -_center[1], -_center[2] - 2.0f);

	glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
	glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
	glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnable(GL_LIGHTING);
	for (std::vector<RenderedMesh>::iterator it = _renderMeshVectModel1.begin(); it < _renderMeshVectModel1.end(); it++)
		it->draw(true);
	for (std::vector<RenderedMesh>::iterator it = _renderMeshVectModel2.begin(); it < _renderMeshVectModel2.end(); it++)
		it->draw(true);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	//	glDisable(GL_LIGHTING);
	//	glDisable(GL_DEPTH_TEST);
	//	glColor4f(0,0.5,0, 1);
	////	QFont serifFont("Times", 20, QFont::Bold);
	////	renderText(10, 10, QString(_errorStr.c_str()), serifFont);
	//	qglColor(Qt::green);
	//	renderText(0, 0, "svqsqbsqbs", QFont("Arial", 12, QFont::Bold, false) );
	//	glEnable(GL_DEPTH_TEST);
	//	glEnable(GL_LIGHTING);

	//	swapBuffers();
}

void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, (GLint)width, (GLint)height);

	/* create viewing cone with near and far clipping planes */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1.0, 1.0, -1.0, 1.0, 5.0, 30.0);

	glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(xRot + 8 * dy);
		setYRotation(yRot + 8 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(xRot + 8 * dy);
		setZRotation(zRot + 8 * dx);
	}
	else if (event->buttons() & Qt::MiddleButton) {
		setYTranslation(yTrans - 0.001 * dy);
		setXTranslation(xTrans + 0.001 * dx);
	}
	lastPos = event->pos();
}

void GLWidget::wheelEvent(QWheelEvent* event)
{
	if (event->delta() > 0)
	{
		fovyPerspective = fovyPerspective - 5;
	}
	else if (event->delta() < 0)
	{
		fovyPerspective = fovyPerspective + 5;
	}
	updateGL();
}

bool GLWidget::findGeometryFile(const std::string& geoFile, bool& geoFileIsAbsolute,
	SimTK::Array_<std::string>& attempts)
{
	attempts.clear();
	std::string geoDirectory, geoFileName, geoExtension;
	SimTK::Pathname::deconstructPathname(geoFile, geoFileIsAbsolute,
		geoDirectory, geoFileName, geoExtension);

	std::string modelPath = _model1->getInputFileName();

	bool foundIt = false;
	if (geoFileIsAbsolute)
	{
		attempts.push_back(geoFile);
		foundIt = SimTK::Pathname::fileExists(attempts.back());
	}
	else
	{

		std::string modelDir;
		std::string geoDir = "/Geometry/";
			//+ SimTK::Pathname::getPathSeparator();
		
		if (modelPath == "Unassigned")
			modelDir = SimTK::Pathname::getCurrentWorkingDirectory();
		else
		{
			//std::string directory, fileName, extension;
			//SimTK::Pathname::deconstructPathname(modelPath, isAbsolutePath,directory, fileName, extension);
			boost::filesystem::path p(modelPath);
			modelDir = p.parent_path().string();
		}

		attempts.push_back(modelDir + "/" + geoFile);
		foundIt = SimTK::Pathname::fileExists(attempts.back());

		if (!foundIt)
		{
			attempts.push_back(modelDir + geoDir + geoFile);
			foundIt = SimTK::Pathname::fileExists(attempts.back());
		}

		if (!foundIt)
		{
			std::string temp = "/../";
			attempts.push_back(modelDir + temp+ geoDir + geoFile);
			foundIt = SimTK::Pathname::fileExists(attempts.back());
		}

		if (!foundIt)
		{
			fs::path resolvedPath = fs::absolute(std::string(modelDir+ "/../../" + geoDir + geoFile));

			attempts.push_back(resolvedPath.string());
			foundIt = SimTK::Pathname::fileExists(attempts.back());
		}
	}
	return foundIt;
}

void GLWidget::computeSceneBounds(float& radius, fVec3& center)
{
	// Record the bounding sphere of every object in the scene.

	vector<fVec3> centers;
	vector<float> radii;
	for (int i = 0; i < _renderMeshVectModel1.size(); i++) {
		fVec3 center;
		float radius;
		_renderMeshVectModel1[i].computeBoundingSphere(radius, center);
		centers.push_back(center);
		radii.push_back(radius);
	}

	// Find the overall bounding sphere of the scene.

	if (centers.size() == 0) {
		radius = 0;
		center = fVec3(0);
	}
	else {
		fVec3 lower = centers[0] - radii[0];
		fVec3 upper = centers[0] + radii[0];
		for (int i = 1; i < (int)centers.size(); i++) {
			for (int j = 0; j < 3; j++) {
				lower[j] = min(lower[j], centers[i][j] - radii[i]);
				upper[j] = max(upper[j], centers[i][j] + radii[i]);
			}
		}
		center = (lower + upper) / 2;
		radius = 0;
		for (int i = 0; i < (int)centers.size(); i++)
			radius = max(radius, (centers[i] - center).norm() + radii[i]);
	}
}

void GLWidget::setPositionModel1(const std::vector<double>& position)
{
	//std::cout << "setPositionModel1" << std::endl;
	const OpenSim::CoordinateSet& coords = _model1->getCoordinateSet();
	//const OpenSim::BodySet& bodies = _model1->getBodySet();
	{
		//		std::cout << std::endl;
		//		std::cout << "coord: ";
		//		boost::timer::auto_cpu_timer auto_t2;
		for (int i = 0; i < coords.getSize(); ++i)
		{
			if (!coords[i].getLocked(*_si1))
			{
				//				std::cout << std::endl;
				//				boost::timer::auto_cpu_timer auto_t3;
				//coords[i].setValue(*_si1, position.at(i));
				_model1->updMatterSubsystem().getMobilizedBody(coords[i].getBodyIndex()).setOneQ(*_si1, coords[i].getMobilizerQIndex(), position.at(i));
				//				std::cout << std::endl;
			}
		}
	}
	//	std::cout << std::endl;

	{
		//		std::cout << "mesh: ";
		//		boost::timer::auto_cpu_timer auto_t2;
		_model1->getMultibodySystem().realize(*_si1, SimTK::Stage::Position);
		int cpt = 0;
		OpenSim::ComponentList<const OpenSim::Body> bodyList = _model1->getComponentList<OpenSim::Body>();
		for (const OpenSim::Body& body : bodyList)
		{
			//const OpenSim::Body& body = bodies[i]; //old
			//const SimTK::MobilizedBodyIndex bx = body.getIndex(); // old
			//const OpenSim::VisibleObject& visible = *body.getDisplayer(); //old
			//const OpenSim::GeometrySet& geomSet = visible.getGeometrySet(); //old
			//const Transform X_BV = visible.getTransform(); //old

			const SimTK::MobilizedBodyIndex bx = body.getMobilizedBodyIndex();
			const Transform X_BV = body.getTransformInGround(*_si1);

			//for (int g = 0; g < geomSet.getSize(); ++g) //old
			OpenSim::ComponentList<const OpenSim::Mesh> meshList = body.getComponentList<OpenSim::Mesh>();
			for (const OpenSim::Mesh& mesh : meshList)
			{
				//const OpenSim::DisplayGeometry& geo = geomSet[g]; //old
				const MobilizedBody& mobod = _model1->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(bx));
				const Transform& X_GB = mobod.getBodyTransform(*_si1);
				SimTK::Transform transform;
				if (_model1->getDocumentFileVersion() == 30000)
				{
					//const OpenSim::DisplayGeometry& geo = geomSet[g];
					//transform = X_GB*X_BV*geo.getTransform(); //old
					//SimTK::Transform transform = X_GB * X_BV * mesh.getFrame().getTransformInGround(*_si1);
				}
				else
				{
					transform = mesh.getFrame().getTransformInGround(*_si1);
				}
				// Option A
				/*if (mesh.get_mesh_file() == "cerv1sm.vtp" || mesh.get_mesh_file() == "cerv2sm.vtp" || mesh.get_mesh_file() == "cerv3sm.vtp" || mesh.get_mesh_file() == "cerv4sm.vtp"
					|| mesh.get_mesh_file() == "cerv5sm.vtp" || mesh.get_mesh_file() == "cerv6sm.vtp" || mesh.get_mesh_file() == "cerv7.vtp" || mesh.get_mesh_file() == "thoracic1_s.vtp"
					|| mesh.get_mesh_file() == "thoracic2_s.vtp")
				{
					transform = X_GB * X_BV * mesh.getFrame().getTransformInGround(*_si1);
				}*/
				//SimTK::Transform transform = X_GB * mesh.getFrame().getTransformInGround(*_si1);
				//				if(_firtsPass)
				//				{
				//					_center[0] += transform.T()[0];
				//					_center[1] += transform.T()[1];
				//					_center[2] += transform.T()[2];
				//					_firtsPass = false;
				//				}
				if (cpt >= _renderMeshVectModel1.size())
					continue;
				_renderMeshVectModel1.at(cpt).setTransform(transform);
				cpt++;
			}
		}
	}
	//	std::cout << std::endl;
}

void GLWidget::setPositionModel2(const std::vector<double>& position)
{
	const OpenSim::CoordinateSet& coords = _model2->getCoordinateSet();
	const OpenSim::BodySet& bodies = _model2->getBodySet();
	for (int i = 0; i < coords.getSize(); ++i)
	{
		if (!coords[i].getLocked(*_si2))
			_model2->updMatterSubsystem().getMobilizedBody(coords[i].getBodyIndex()).setOneQ(*_si2, coords[i].getMobilizerQIndex(), position.at(i));
	}

	_model2->getMultibodySystem().realize(*_si2, SimTK::Stage::Position);
	int cpt = 0;
	OpenSim::ComponentList<const OpenSim::Body> bodyList = _model2->getComponentList<OpenSim::Body>();
	for (const OpenSim::Body& body : bodyList)
	{
		const SimTK::MobilizedBodyIndex bx = body.getMobilizedBodyIndex();
		//const OpenSim::VisibleObject& visible = *body.getDisplayer();
		//const Transform X_BV = visible.getTransform();
		//const Transform X_BV = body.getTransformInGround(*_si2);
		//const OpenSim::GeometrySet& geomSet = visible.getGeometrySet();
		OpenSim::ComponentList<const OpenSim::Mesh> meshList = body.getComponentList<OpenSim::Mesh>();
		for (const OpenSim::Mesh& mesh : meshList)
		{
			//const OpenSim::DisplayGeometry& geo = geomSet[g];
			const MobilizedBody& mobod = _model2->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(bx));
			const Transform& X_GB = mobod.getBodyTransform(*_si2);
			//SimTK::Transform transform = X_GB*X_BV* mesh.getFrame().getTransformInGround(*_si2);
			//SimTK::Transform transform = X_GB * mesh.getFrame().getTransformInGround(*_si2);
			SimTK::Transform transform = X_GB;
			_renderMeshVectModel2.at(cpt).setTransform(transform);
			cpt++;
		}
	}
}

double GLWidget::computeErrorBetweenModel()
{
	double error;
	const OpenSim::CoordinateSet& coords1 = _model1->getCoordinateSet();
	const OpenSim::CoordinateSet& coords2 = _model2->getCoordinateSet();
	int cpt = 0;
	for (int i = 0; i < coords1.getSize(); ++i)
	{
		if (!coords1[i].getLocked(*_si2))
		{
			error += abs(coords1[i].getValue(*_si1) - coords2[i].getValue(*_si2));
			cpt++;
		}
	}
	const OpenSim::BodySet& bodies1 = _model1->getBodySet();
	const OpenSim::BodySet& bodies2 = _model2->getBodySet();

	SimTK::Vec3 pos1 = _model1->getMatterSubsystem().getMobilizedBody(
		MobilizedBodyIndex(bodies1[bodies1.getSize() - 1].getMobilizedBodyIndex())).getBodyOriginLocation(*_si1);
	SimTK::Vec3 pos2 = _model2->getMatterSubsystem().getMobilizedBody(
		MobilizedBodyIndex(bodies2[bodies2.getSize() - 1].getMobilizedBodyIndex())).getBodyOriginLocation(*_si2);

	double errorPosition = sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));

	setColorFromError(error / cpt + errorPosition);

	std::ostringstream strs;
	strs << error / cpt + errorPosition;
	_errorStr = strs.str();

	return error / cpt + errorPosition;
}

void GLWidget::setColorFromError(double error)
{
	if (error > 1)
		error = 1;
	//	_bgColor[0] = 1 - log(error * 100 + 1) / 2;
	//	_bgColor[1] = log(error * 100 + 1) / 2;
	_bgColor[0] = 1 - error;
	//	_bgColor[1] = error;
}

void GLWidget::ComputeRandPositionForModel2()
{
	const OpenSim::CoordinateSet& coords2 = _model2->getCoordinateSet();
	for (int i = 0; i < coords2.getSize(); ++i)
	{
		if (i > 5 && i < 10)
			if (!coords2[i].getLocked(*_si2))
			{
				double value = double(rand() % int(coords2[i].getRangeMax() * 1000) + int(coords2[i].getRangeMin() * 1000)) / 1000;
				_model2->updMatterSubsystem().getMobilizedBody(coords2[i].getBodyIndex()).setOneQ(
					*_si2, coords2[i].getMobilizerQIndex(), value);
				std::cout << coords2[i].getRangeMax() << " < " << value << " < " << coords2[i].getRangeMin() << std::endl;
			}
	}
	const OpenSim::BodySet& bodies = _model2->getBodySet();
	_model2->getMultibodySystem().realize(*_si2, SimTK::Stage::Position);
	int cpt = 0;
	OpenSim::ComponentList<const OpenSim::Body> bodyList = _model2->getComponentList<OpenSim::Body>();
	for (const OpenSim::Body& body : bodyList)
	{
		const SimTK::MobilizedBodyIndex bx = body.getMobilizedBodyIndex();
		//const OpenSim::VisibleObject& visible = *body.getDisplayer();
		//const Transform X_BV = visible.getTransform();
		//const Transform X_BV = body.getTransformInGround(*_si2);
		//const OpenSim::GeometrySet& geomSet = visible.getGeometrySet();
		OpenSim::ComponentList<const OpenSim::Mesh> meshList = body.getComponentList<OpenSim::Mesh>();
		for (const OpenSim::Mesh& mesh : meshList)
		{
			//const OpenSim::DisplayGeometry& geo = geomSet[g];
			const MobilizedBody& mobod = _model2->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(bx));
			const Transform& X_GB = mobod.getBodyTransform(*_si2);
			//SimTK::Transform transform = X_GB*X_BV* mesh.getFrame().getTransformInGround(*_si2);
			//SimTK::Transform transform = X_GB * mesh.getFrame().getTransformInGround(*_si2);
			SimTK::Transform transform = X_GB;
			_renderMeshVectModel2.at(cpt).setTransform(transform);
			cpt++;
		}
	}
}

void GLWidget::testModel()
{
	const OpenSim::CoordinateSet& coords1 = _model1->getCoordinateSet();
	std::vector<double> position1(coords1.getSize());
	for (int i = 0; i < coords1.getSize(); ++i)
	{
		if (i < 7)
			position1[i] = coords1[i].getValue(*_si1);
		else
		{
			if (increment1.at(i))
				position1[i] = coords1[i].getValue(*_si1) + 0.02;
			else
				position1[i] = coords1[i].getValue(*_si1) - 0.02;
		}

		if (position1[i] > coords1[i].getRangeMax())
		{
			position1[i] = coords1[i].getRangeMax();
			increment1.at(i) = false;
		}
		else if (position1[i] < coords1[i].getRangeMin())
		{
			position1[i] = coords1[i].getRangeMin();
			increment1.at(i) = true;
		}
	}

	const OpenSim::CoordinateSet& coords2 = _model2->getCoordinateSet();
	std::vector<double> position2(coords2.getSize());
	for (int i = 0; i < coords2.getSize(); ++i)
	{
		if (i < 7)
			position2[i] = coords2[i].getValue(*_si2);
		else
		{
			if (increment2.at(i))
				position2[i] = coords2[i].getValue(*_si2) - 0.02;
			else
				position2[i] = coords2[i].getValue(*_si2) + 0.02;
		}

		if (position2[i] > coords2[i].getRangeMax())
		{
			position2[i] = coords2[i].getRangeMax();
			increment2.at(i) = true;
		}
		else if (position2[i] < coords2[i].getRangeMin())
		{
			position2[i] = coords2[i].getRangeMin();
			increment2.at(i) = false;
		}
	}
	setPositionModel1(position1);
	setPositionModel2(position2);
	updateGL();
}

void GLWidget::createModel1()
{
	//std::string filename = "cfg/Guillaume/gait2392_simbody_marker_reducedSet.osim";
	{
		std::ifstream infile(_modelFileName);
		if (!infile.good())
			std::cout << "File not found : " << _modelFileName << std::endl << std::flush;
	}
	//std::cout << "myGLWidget.cpp:584 _modelFileName not set as InterThread::getModelFileName() is not set (do it in CEINMS)" << std::endl;
	//OpenSim::Model model(filename);
	//model.initSystem();
//	_model1 = new OpenSim::Model(filename.c_str());


	fs::path fsLibPath = _modelFileName;
	fs::path resolvedPath;

	if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
		resolvedPath = fsLibPath;
	else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
		resolvedPath = fs::absolute(_modelFileName);
	else
		resolvedPath = fs::absolute(fs::current_path().string() + "/../../" + _modelFileName); // added because when using RTOSIM + GUI + saving data the model is not found. The current workspace is where the file is save

	_model1 = new OpenSim::Model(resolvedPath.string());
	_model1->initSystem();
	_si1 = new SimTK::State(_model1->initSystem());

	_model1->getMultibodySystem().realize(*_si1, SimTK::Stage::Position);

	//const OpenSim::BodySet& bodies = _model1->getBodySet();
	OpenSim::ComponentList<const OpenSim::Body> bodyList = _model1->getComponentList<OpenSim::Body>();
	for (const OpenSim::Body& body : bodyList)
	{
		const SimTK::MobilizedBodyIndex bx = body.getMobilizedBodyIndex();
		//const OpenSim::VisibleObject& visible = *body.getDisplayer();
		SimTK::Vec3 scale;
		//visible.getScaleFactors(scale);

		// I think there is no need to get these scaling factors, thus I set them to 1
		scale = body.get_frame_geometry().get_scale_factors();
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;

		const SimTK::Transform X_BV = body.getTransformInGround(*_si1);
		//const OpenSim::GeometrySet& geomSet = visible.getGeometrySet();

		//for (int g = 0; g < geomSet.getSize(); ++g)
		OpenSim::ComponentList<const OpenSim::Mesh> meshList = body.getComponentList<OpenSim::Mesh>();
		for (const OpenSim::Mesh& mesh : meshList)
		{
//#ifdef OPENSIM4
//#else
//#endif
			//const OpenSim::Geometry& geo = body.get_attached_geometry(g);
			// const OpenSim::DisplayGeometry& geo = geomSet[g]; //old

			//const OpenSim::DisplayGeometry& geo = geomSet[g];
			//mesh.get_scale_factors();

			// Alejandro: There is no display preference in OpenSim 4.1. I will set the default to: SimTK:DecorativeGeometry::DrawSurface
			/*
			const OpenSim::DisplayGeometry::DisplayPreference pref =
				geo.getDisplayPreference();
			
			switch (pref)
			{
			case OpenSim::DisplayGeometry::None:
				//				continue; // don't bother with this one (TODO: is that right)
			case OpenSim::DisplayGeometry::WireFrame:
				rep = SimTK::DecorativeGeometry::DrawWireframe;
				break;
			case OpenSim::DisplayGeometry::SolidFill:
			case OpenSim::DisplayGeometry::FlatShaded:
			case OpenSim::DisplayGeometry::GouraudShaded:
				rep = SimTK::DecorativeGeometry::DrawSurface;
				break;
			default:
				assert(!"bad DisplayPreference");
			};
			*/

			SimTK::DecorativeGeometry::Representation rep = SimTK::DecorativeGeometry::DrawSurface;

			// This is a temporary solution. The problem is that from get_attached_geometry I get a Geometry object, which does not have the
			// get_mesh_file function. This is only available for Mesh class (subclass of Geometry class).

			const std::string& file = mesh.get_mesh_file();

			bool isAbsolutePath;
			std::string directory, fileName, extension;
			SimTK::Pathname::deconstructPathname(file, isAbsolutePath,
				directory, fileName, extension);
			const std::string lowerExtension = SimTK::String::toLower(
				extension);
			if (lowerExtension != ".vtp" && lowerExtension != ".obj")
			{
				std::clog << "ModelVisualizer ignoring '" << file
					<< "'; only .vtp and .obj files currently supported.\n";
				continue;
			}

			// File is a .vtp or .obj. See if we can find it.
			SimTK::Array_<std::string> attempts;
			bool foundIt = findGeometryFile(file, isAbsolutePath, attempts);

			if (!foundIt)
			{
				std::clog << "ModelVisualizer couldn't find file '" << file
					<< "'; tried\n";
				for (unsigned i = 0; i < attempts.size(); ++i)
					std::clog << "  " << attempts[i] << "\n";
				if (!isAbsolutePath
					&& !SimTK::Pathname::environmentVariableExists(
					"OPENSIM_HOME"))
					std::clog << "Set environment variable OPENSIM_HOME "
					<< "to search $OPENSIM_HOME/Geometry.\n";
				continue;
			}

			SimTK::PolygonalMesh pmesh;
			try
			{
				if (lowerExtension == ".vtp")
				{
					pmesh.loadVtpFile(attempts.back());
				}
				else
				{
					std::ifstream objFile;
					objFile.open(attempts.back().c_str());
					pmesh.loadObjFile(objFile);
					// objFile closes when destructed
				}
			}
			catch (const std::exception& e)
			{
				std::clog << "ModelVisualizer couldn't read " << attempts.back()
					<< " because:\n" << std::string(e.what()) << "\n";
				continue;
			}

			// SimTK::Vec3 netScale = geo.getScaleFactors().elementwiseMultiply(scale); // old
			//std::cout << mesh.get_scale_factors() << std::endl;
			SimTK::Vec3 bodyScale = mesh.get_scale_factors();
			SimTK::Vec3 netScale = bodyScale.elementwiseMultiply(scale);

			std::vector<float> vertices;
			std::vector<GLushort> faces;
			for (int i = 0; i < pmesh.getNumVertices(); i++) {
				SimTK::Vec3 pos = pmesh.getVertexPosition(i);
				vertices.push_back((float)pos[0]);
				vertices.push_back((float)pos[1]);
				vertices.push_back((float)pos[2]);
			}
			for (int i = 0; i < pmesh.getNumFaces(); i++) {
				int numVert = pmesh.getNumVerticesForFace(i);
				if (numVert < 3)
					continue; // Ignore it.
				if (numVert == 3) {
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 1));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
				}
				else if (numVert == 4) {
					// Split it into two triangles.

					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 1));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 3));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
				}
				else {
					// Add a vertex at the center, then split it into triangles.

					SimTK::Vec3 center(0);
					for (int j = 0; j < numVert; j++) {
						SimTK::Vec3 pos = pmesh.getVertexPosition(pmesh.getFaceVertex(i, j));
						center += pos;
					}
					center /= numVert;
					vertices.push_back((float)center[0]);
					vertices.push_back((float)center[1]);
					vertices.push_back((float)center[2]);
					const unsigned newIndex = (unsigned)(vertices.size() / 3 - 1);
					for (int j = 0; j < numVert - 1; j++) {
						faces.push_back((GLushort)pmesh.getFaceVertex(i, j));
						faces.push_back((GLushort)pmesh.getFaceVertex(i, j + 1));
						faces.push_back((GLushort)newIndex);
					}
				}
			}

			std::vector<SimTK::fVec3> normals(vertices.size(), SimTK::fVec3(0));
			for (int i = 0; i < faces.size() / 3; i++) {
				int v1 = faces[3 * i];
				int v2 = faces[3 * i + 1];
				int v3 = faces[3 * i + 2];
				SimTK::fVec3 vert1(vertices[3 * v1], vertices[3 * v1 + 1], vertices[3 * v1 + 2]);
				SimTK::fVec3 vert2(vertices[3 * v2], vertices[3 * v2 + 1], vertices[3 * v2 + 2]);
				SimTK::fVec3 vert3(vertices[3 * v3], vertices[3 * v3 + 1], vertices[3 * v3 + 2]);
				SimTK::fVec3 norm = (vert2 - vert1) % (vert3 - vert1);
				float length = norm.norm();
				if (length > 0) {
					norm /= length;
					normals[v1] += norm;
					normals[v2] += norm;
					normals[v3] += norm;
				}
			}

			std::vector<float> normalStd(vertices.size() * 3);
			for (int i = 0; i < vertices.size(); i++) {
				normals[i] = normals[i].normalize();
				normalStd[3 * i] = normals[i][0];
				normalStd[3 * i + 1] = normals[i][1];
				normalStd[3 * i + 2] = normals[i][2];
			}
			const MobilizedBody& mobod =
				_model1->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(bx));
			const SimTK::Transform& X_GB = mobod.getBodyTransform(*_si1);

			//SimTK::Transform Xtest = mesh.getFrame().getTransformInGround(*_si1);
			//SimTK::Transform transform = X_GB*X_BV*mesh.getFrame().getTransformInGround(*_si1);
			//SimTK::Transform transform = X_GB * mesh.getFrame().getTransformInGround(*_si1);
			//SimTK::Transform transform = mesh.getFrame().getTransformInGround(*_si1);
			SimTK::Transform transform = mesh.getFrame().getTransformInGround(*_si1);
			SimTK::Vec4 resultColor;
			resultColor.updSubVec<3>(0) = mesh.getColor();
			//resultColor[3] = geo.getOpacity(); // old
			resultColor[3] = 1; // TO DO!!!!!!!!!
			_renderMeshVectModel1.push_back(RenderedMesh(transform, netScale, resultColor, rep, 0, 0));
			_renderMeshVectModel1.back().setMesh(vertices, normalStd, faces);
		}
	}
	const OpenSim::CoordinateSet& coords = _model1->getCoordinateSet();
	increment1.resize(coords.getSize());
}

void GLWidget::createModel2()
{
	_model2 = new OpenSim::Model(_modelFileName.c_str());
	_si2 = new SimTK::State(_model2->initSystem());

	_model2->getMultibodySystem().realize(*_si2, SimTK::Stage::Position);

	OpenSim::ComponentList<const OpenSim::Body> bodyList = _model2->getComponentList<OpenSim::Body>();
	for (const OpenSim::Body& body : bodyList)
	{
		const SimTK::MobilizedBodyIndex bx = body.getMobilizedBodyIndex();
		//const OpenSim::VisibleObject& visible = *body.getDisplayer();
		//visible.getScaleFactors(scale);
		SimTK::Vec3 scale;
		scale = body.get_frame_geometry().get_scale_factors();
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
		//const Transform X_BV = visible.getTransform();
		const Transform X_BV = body.getTransformInGround(*_si2);
		//const OpenSim::GeometrySet& geomSet = visible.getGeometrySet();
		//for (int g = 0; g < geomSet.getSize(); ++g)
		OpenSim::ComponentList<const OpenSim::Mesh> meshList = body.getComponentList<OpenSim::Mesh>();
		for (const OpenSim::Mesh& mesh : meshList)
		{
			//const OpenSim::DisplayGeometry& geo = geomSet[g];
			mesh.get_scale_factors();

			SimTK::DecorativeGeometry::Representation rep = SimTK::DecorativeGeometry::DrawSurface;

			/*const OpenSim::DisplayGeometry& geo = geomSet[g];
			const OpenSim::DisplayGeometry::DisplayPreference pref =
				geo.getDisplayPreference();
			SimTK::DecorativeGeometry::Representation rep;
			switch (pref)
			{
			case OpenSim::DisplayGeometry::None:
				//				continue; // don't bother with this one (TODO: is that right)
			case OpenSim::DisplayGeometry::WireFrame:
				rep = SimTK::DecorativeGeometry::DrawWireframe;
				break;
			case OpenSim::DisplayGeometry::SolidFill:
			case OpenSim::DisplayGeometry::FlatShaded:
			case OpenSim::DisplayGeometry::GouraudShaded:
				rep = SimTK::DecorativeGeometry::DrawSurface;
				break;
			default:
				assert(!"bad DisplayPreference");
			};
			*/

			//const std::string& file = geo.getGeometryFile();
			const std::string& file = mesh.get_mesh_file();

			bool isAbsolutePath;
			std::string directory, fileName, extension;
			SimTK::Pathname::deconstructPathname(file, isAbsolutePath,
				directory, fileName, extension);
			const std::string lowerExtension = SimTK::String::toLower(
				extension);
			if (lowerExtension != ".vtp" && lowerExtension != ".obj")
			{
				std::clog << "ModelVisualizer ignoring '" << file
					<< "'; only .vtp and .obj files currently supported.\n";
				continue;
			}

			// File is a .vtp or .obj. See if we can find it.
			SimTK::Array_<std::string> attempts;
			bool foundIt = findGeometryFile(file, isAbsolutePath, attempts);

			if (!foundIt)
			{
				std::clog << "ModelVisualizer couldn't find file '" << file
					<< "'; tried\n";
				for (unsigned i = 0; i < attempts.size(); ++i)
					std::clog << "  " << attempts[i] << "\n";
				if (!isAbsolutePath
					&& !SimTK::Pathname::environmentVariableExists(
					"OPENSIM_HOME"))
					std::clog << "Set environment variable OPENSIM_HOME "
					<< "to search $OPENSIM_HOME/Geometry.\n";
				continue;
			}

			SimTK::PolygonalMesh pmesh;
			try
			{
				if (lowerExtension == ".vtp")
				{
					pmesh.loadVtpFile(attempts.back());
				}
				else
				{
					std::ifstream objFile;
					objFile.open(attempts.back().c_str());
					pmesh.loadObjFile(objFile);
					// objFile closes when destructed
				}
			}
			catch (const std::exception& e)
			{
				std::clog << "ModelVisualizer couldn't read " << attempts.back()
					<< " because:\n" << std::string(e.what()) << "\n";
				continue;
			}

			//SimTK::Vec3 netScale =geo.getScaleFactors().elementwiseMultiply(scale);
			SimTK::Vec3 bodyScale = mesh.get_scale_factors();
			SimTK::Vec3 netScale = bodyScale.elementwiseMultiply(scale);

			std::vector<float> vertices;
			std::vector<GLushort> faces;
			for (int i = 0; i < pmesh.getNumVertices(); i++) {
				SimTK::Vec3 pos = pmesh.getVertexPosition(i);
				vertices.push_back((float)pos[0]);
				vertices.push_back((float)pos[1]);
				vertices.push_back((float)pos[2]);
			}
			for (int i = 0; i < pmesh.getNumFaces(); i++) {
				int numVert = pmesh.getNumVerticesForFace(i);
				if (numVert < 3)
					continue; // Ignore it.
				if (numVert == 3) {
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 1));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
				}
				else if (numVert == 4) {
					// Split it into two triangles.

					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 1));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 2));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 3));
					faces.push_back((GLushort)pmesh.getFaceVertex(i, 0));
				}
				else {
					// Add a vertex at the center, then split it into triangles.

					SimTK::Vec3 center(0);
					for (int j = 0; j < numVert; j++) {
						SimTK::Vec3 pos = pmesh.getVertexPosition(pmesh.getFaceVertex(i, j));
						center += pos;
					}
					center /= numVert;
					vertices.push_back((float)center[0]);
					vertices.push_back((float)center[1]);
					vertices.push_back((float)center[2]);
					const unsigned newIndex = (unsigned)(vertices.size() / 3 - 1);
					for (int j = 0; j < numVert - 1; j++) {
						faces.push_back((GLushort)pmesh.getFaceVertex(i, j));
						faces.push_back((GLushort)pmesh.getFaceVertex(i, j + 1));
						faces.push_back((GLushort)newIndex);
					}
				}
			}

			std::vector<SimTK::fVec3> normals(vertices.size(), SimTK::fVec3(0));
			for (int i = 0; i < faces.size() / 3; i++) {
				int v1 = faces[3 * i];
				int v2 = faces[3 * i + 1];
				int v3 = faces[3 * i + 2];
				SimTK::fVec3 vert1(vertices[3 * v1], vertices[3 * v1 + 1], vertices[3 * v1 + 2]);
				SimTK::fVec3 vert2(vertices[3 * v2], vertices[3 * v2 + 1], vertices[3 * v2 + 2]);
				SimTK::fVec3 vert3(vertices[3 * v3], vertices[3 * v3 + 1], vertices[3 * v3 + 2]);
				SimTK::fVec3 norm = (vert2 - vert1) % (vert3 - vert1);
				float length = norm.norm();
				if (length > 0) {
					norm /= length;
					normals[v1] += norm;
					normals[v2] += norm;
					normals[v3] += norm;
				}
			}

			std::vector<float> normalStd(vertices.size() * 3);
			for (int i = 0; i < vertices.size(); i++) {
				normals[i] = normals[i].normalize();
				normalStd[3 * i] = normals[i][0];
				normalStd[3 * i + 1] = normals[i][1];
				normalStd[3 * i + 2] = normals[i][2];
			}
			const MobilizedBody& mobod =
				_model2->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(bx));
			const Transform& X_GB = mobod.getBodyTransform(*_si2);
			//SimTK::Transform transform = X_GB*X_BV*mesh.getFrame().getTransformInGround(*_si2);
			//SimTK::Transform transform = X_GB * mesh.getFrame().getTransformInGround(*_si2);
			SimTK::Transform transform = X_GB;
			SimTK::Vec4 resultColor;
			resultColor.updSubVec<3>(0) = SimTK::Vec3(0, 0.7, 0);
			//resultColor[3] = geo.getOpacity();
			resultColor[3] = 1;
			_renderMeshVectModel2.push_back(RenderedMesh(transform, netScale, resultColor, rep, 0, 0));
			_renderMeshVectModel2.back().setMesh(vertices, normalStd, faces);
		}
	}
	const OpenSim::CoordinateSet& coords = _model2->getCoordinateSet();
	increment2.resize(coords.getSize());
}

void GLWidget::setPositionfromMap1(const std::map<std::string, double>& positionMap)
{
	const OpenSim::CoordinateSet& coords1 = _model1->getCoordinateSet();
	std::vector<double> position1(coords1.getSize());
	for (int i = 0; i < coords1.getSize(); ++i)
	{
		try
		{
			position1[i] = positionMap.at(coords1[i].getName());
		}
		catch (const std::out_of_range& oor)
		{
			// 			std::cout << coords1[i].getName() << std::endl;
			position1[i] = coords1[i].getValue(*_si1);
		}
	}
	setPositionModel1(position1);
}

#endif