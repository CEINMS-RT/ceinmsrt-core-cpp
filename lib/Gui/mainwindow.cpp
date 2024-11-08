/*
 * Gui.cpp
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#include "mainwindow.h"

MainWindow::MainWindow(QWidget* parent) :
QMainWindow(parent), _ui(new Ui::MainWindow), firstPass_(true), _timeInitTorque(0),
_timeInitEmg(0), _firstPassTimeTorque(true), _firstPassTimeEmg(true), _maxForce(0),
_minForce(0), _nbMTU(0), _nbNMS(0), AddEMG_(false), add3DIK_(false),
addMuscleForceBar_(false), addTorqueID_(false), addTorqueCEINMS_(false),
addTorqueMotor_(false), addTimingPanel_(false), addCalibrationPanel_(false), addDemo_(false),
emgTotalPeriod_(0), nbofEMG_(1), lmtTotalPeriod_(0), nbofLMT_(1), torqueTotalPeriod_(0), nbofTorque_(1),
posTotalPeriod_(0), nbofPos_(1) // start at 1 for removing the /0
{
#ifdef UNIX
	add3DIK_ = false;
#endif

	QRect rec = QApplication::desktop()->screenGeometry();

	setWindowTitle(tr("CEINMS-RT GUI"));

	deskHeight_ = rec.height() - 100;

	resize(rec.width(), rec.height());

	_splitter = new QSplitter(this);

	_verbose = InterThread::getSyncVerbose();
}

MainWindow::~MainWindow()
{
	delete _ui;
	delete fileMenu_;
	delete helpMenu_;
	delete windowsMenu_;
	delete plotMenu_;
	delete aboutAct_;
	delete openAct_;
	delete exitAct_;
	delete plotPanel_;
	delete treePlotPanel_;
	if (add3DIK_)
		delete openGLPanel_;
	delete timingPanel_;
	delete signalMapperPlot_;
	delete _label;

	for (std::vector<QAction*>::iterator it(plotAct_.begin()); it != plotAct_.end(); it++)
		delete *it;

	for (std::vector<QCPPlotTitle*>::iterator it(title_.begin()); it != title_.end(); it++)
		delete *it;

#ifdef VERBOSE

	if (_verbose > 1)
		std::cout << "\033[1;31mGUI " << this << "\033[0m" << std::endl;

#endif
}

void MainWindow::addEMG()
{
	_musclesNames = InterThread::getMusclesNames();
	AddEMG_ = true;
}

void MainWindow::addTorqueCEINMS()
{
	_dofNames = InterThread::getDofNames();
	addTorqueCEINMS_ = true;
}

void MainWindow::addTorqueID()
{
	_dofNames = InterThread::getDofNames();
	addTorqueID_ = true;
}

void MainWindow::addTorqueMotor()
{
	_dofNames = InterThread::getDofNames();
	addTorqueMotor_ = true;
}

void MainWindow::addTimingPanel()
{
	addTimingPanel_ = true;
}

void MainWindow::add3DIK()
{
	modelName_ = InterThread::getModelFileName();

	add3DIK_ = true;
}


void MainWindow::addDemo()
{
	optimalFiberLength_ = InterThread::getOptimaFiberLength();
	isometricsMaxForce_ = InterThread::getIsometricsMaxForce();
	addDemo_ = true;
}

void MainWindow::addCalibrationPanel()
{
	addCalibrationPanel_ = true;
}

void MainWindow::addMuscleForceBar()
{
	_musclesNames = InterThread::getMusclesNames();
	MapStrVectStr channelNameOnMuscleTemp;
	MapStrVectStr musclesNamesOnChannel = InterThread::getMusclesNamesOnChannel();

	for (MapStrVectStr::const_iterator it1 = musclesNamesOnChannel.begin(); it1 != musclesNamesOnChannel.end(); it1++)
		for (vector<string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++)
		{
			channelNameOnMuscleTemp[*it2].push_back(it1->first);
		}

	std::vector<std::string> channelName;

	for (MapStrVectStr::const_iterator it1 = channelNameOnMuscleTemp.begin(); it1 != channelNameOnMuscleTemp.end(); it1++)
		channelName.push_back(it1->first);

	_muscleNameOnChannel = std::set<std::string>(channelName.begin(), channelName.end());

	addMuscleForceBar_ = true;
}

void MainWindow::start()
{
	QSize totalSizeOfWindows = size();
	QSize sizeOfThePanel = totalSizeOfWindows;

	sizeOfThePanel.setWidth(totalSizeOfWindows.width() * 0.1);
	constructTreePlotPanel(sizeOfThePanel);

	sizeOfThePanel.setWidth(totalSizeOfWindows.width() * 0.5);
	constructPlotPanel(sizeOfThePanel);

	if (add3DIK_)
	{
		sizeOfThePanel.setWidth(totalSizeOfWindows.width() * 0.28);
		constructOpenGLPanel(sizeOfThePanel, modelName_);
	}

	if (addTimingPanel_)
	{
		sizeOfThePanel.setWidth(totalSizeOfWindows.width() * 0.12);
		constructTimingPanel(sizeOfThePanel);
	}

	if (addCalibrationPanel_)
	{
		sizeOfThePanel.setWidth(totalSizeOfWindows.width() * 0.12);
		constructCalibrationPanel(sizeOfThePanel);
	}

	setCentralWidget(_splitter);

	createActions();
	createMenus();

	connect(&_dataTimer, SIGNAL(timeout()), this, SLOT(thread()));
	timePrevious_ = rtb::getTime();
	timeInit_ = rtb::getTime();
	_dataTimer.start(1); //30 Hz
}

void MainWindow::constructTreePlotPanel(const QSize& sizeOfThePanel)
{
	_scrollAreaCheck = new QScrollArea(_splitter);
	_splitter->addWidget(_scrollAreaCheck);
	_scrollAreaCheck->setWidgetResizable(true);
	_treeViewRight = new QTreeWidget(_scrollAreaCheck);
	_treeViewRight->setHeaderLabels(
		QStringList() << "Name" << "Save on PDF");
	_treeViewRight->header()->close();
	_treeViewRight->header()->resizeSection(0, 180);
	_treeViewRight->header()->resizeSection(1, 100);
	_treeViewRight->header()->setStretchLastSection(false);
	_scrollAreaCheck->setWidget(_treeViewRight);
	_scrollAreaCheck->resize(sizeOfThePanel);

	if (addMuscleForceBar_)
	{
		_VectTreeItem.push_back(new QTreeWidgetItem(_treeViewRight));
		_VectTreeItem.back()->setText(0, "Muscle Force");
		_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Checked);
		_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
	}

	unsigned int cpt = 1;

	if (AddEMG_)
	{
		_VectTreeItem.push_back(new QTreeWidgetItem(_treeViewRight));
		_VectTreeItem.back()->setText(0, "EMG");

		for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
			it != _musclesNames.end(); it++)
		{
			_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
			_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
			_VectTreeItem.back()->setText(0, QString((*it).c_str()));
			_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
			_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
		}

		cpt += _musclesNames.size() + 1;
	}

	if (addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_)
	{
		_VectTreeItem.push_back(new QTreeWidgetItem(_treeViewRight));
		_VectTreeItem.back()->setText(0, "Torque");

		for (std::vector<std::string>::const_iterator it = _dofNames.begin();
			it != _dofNames.end(); it++)
		{
			_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
			_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
			_VectTreeItem.back()->setText(0, QString((*it).c_str()));
			_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
			_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
		}
		cpt += _dofNames.size() + 1;
	}

	if (addDemo_)
	{
		_VectTreeItem.push_back(new QTreeWidgetItem(_treeViewRight));
		_VectTreeItem.back()->setText(0, "Demo");
		_VectButtonItem.push_back(new QPushButton("Reset", _treeViewRight));
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());

		connect(_VectButtonItem.back(), SIGNAL(pressed()), this, SLOT(resetPressed()));

		for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
			it != _musclesNames.end(); it++)
		{
			_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
			_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
			_VectTreeItem.back()->setText(0, QString((*it).c_str()));
			_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
			_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
		}

		// Plantar
		_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
		_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
		_VectTreeItem.back()->setText(0, QString("Plantar"));
		_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());

		//Dorsi
		_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
		_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
		_VectTreeItem.back()->setText(0, QString("Dorsi"));
		_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());

		//Total
		_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
		_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight));
		_VectTreeItem.back()->setText(0, QString("Total"));
		_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());

		for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
			it != _musclesNames.end(); it++)
		{
			_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
			_VectButtonItem.push_back(new QPushButton("Save on PDF", _treeViewRight)); 
			std::stringstream emgText;
			emgText << "WorkLoop " + *it;
			_VectTreeItem.back()->setText(0, QString(emgText.str().c_str()));
			_VectTreeItem.back()->setData(0, Qt::CheckStateRole, Qt::Unchecked);
			_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
		}

		_VectTreeItem.push_back(new QTreeWidgetItem(_VectTreeItem.at(cpt)));
		_VectButtonItem.push_back(new QPushButton("Save", _treeViewRight));

		connect(_VectButtonItem.back(), SIGNAL(pressed()), this, SLOT(savePressed()));

		lineEditDemo_ = new QLineEdit(_treeViewRight);
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 1, _VectButtonItem.back());
		_treeViewRight->setItemWidget(_VectTreeItem.back(), 0, lineEditDemo_);

		cpt += 2 * _musclesNames.size() + 1 + 1 + 3;
	}

	treePlotPanelBool_ = true;
}

void MainWindow::constructTimingPanel(const QSize& sizeOfThePanel)
{
	_leftVLayoutWidget = new QWidget(_splitter);
	_leftVLayout = new QVBoxLayout(_leftVLayoutWidget);
	QString text =
		"NMS Computation Time:\n10 ms\n\nMTU Spline Computation time:\n20 ms\n\nTotal Delay:\n30ms\n\nTotal MTU frames:\n40\n\nTotal NMS frames:\n50 \n\nEMG Framerate:\n60Hz\n\nPosition Framerate:\n70Hz\n\nMTU Framerate: 80Hz\n\nNMS Framerate:\n90Hz\n";
	_label = new QLabel(text, _leftVLayoutWidget);
	_leftVLayout->addWidget(_label);
	_leftVLayout->addStretch(1);
	_leftVLayoutWidget->resize(sizeOfThePanel);
	_splitter->addWidget(_leftVLayoutWidget);
	_leftVLayoutWidget->hide();
	timingPanelBool_ = false;
}

void MainWindow::constructOpenGLPanel(const QSize& sizeOfThePanel, const std::string& modelName)
{
#ifdef USE_OPENSIM
	if (add3DIK_)
	{
		_openglWin = new GLWidget(modelName, _splitter);
		_openglWin->resize(sizeOfThePanel);
		_splitter->addWidget(_openglWin);
		_openglWin->updateGL();
		openGLPanelBool_ = true;
	}
#endif
}

void MainWindow::constructPlotPanel(const QSize& sizeOfThePanel)
{
	_scrollArea = new QScrollArea(_splitter);
	_splitter->addWidget(_scrollArea);
	_scrollArea->setWidgetResizable(true);
	_viewport = new QWidget(_scrollArea);
	_layout = new QVBoxLayout(_viewport);
	_nbOfPlot = (_musclesNames.size() + _dofNames.size() + 1);
	if (addDemo_)
		_nbOfPlot += 2 * _musclesNames.size() + 3;

	QPen redPen;
	redPen.setColor(QColor(200, 0, 0, 100));
	redPen.setWidthF(3);//3
	WLColor_.push_back(redPen);
	redPen.setColor(QColor(200, 0, 0, 150));
	barColor_.push_back(redPen);

	QPen greenPen;
	greenPen.setColor(QColor(0, 200, 0, 100));
	greenPen.setWidthF(3);
	WLColor_.push_back(greenPen);
	greenPen.setColor(QColor(0, 200, 0, 150));
	barColor_.push_back(greenPen);

	QPen cyanPen;
	cyanPen.setColor(QColor(0, 120, 200, 100));
	cyanPen.setWidthF(3);//3
	WLColor_.push_back(cyanPen);
	cyanPen.setColor(QColor(0, 120, 200, 150));
	barColor_.push_back(cyanPen);

	QPen magentaPen;
	magentaPen.setColor(QColor(150, 0, 200, 100));
	magentaPen.setWidthF(3);//3
	WLColor_.push_back(magentaPen);
	magentaPen.setColor(QColor(150, 0, 200, 150));
	barColor_.push_back(magentaPen);

	if (_nbOfPlot < 3)
	{
		_viewport->setMaximumHeight(deskHeight_ );
		_viewport->setMinimumHeight(deskHeight_ );
	}
	else
	{
		_viewport->setMaximumHeight(_nbOfPlot * deskHeight_ / 3);
		_viewport->setMinimumHeight(_nbOfPlot * deskHeight_ / 3);
	}

	_scrollArea->setWidget(_viewport);
	_scrollArea->resize(sizeOfThePanel);

	if (addMuscleForceBar_)
		constructBar();

	if (AddEMG_)
		constructEMGPlot();

	if (addTorqueCEINMS_ || addTorqueID_)
		constructTorquePlot();

	if (addDemo_)
	{
		constructDemoBar();
		constructWorkLoopPlot();
	}

	plotPanelBool_ = true;
}

void MainWindow::constructCalibrationPanel(const QSize& sizeOfThePanel)
{
}

void MainWindow::constructBar()
{
	_VectDock.push_back(new QDockWidget(tr("Muscle Force "), this));
	_VectPlot.push_back(new QCustomPlot(_VectDock.back()));
	//_VectDock.back()->setFeatures(QDockWidget::NoDockWidgetFeatures);
	_layout->addWidget(_VectDock.back());
	title_.push_back(new QCPPlotTitle(_VectPlot.back()));
	title_.back()->setText("Muscle Force ");
	title_.back()->setFont(QFont("sans", SIZEFONTTITLE, QFont::Bold));
	title_.back()->setTextColor(QColor(255, 131, 0));
	_VectPlot.back()->plotLayout()->insertRow(0);
	_VectPlot.back()->plotLayout()->addElement(0, 0, title_.back());
	_bar = new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis);
	_VectPlot.back()->addPlottable(_bar);
	QPen pen;
	pen.setWidthF(1.2);
	_bar->setName("Muscle Force");
	pen.setColor(QColor(255, 131, 0));
	pen.setWidthF(3);
	_bar->setPen(pen);
	_bar->setBrush(QColor(255, 131, 0, 50));
	QVector<double> ticks;
	QVector<QString> labels;

	for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
		it != _musclesNames.end(); it++)
	{
		//std::cout << *it << std::endl << std::flush;
		const int& cpt =
			std::distance<std::vector<std::string>::const_iterator>(
			_musclesNames.begin(), it);
		labels << QString(it->c_str());
		ticks << cpt;
		// 		barData << cpt;
	}

	_VectPlot.back()->xAxis->setAutoTicks(false);
	_VectPlot.back()->xAxis->setAutoTickLabels(false);
	_VectPlot.back()->xAxis->setTickVector(ticks);
	_VectPlot.back()->xAxis->setTickLabelFont(QFont("sans", SIZEFONT));
	_VectPlot.back()->yAxis->setTickLabelFont(QFont("sans", SIZEFONT));
	_VectPlot.back()->xAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
	_VectPlot.back()->yAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
	_VectPlot.back()->xAxis->setTickVectorLabels(labels);
	_VectPlot.back()->xAxis->setTickLabelRotation(60);
	_VectPlot.back()->xAxis->setSubTickCount(0);
	_VectPlot.back()->xAxis->setTickLength(0, 4);
	_VectPlot.back()->xAxis->grid()->setVisible(true);
	_VectPlot.back()->xAxis->setRange(-1, _musclesNames.size());
	_VectPlot.back()->yAxis->setRange(0, _musclesNames.size());
	_VectPlot.back()->yAxis->setPadding(5); // a bit more space to the left border
	_VectPlot.back()->yAxis->setLabel("Newton");
	_VectDock.back()->hide();
	//_bar->setData ( ticks, barData );
	// 	_VectPlot.back()->setInteractions ( QCP::iRangeDrag | QCP::iRangeZoom );
	_VectDock.back()->setWidget(_VectPlot.back());
	_checkBoxBool.push_back(false);
}


void MainWindow::constructWorkLoopPlot()
{

	WLDataDemo_.resize(_musclesNames.size());
	forceDemo_.resize(_musclesNames.size());
	fiberLengthDemo_.resize(_musclesNames.size());
	forceDemoSave_.resize(4);
	fiberLengthDemoSave_.resize(4);
	timeDemoWLSave_.resize(4);
	demoWLCurve_.resize(5);

	for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
		it != _musclesNames.end(); it++)
	{
		std::stringstream emgText;
		emgText << "Work loop " + *it;
		_VectDock.push_back(new QDockWidget(emgText.str().c_str(), this));
		_VectPlot.push_back(new QCustomPlot(_VectDock.back()));
		//_VectDock.back()->setFeatures(QDockWidget::NoDockWidgetFeatures);
		title_.push_back(new QCPPlotTitle(_VectPlot.back()));
		_layout->addWidget(_VectDock.back());
		title_.back()->setText(emgText.str().c_str());
		title_.back()->setFont(QFont("sans", SIZEFONTTITLE, QFont::Bold));
		title_.back()->setTextColor(Qt::blue);
		_VectPlot.back()->plotLayout()->insertRow(0);
		_VectPlot.back()->plotLayout()->addElement(0, 0, title_.back());

		demoWLCurve_[4].push_back(new QCPCurve(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoWLCurve_[3].push_back(new QCPCurve(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoWLCurve_[2].push_back(new QCPCurve(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoWLCurve_[1].push_back(new QCPCurve(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoWLCurve_[0].push_back(new QCPCurve(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));

		_VectPlot.back()->addPlottable(demoWLCurve_[4].back());
		_VectPlot.back()->addPlottable(demoWLCurve_[3].back());
		_VectPlot.back()->addPlottable(demoWLCurve_[2].back());
		_VectPlot.back()->addPlottable(demoWLCurve_[1].back());
		_VectPlot.back()->addPlottable(demoWLCurve_[0].back());

		QPen bluePen;
		bluePen.setColor(QColor(0, 0, 200));
		bluePen.setWidthF(3);//3
		demoWLCurve_[4].back()->setPen(bluePen);
		demoWLCurve_[4].back()->setAntialiasedFill(false);
		demoWLCurve_[4].back()->setName("Current");

		demoWLCurve_[3].back()->setPen(WLColor_[0]);
		demoWLCurve_[3].back()->setAntialiasedFill(false);
		demoWLCurve_[3].back()->setName("Save 1");

		demoWLCurve_[2].back()->setPen(WLColor_[1]);
		demoWLCurve_[2].back()->setAntialiasedFill(false);
		demoWLCurve_[2].back()->setName("Save 2");

		demoWLCurve_[1].back()->setPen(WLColor_[2]);
		demoWLCurve_[1].back()->setAntialiasedFill(false);
		demoWLCurve_[1].back()->setName("Save 3");

		demoWLCurve_[0].back()->setPen(WLColor_[3]);
		demoWLCurve_[0].back()->setAntialiasedFill(false);
		demoWLCurve_[0].back()->setName("Save 4");

		_VectPlot.back()->yAxis->setLabel("Muscle Force");
		_VectPlot.back()->xAxis->setLabel("Muscle Length");
		_VectPlot.back()->xAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->yAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->xAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->yAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->legend->setVisible(true);
		_VectDock.back()->setWidget(_VectPlot.back());
		_VectDock.back()->hide();
		_nbOfPlot--;
		_checkBoxBool.push_back(false);
	}
}

void MainWindow::constructDemoBar()
{
	std::vector<std::string> musclesNamesAndAdd = _musclesNames;
	musclesNamesAndAdd.push_back("Plantarflexion");
	musclesNamesAndAdd.push_back("Dorsiflexion");
	musclesNamesAndAdd.push_back("Total");
	barDataDemo_.resize(musclesNamesAndAdd.size());
	EmgDemo_.resize(musclesNamesAndAdd.size());
	timeDemoSave_.resize(4);
	EMGDemoSave_.resize(musclesNamesAndAdd.size());
	labelDemo_.resize(4);
	demoBar_.resize(5);
	nbSampleDemo_ = 0;
	for (std::vector<std::string>::const_iterator it = musclesNamesAndAdd.begin();
		it != musclesNamesAndAdd.end(); it++)
	{
		const int& cpt = std::distance<std::vector<std::string>::const_iterator>(musclesNamesAndAdd.begin(), it);
		EMGDemoSave_[cpt].resize(4);

		std::stringstream emgText;
		emgText << "EMG Bar: " + *it;
		_VectDock.push_back(new QDockWidget(emgText.str().c_str(), this));
		_VectPlot.push_back(new QCustomPlot(_VectDock.back()));
		//_VectDock.back()->setFeatures(QDockWidget::NoDockWidgetFeatures);
		_layout->addWidget(_VectDock.back());
		_VectDock.back()->setAllowedAreas(Qt::AllDockWidgetAreas);
		title_.push_back(new QCPPlotTitle(_VectPlot.back()));
		title_.back()->setText(emgText.str().c_str());
		title_.back()->setFont(QFont("sans", SIZEFONTTITLE, QFont::Bold));
		title_.back()->setTextColor(Qt::blue);
		_VectPlot.back()->plotLayout()->insertRow(0);
		_VectPlot.back()->plotLayout()->addElement(0, 0, title_.back());
		demoBar_[0].push_back(new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoBar_[1].push_back(new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoBar_[2].push_back(new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoBar_[3].push_back(new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		demoBar_[4].push_back(new QCPBars(_VectPlot.back()->xAxis, _VectPlot.back()->yAxis));
		_VectPlot.back()->addPlottable(demoBar_[0].back());
		_VectPlot.back()->addPlottable(demoBar_[1].back());
		_VectPlot.back()->addPlottable(demoBar_[2].back());
		_VectPlot.back()->addPlottable(demoBar_[3].back());
		_VectPlot.back()->addPlottable(demoBar_[4].back());

		QPen pen;
		pen.setWidthF(1.2);
		pen.setColor(QColor(0, 0, 228));
		pen.setWidthF(3);
		demoBar_[4].back()->setPen(pen);
		demoBar_[3].back()->setPen(WLColor_[3]);
		demoBar_[2].back()->setPen(WLColor_[2]);
		demoBar_[1].back()->setPen(WLColor_[1]);
		demoBar_[0].back()->setPen(WLColor_[0]);
		demoBar_[4].back()->setBrush(QColor(0, 0, 228, 100));
		demoBar_[3].back()->setBrush(WLColor_[3].color());
		demoBar_[2].back()->setBrush(WLColor_[2].color());
		demoBar_[1].back()->setBrush(WLColor_[1].color());
		demoBar_[0].back()->setBrush(WLColor_[0].color());
		//demoBar_[0].back()->setBrush(QColor(0, 0, 228, 50));

		QVector<double> ticks;
		QVector<QString> labels;

		barDataDemo_[cpt].push_back(0);
		labelDemo_[0] = QString("Save 4");
		labels << labelDemo_[0];
		ticks << 1;
		barDataDemo_[cpt].push_back(0);
		labelDemo_[1] = QString("Save 3");
		labels << labelDemo_[1];
		ticks << 2;
		barDataDemo_[cpt].push_back(0);
		labelDemo_[2] = QString("Save 2");
		labels << labelDemo_[2];
		ticks << 3;
		barDataDemo_[cpt].push_back(0);
		labelDemo_[3] = QString("Save 1");
		labels << labelDemo_[3];
		ticks << 4;
		barDataDemo_[cpt].push_back(0);
		labels << QString("Current");
		ticks << 5;

		QPen bluePen;
		bluePen.setColor(QColor(0, 0, 200));
		bluePen.setWidthF(3);
		_VectPlot.back()->addGraph(_VectPlot.back()->xAxis2, _VectPlot.back()->yAxis2);
		_VectPlot.back()->graph(0)->setPen(bluePen);
		_VectPlot.back()->graph(0)->setAntialiasedFill(false);
		_VectPlot.back()->graph(0)->setName("Current");

		_VectPlot.back()->addGraph(_VectPlot.back()->xAxis2, _VectPlot.back()->yAxis2);
		_VectPlot.back()->graph(1)->setPen(barColor_[0]);
		_VectPlot.back()->graph(1)->setAntialiasedFill(false);
		_VectPlot.back()->graph(1)->setName(labelDemo_[0]);

		_VectPlot.back()->addGraph(_VectPlot.back()->xAxis2, _VectPlot.back()->yAxis2);
		_VectPlot.back()->graph(2)->setPen(barColor_[1]);
		_VectPlot.back()->graph(2)->setAntialiasedFill(false);
		_VectPlot.back()->graph(2)->setName(labelDemo_[1]);

		_VectPlot.back()->addGraph(_VectPlot.back()->xAxis2, _VectPlot.back()->yAxis2);
		_VectPlot.back()->graph(3)->setPen(barColor_[2]);
		_VectPlot.back()->graph(3)->setAntialiasedFill(false);
		_VectPlot.back()->graph(3)->setName(labelDemo_[2]);

		_VectPlot.back()->addGraph(_VectPlot.back()->xAxis2, _VectPlot.back()->yAxis2);
		_VectPlot.back()->graph(4)->setPen(barColor_[3]);
		_VectPlot.back()->graph(4)->setAntialiasedFill(false);
		_VectPlot.back()->graph(4)->setName(labelDemo_[3]);

		_VectPlot.back()->xAxis->setAutoTicks(false);
		_VectPlot.back()->xAxis->setAutoTickLabels(false);
		_VectPlot.back()->xAxis->setTickVector(ticks);
		_VectPlot.back()->xAxis->setTickVectorLabels(labels);
		_VectPlot.back()->xAxis->setTickLabelRotation(60);
		_VectPlot.back()->xAxis->setSubTickCount(0);
		_VectPlot.back()->xAxis->setTickLength(0, 5);
		_VectPlot.back()->xAxis->grid()->setVisible(true);
		_VectPlot.back()->xAxis->setRange(0, 6);
		_VectPlot.back()->yAxis->setRange(0, 0.001);
		_VectPlot.back()->yAxis->setPadding(5); // a bit more space to the left border
		_VectPlot.back()->yAxis->setLabel("EMG Normalized RMS");
		_VectPlot.back()->xAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->yAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->xAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->yAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));

		_VectPlot.back()->xAxis2->setVisible(true);
		_VectPlot.back()->yAxis2->setVisible(true);
		_VectPlot.back()->xAxis2->setLabel("Time");
		_VectPlot.back()->yAxis2->setLabel("EMG Normalized");
		_VectPlot.back()->xAxis2->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->yAxis2->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->xAxis2->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->yAxis2->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->xAxis2->setRange(0, 10);
		_VectPlot.back()->yAxis2->setRange(0, 1.1);

		_VectDock.back()->setWidget(_VectPlot.back());
		_VectDock.back()->hide();
		_nbOfPlot--;
		_checkBoxBool.push_back(false);
	}
}

void MainWindow::constructEMGPlot()
{
	for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
		it != _musclesNames.end(); it++)
	{
		std::stringstream emgText;
		emgText << "EMG " + *it;
		_VectDock.push_back(new QDockWidget(emgText.str().c_str(), this));
		_VectPlot.push_back(new QCustomPlot(_VectDock.back()));
		//_VectDock.back()->setFeatures(QDockWidget::NoDockWidgetFeatures);
		title_.push_back(new QCPPlotTitle(_VectPlot.back()));
		_layout->addWidget(_VectDock.back());
		title_.back()->setText(emgText.str().c_str());
		title_.back()->setFont(QFont("sans", SIZEFONTTITLE, QFont::Bold));
		title_.back()->setTextColor(Qt::blue);
		_VectPlot.back()->plotLayout()->insertRow(0);
		_VectPlot.back()->plotLayout()->addElement(0, 0, title_.back());
		_VectPlot.back()->addGraph(); // blue line
		QPen bluePen;
		bluePen.setColor(Qt::blue);
		bluePen.setWidthF(3);//3
		_VectPlot.back()->graph(0)->setPen(bluePen);
		_VectPlot.back()->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
		_VectPlot.back()->graph(0)->setAntialiasedFill(false);

		_VectPlot.back()->xAxis->setTickLabelType(QCPAxis::ltDateTime);
		_VectPlot.back()->xAxis->setDateTimeFormat("hh:mm:ss");
		_VectPlot.back()->xAxis->setAutoTickStep(false);
		_VectPlot.back()->xAxis->setTickStep(2);
		_VectPlot.back()->axisRect()->setupFullAxesBox();

		// make left and bottom axes transfer their ranges to right and top axes:
		connect(_VectPlot.back()->xAxis, SIGNAL(rangeChanged(QCPRange)),
			_VectPlot.back()->xAxis2, SLOT(setRange(QCPRange)));
		connect(_VectPlot.back()->yAxis, SIGNAL(rangeChanged(QCPRange)),
			_VectPlot.back()->yAxis2, SLOT(setRange(QCPRange)));
		_VectPlot.back()->yAxis->setLabel("Normalize");
		_VectPlot.back()->xAxis->setLabel("Time");
		_VectPlot.back()->xAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->yAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->xAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->yAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectDock.back()->setWidget(_VectPlot.back());
		_VectDock.back()->hide();
		_nbOfPlot--;
		_checkBoxBool.push_back(false);
	}
}

void MainWindow::constructTorquePlot()
{
	for (std::vector<std::string>::const_iterator it = _dofNames.begin();
		it != _dofNames.end(); it++)
	{
		std::stringstream torqueText;
		torqueText << "Torque " + *it;
		_VectDock.push_back(new QDockWidget(torqueText.str().c_str(), this));
		_VectPlot.push_back(new QCustomPlot(_VectDock.back()));
		//_VectDock.back()->setFeatures(QDockWidget::NoDockWidgetFeatures);
		title_.push_back(new QCPPlotTitle(_VectPlot.back()));
		_layout->addWidget(_VectDock.back());
		title_.back()->setText(torqueText.str().c_str());
		title_.back()->setFont(QFont("sans", SIZEFONTTITLE, QFont::Bold));
		title_.back()->setTextColor(Qt::darkGreen);
		_VectPlot.back()->plotLayout()->insertRow(0);
		_VectPlot.back()->plotLayout()->addElement(0, 0, title_.back());

		if (addTorqueCEINMS_)
		{
			_VectPlot.back()->addGraph(); // blue line
			QPen bluePen;
			bluePen.setColor(Qt::blue);
			bluePen.setWidthF(3);
			_VectPlot.back()->graph(0)->setPen(bluePen);
			// 			_VectPlot.back()->graph ( 0 )->setBrush ( QBrush ( QColor ( 240, 255, 200 ) ) );
			_VectPlot.back()->graph(0)->setAntialiasedFill(false);
			_VectPlot.back()->graph(0)->setName("NMS");

			if (addTorqueID_)
			{
				_VectPlot.back()->addGraph(); // red line
				QPen redPen;
				redPen.setColor(Qt::red);
				redPen.setWidthF(3);//3
				_VectPlot.back()->graph(1)->setPen(redPen);
				_VectPlot.back()->graph(1)->setAntialiasedFill(false);
				_VectPlot.back()->graph(1)->setName("ID");
			}

			if (addTorqueMotor_)
			{
				_VectPlot.back()->addGraph(); // green line
				QPen greenPen;
				greenPen.setColor(Qt::green);
				greenPen.setWidthF(3);
				_VectPlot.back()->graph(2)->setPen(greenPen);
				_VectPlot.back()->graph(2)->setAntialiasedFill(false);
				_VectPlot.back()->graph(2)->setName("Motor");
			}
		}
		else if (addTorqueID_)
		{
			_VectPlot.back()->addGraph(); // blue line
			QPen bluePen;
			bluePen.setColor(Qt::blue);
			bluePen.setWidthF(3);
			_VectPlot.back()->graph(0)->setPen(bluePen);
			_VectPlot.back()->graph(0)->setAntialiasedFill(false);
			_VectPlot.back()->graph(0)->setName("ID");

			if (addTorqueMotor_)
			{
				_VectPlot.back()->addGraph(); // red line
				QPen redPen;
				redPen.setColor(Qt::red);
				redPen.setWidthF(3);
				_VectPlot.back()->graph(1)->setPen(redPen);
				_VectPlot.back()->graph(1)->setAntialiasedFill(false);
				_VectPlot.back()->graph(1)->setName("Motor");
			}
		}
		else if (addTorqueMotor_)
		{
			_VectPlot.back()->addGraph(); // blue line
			QPen bluePen;
			bluePen.setColor(Qt::blue);
			bluePen.setWidthF(3);
			_VectPlot.back()->graph(0)->setPen(bluePen);
			_VectPlot.back()->graph(0)->setAntialiasedFill(false);
			_VectPlot.back()->graph(0)->setName("Motor");
		}

		_VectPlot.back()->xAxis->setTickLabelType(QCPAxis::ltDateTime);
		_VectPlot.back()->xAxis->setDateTimeFormat("hh:mm:ss");
		_VectPlot.back()->xAxis->setAutoTickStep(false);
		_VectPlot.back()->xAxis->setTickStep(2);
		_VectPlot.back()->axisRect()->setupFullAxesBox();
		_VectPlot.back()->legend->setVisible(true);
		_VectPlot.back()->yAxis->setLabel("N.m");
		_VectPlot.back()->xAxis->setLabel("Time");
		_VectPlot.back()->xAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->yAxis->setTickLabelFont(QFont("sans", SIZEFONT));
		_VectPlot.back()->xAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));
		_VectPlot.back()->yAxis->setLabelFont(QFont("sans", SIZEFONTLABEL));

		// make left and bottom axes transfer their ranges to right and top axes:
		connect(_VectPlot.back()->xAxis, SIGNAL(rangeChanged(QCPRange)),
			_VectPlot.back()->xAxis2, SLOT(setRange(QCPRange)));
		_VectDock.back()->setWidget(_VectPlot.back());
		_VectDock.back()->hide();
		_nbOfPlot--;
		_checkBoxBool.push_back(false);
	}
}

void MainWindow::createMenus()
{
	fileMenu_ = menuBar()->addMenu(tr("&File"));
	fileMenu_->addAction(openAct_);
	fileMenu_->addSeparator();
	fileMenu_->addAction(exitAct_);

	windowsMenu_ = menuBar()->addMenu(tr("&Windows"));
	windowsMenu_->addAction(plotPanel_);
	windowsMenu_->addAction(treePlotPanel_);

	if (addCalibrationPanel_)
		windowsMenu_->addAction(calibrationPanel_);

	if (addTimingPanel_)
		windowsMenu_->addAction(timingPanel_);

#ifdef USE_OPENSIM
	if (add3DIK_)
		windowsMenu_->addAction(openGLPanel_);
#endif

	plotMenu_ = menuBar()->addMenu(tr("&Plot"));

	for (std::vector<QAction*>::iterator it = plotAct_.begin(); it != plotAct_.end(); it++)
		plotMenu_->addAction(*it);

	helpMenu_ = menuBar()->addMenu(tr("&Help"));
	helpMenu_->addAction(aboutAct_);
}

void MainWindow::createActions()
{
	openAct_ = new QAction(tr("&Open"), this);
	openAct_->setStatusTip(tr("Open new file"));

	exitAct_ = new QAction(tr("&Exit"), this);
	exitAct_->setStatusTip(tr("Exit CEINMS"));
	connect(exitAct_, SIGNAL(triggered()), this, SLOT(exit()));

	plotPanel_ = new QAction(tr("&PLot"), this);
	plotPanel_->setStatusTip(tr("Open/Close the plot panel"));
	plotPanel_->setCheckable(true);
	plotPanel_->setChecked(true);
	connect(plotPanel_, SIGNAL(triggered()), this, SLOT(viewPlotPanel()));

	treePlotPanel_ = new QAction(tr("&Tree Plot"), this);
	treePlotPanel_->setStatusTip(tr("Open/Close the tree plot panel"));
	treePlotPanel_->setCheckable(true);
	treePlotPanel_->setChecked(true);
	connect(treePlotPanel_, SIGNAL(triggered()), this, SLOT(viewTreePlotPanel()));

#ifdef USE_OPENSIM

	if (add3DIK_)
	{
		openGLPanel_ = new QAction(tr("&3d view"), this);
		openGLPanel_->setStatusTip(tr("Open/Close the 3d view panel"));
		openGLPanel_->setCheckable(true);
		openGLPanel_->setChecked(true);
		connect(openGLPanel_, SIGNAL(triggered()), this, SLOT(viewOpenGLPanel()));
	}

#endif
	if (addCalibrationPanel_)
	{
		calibrationPanel_ = new QAction(tr("&Calibration"), this);
		calibrationPanel_->setStatusTip(tr("Open/Close the Calibration parameter panel"));
		calibrationPanel_->setCheckable(true);
		calibrationPanel_->setChecked(false);
		connect(calibrationPanel_, SIGNAL(triggered()), this, SLOT(viewCalibrationPanel()));
	}

	if (addTimingPanel_)
	{
		timingPanel_ = new QAction(tr("&Timing"), this);
		timingPanel_->setStatusTip(tr("Open/Close the Timing panel"));
		timingPanel_->setCheckable(true);
		timingPanel_->setChecked(false);
		connect(timingPanel_, SIGNAL(triggered()), this, SLOT(viewTimingPanel()));
	}

	aboutAct_ = new QAction(tr("&About"), this);
	aboutAct_->setStatusTip(tr("About CEINMS-RT"));
	connect(aboutAct_, SIGNAL(triggered()), this, SLOT(about()));

	signalMapperPlot_ = new QSignalMapper(this);

	if (addMuscleForceBar_)
	{
		plotAct_.push_back(new QAction(tr("Muscle Force"), this));
		plotAct_.back()->setCheckable(true);
		plotAct_.back()->setChecked(false);
		connect(plotAct_.back(), SIGNAL(triggered()), signalMapperPlot_, SLOT(map()));
		signalMapperPlot_->setMapping(plotAct_.back(), 0);
	}

	if (AddEMG_)
	{
		for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
		{
			const int& cpt = std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), it);
			plotAct_.push_back(new QAction(QString(it->c_str()), this));
			plotAct_.back()->setCheckable(true);
			plotAct_.back()->setChecked(false);
			connect(plotAct_.back(), SIGNAL(triggered()), signalMapperPlot_, SLOT(map()));
			signalMapperPlot_->setMapping(plotAct_.back(), cpt + 1);
		}
	}

	if (addTorqueID_ || addTorqueCEINMS_ || addTorqueMotor_)
	{
		for (std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++)
		{
			const int& cpt = std::distance<std::vector<std::string>::const_iterator>(_dofNames.begin(), it);
			plotAct_.push_back(new QAction(QString(it->c_str()), this));
			plotAct_.back()->setCheckable(true);
			plotAct_.back()->setChecked(false);
			connect(plotAct_.back(), SIGNAL(triggered()), signalMapperPlot_, SLOT(map()));
			signalMapperPlot_->setMapping(plotAct_.back(), cpt + 1 + _musclesNames.size());
		}
	}

	connect(signalMapperPlot_, SIGNAL(mapped(int)), this, SLOT(viewplot(int)));
}

void MainWindow::viewplot(int plotNumber)
{
	if (_VectDock.at(plotNumber)->isVisible())
	{
		_VectDock.at(plotNumber)->hide();
		_nbOfPlot--;

		if (addMuscleForceBar_ && AddEMG_ && (addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_) && addDemo_)
		{
			if (plotNumber > 0 && plotNumber < _muscleNameOnChannel.size())
				_VectTreeItem.at(plotNumber + 1)->setCheckState(0, Qt::Unchecked);
			else if (plotNumber > _muscleNameOnChannel.size() && plotNumber < _muscleNameOnChannel.size() + _dofNames.size())
				_VectTreeItem.at(plotNumber + 2)->setCheckState(0, Qt::Unchecked);
			else if (plotNumber > _muscleNameOnChannel.size() + _dofNames.size())
				_VectTreeItem.at(plotNumber + 3)->setCheckState(0, Qt::Unchecked);
			else 
				_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Unchecked);
		}
		else if ((addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_ || AddEMG_) && addMuscleForceBar_)
		{
			if (plotNumber > 0)
				_VectTreeItem.at(plotNumber + 1)->setCheckState(0, Qt::Unchecked);
			else
				_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Unchecked);
		}
		else
			_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Unchecked);
	}
	else
	{
		_VectDock.at(plotNumber)->show();
		_nbOfPlot++;

		if (addMuscleForceBar_ && AddEMG_ && (addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_))
		{
			if (plotNumber > 0 && plotNumber < _muscleNameOnChannel.size())
				_VectTreeItem.at(plotNumber + 1)->setCheckState(0, Qt::Checked);
			else if (plotNumber > _muscleNameOnChannel.size() && plotNumber < _muscleNameOnChannel.size() + _dofNames.size())
				_VectTreeItem.at(plotNumber + 2)->setCheckState(0, Qt::Checked);
			else if (plotNumber > _muscleNameOnChannel.size() + _dofNames.size())
				_VectTreeItem.at(plotNumber + 3)->setCheckState(0, Qt::Checked);
			else
				_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Checked);
		}
		else if ((addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_ || AddEMG_) && addMuscleForceBar_)
		{
			if (plotNumber > 0)
				_VectTreeItem.at(plotNumber + 1)->setCheckState(0, Qt::Checked);
			else
				_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Checked);
		}
		else
			_VectTreeItem.at(plotNumber)->setCheckState(0, Qt::Checked);
	}

	if (_nbOfPlot < 3)
	{
		_viewport->setMaximumHeight(deskHeight_);
		_viewport->setMinimumHeight(deskHeight_);
	}
	else
	{
		_viewport->setMaximumHeight(_nbOfPlot * deskHeight_ / 3);
		_viewport->setMinimumHeight(_nbOfPlot * deskHeight_ / 3);
	}
}

void MainWindow::viewPlotPanel()
{
	if (_scrollArea->isVisible())
		_scrollArea->hide();
	else
		_scrollArea->show();
}

void MainWindow::viewTreePlotPanel()
{
	if (_scrollAreaCheck->isVisible())
		_scrollAreaCheck->hide();
	else
		_scrollAreaCheck->show();
}

void MainWindow::viewOpenGLPanel()
{
#ifdef USE_OPENSIM
	if (add3DIK_)
	{
		if (_openglWin->isVisible())
			_openglWin->hide();
		else
			_openglWin->show();
	}
#endif
}

void MainWindow::viewTimingPanel()
{
	if (_leftVLayoutWidget->isVisible())
		_leftVLayoutWidget->hide();
	else
		_leftVLayoutWidget->show();
}

void MainWindow::viewCalibrationPanel()
{
}

void MainWindow::exit()
{
	this->close();
	stop();
}

void MainWindow::close() // public close function
{

	exit(); // load the private exit function that defined before

}

void MainWindow::about()
{
	QMessageBox::about(this, tr("About CEINMS-RT"),
		tr("CEINMS-RT:\nCalibrated EMG-informed Neuromusculoskeletal real-time toolbox\n\nby:\nDavid LLoyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato, Guillaume Durandau"));
}

void MainWindow::checkTreeBox()
{
	unsigned int cpt = 0;
	unsigned int cptDock = 0;

	if (addMuscleForceBar_)
	{
		if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Unchecked
			&& _checkBoxBool.at(cptDock) == true)
		{
			_VectDock.at(cptDock)->hide();
			_nbOfPlot--;
			_checkBoxBool[cpt] = false;
			plotAct_.at(cptDock)->setChecked(false);
		}
		else if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Checked
			&& _checkBoxBool.at(cptDock) == false)
		{
			_VectDock.at(cptDock)->show();
			_nbOfPlot++;
			_checkBoxBool[cpt] = true;
			plotAct_.at(cptDock)->setChecked(true);
		}

		cpt++;
		cpt++;
		cptDock++;
	}

	if (AddEMG_)
	{
		for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
		{
			if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Unchecked
				&& _checkBoxBool.at(cptDock) == true)
			{
				_VectDock.at(cptDock)->hide();
				_nbOfPlot--;
				_checkBoxBool[cptDock] = false;
				plotAct_.at(cptDock)->setChecked(false);
			}
			else if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Checked
				&& _checkBoxBool.at(cptDock) == false)
			{
				_VectDock.at(cptDock)->show();
				_nbOfPlot++;
				_checkBoxBool[cptDock] = true;
				plotAct_.at(cptDock)->setChecked(true);
			}

			cpt++;
			cptDock++;
		}

		cpt++;
	}

	if (addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_)
	{
		for (std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++)
		{
			if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Unchecked
				&& _checkBoxBool.at(cptDock) == true)
			{
				_VectDock.at(cptDock)->hide();
				_nbOfPlot--;
				_checkBoxBool[cptDock] = false;
				plotAct_.at(cptDock)->setChecked(false);
			}
			else if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Checked
				&& _checkBoxBool.at(cptDock) == false)
			{
				_VectDock.at(cptDock)->show();
				_nbOfPlot++;
				_checkBoxBool[cptDock] = true;
				plotAct_.at(cptDock)->setChecked(true);
			}

			cpt++;
			cptDock++;
		}
		cpt++;
	}

	if (addDemo_)
	{
		for (int it = 0; it != 2*_musclesNames.size() + 3; it++)
		{
			if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Unchecked
				&& _checkBoxBool.at(cptDock) == true)
			{
				_VectDock.at(cptDock)->hide();
				_VectDock.at(cptDock)->setFloating(false);
				_nbOfPlot--;
				_checkBoxBool[cptDock] = false;
				//plotAct_.at(cptDock)->setChecked(false);
			}
			else if (_VectTreeItem.at(cpt)->checkState(0) == Qt::Checked
				&& _checkBoxBool.at(cptDock) == false)
			{
				_VectDock.at(cptDock)->show();
				_nbOfPlot++;
				_checkBoxBool[cptDock] = true;
				//plotAct_.at(cptDock)->setChecked(true);
			}

			cpt++;
			cptDock++;
		}
	}

	if (_nbOfPlot < 3)
	{
		_viewport->setMaximumHeight(deskHeight_);
		_viewport->setMinimumHeight(deskHeight_);
	}
	else
	{
		_viewport->setMaximumHeight(_nbOfPlot * deskHeight_ / 3);
		_viewport->setMinimumHeight(_nbOfPlot * deskHeight_ / 3);
	}
}

void MainWindow::checkSaveonPDF()
{
	unsigned int cpt = 0;

	if (addMuscleForceBar_)
	{
		if (_VectButtonItem.at(cpt)->isDown() == true)
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "Muscle_Force_" << date.toStdString() << ".pdf";
			_VectPlot.at(cpt)->savePdf(QString(text.str().c_str()));
		}

		cpt++;
	}

	if (AddEMG_)
	{
		for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
		{
			if (_VectButtonItem.at(cpt)->isDown() == true)
			{
				QString date = QDateTime::currentDateTime().toString();
				std::stringstream text;
				text << "EMG_" << *it << "_" << date.toStdString() << ".pdf";
				_VectPlot.at(cpt)->savePdf(QString(text.str().c_str()));
			}

			cpt++;
		}
	}

	if (addTorqueCEINMS_ || addTorqueID_ || addTorqueMotor_)
	{
		for (std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++)
		{
			if (_VectButtonItem.at(cpt)->isDown() == true)
			{
				QString date = QDateTime::currentDateTime().toString();
				std::stringstream text;
				text << "Torque_" << *it << "_" << date.toStdString() << ".pdf";
				_VectPlot.at(cpt)->savePdf(QString(text.str().c_str()));
			}

			cpt++;
		}
	}

	if (addDemo_)
	{
		cpt++;
		for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
		{
			if (_VectButtonItem.at(cpt)->isDown() == true)
			{
				QString date = QDateTime::currentDateTime().toString();
				std::stringstream text;
				text << "EMGBar_" << *it << "_" << date.toStdString() << ".pdf";
				_VectPlot.at(cpt-1)->savePdf(QString(text.str().c_str()));
			}
			cpt++;
		}
		if (_VectButtonItem.at(cpt)->isDown() == true)
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "EMGBar_Plantarflexion_" << date.toStdString() << ".pdf";
			_VectPlot.at(cpt - 1)->savePdf(QString(text.str().c_str()));
		}
		cpt++;
		if (_VectButtonItem.at(cpt)->isDown() == true)
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "EMGBar_Dorsiflexion_" << date.toStdString() << ".pdf";
			_VectPlot.at(cpt - 1)->savePdf(QString(text.str().c_str()));
		}
		cpt++;
		if (_VectButtonItem.at(cpt)->isDown() == true)
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "EMGBar_Total_" << date.toStdString() << ".pdf";
			_VectPlot.at(cpt - 1)->savePdf(QString(text.str().c_str()));
		}
		cpt++;
	}
}

void MainWindow::resetPressed()
{
	std::cout << "Button reset pressed!" << std::endl;
	for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
		it != _musclesNames.end(); it++)
	{
		const int& cptM = std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), it);
		EmgDemo_[cptM].clear();
		timeDemo_.clear();
		nbSampleDemo_ = 0;

		timeDemoWL_.clear();
		fiberLengthDemo_[cptM].clear();
		forceDemo_[cptM].clear();
	}
}

void MainWindow::savePressed()
{

	std::cout << "Button save pressed!" << std::endl;
	labelDemo_[0] = labelDemo_[1];
	labelDemo_[1] = labelDemo_[2];
	labelDemo_[2] = labelDemo_[3];
	labelDemo_[3] = lineEditDemo_->text();

	QVector<QString> labels;
	labels << labelDemo_[0];
	labels << labelDemo_[1];
	labels << labelDemo_[2];
	labels << labelDemo_[3];
	labels << QString("Current");

	QPen tempColor = WLColor_[0];
	WLColor_[0] = WLColor_[1];
	WLColor_[1] = WLColor_[2];
	WLColor_[2] = WLColor_[3];
	WLColor_[3] = tempColor;

	tempColor = barColor_[0];
	barColor_[0] = barColor_[1];
	barColor_[1] = barColor_[2];
	barColor_[2] = barColor_[3];
	barColor_[3] = tempColor;

	double initValTime = timeDemo_[0];
	for (QVector<double>::iterator it = timeDemo_.begin(); it != timeDemo_.end(); it++)
		*it -= initValTime;

	timeDemoSave_[0] = timeDemoSave_[1];
	timeDemoSave_[1] = timeDemoSave_[2];
	timeDemoSave_[2] = timeDemoSave_[3];
	timeDemoSave_[3] = timeDemo_;

	//nbSampleDemo_ = 0;

	for (std::vector<std::string>::const_iterator it = _musclesNames.begin();
		it != _musclesNames.end(); it++)
	{
		const int& cptM = std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), it);

		barDataDemo_[cptM][0] = barDataDemo_[cptM][1];
		barDataDemo_[cptM][1] = barDataDemo_[cptM][2];
		barDataDemo_[cptM][2] = barDataDemo_[cptM][3];
		barDataDemo_[cptM][3] = barDataDemo_[cptM][4];

		EMGDemoSave_[cptM][0] = EMGDemoSave_[cptM][1];
		EMGDemoSave_[cptM][1] = EMGDemoSave_[cptM][2];
		EMGDemoSave_[cptM][2] = EMGDemoSave_[cptM][3];
		EMGDemoSave_[cptM][3] = EmgDemo_[cptM];


		demoBar_[3][cptM]->setPen(WLColor_[3]);
		demoBar_[2][cptM]->setPen(WLColor_[2]);
		demoBar_[1][cptM]->setPen(WLColor_[1]);
		demoBar_[0][cptM]->setPen(WLColor_[0]);
		demoBar_[3][cptM]->setBrush(WLColor_[3].color());
		demoBar_[2][cptM]->setBrush(WLColor_[2].color());
		demoBar_[1][cptM]->setBrush(WLColor_[1].color());
		demoBar_[0][cptM]->setBrush(WLColor_[0].color());


		//EmgDemo_[cptM].clear();

		if (timeDemoSave_[3].size() != 0)
		{
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(3)->setData(timeDemoSave_[3], EMGDemoSave_[cptM][3]);
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(0)->setPen(barColor_[0]);
		}
		if (timeDemoSave_[2].size() != 0)
		{
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(2)->setData(timeDemoSave_[2], EMGDemoSave_[cptM][2]);
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(0)->setPen(barColor_[1]);
		}
		if (timeDemoSave_[1].size() != 0)
		{
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(1)->setData(timeDemoSave_[1], EMGDemoSave_[cptM][1]);
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(0)->setPen(barColor_[2]);
		}
		if (timeDemoSave_[0].size() != 0)
		{
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(0)->setData(timeDemoSave_[0], EMGDemoSave_[cptM][0]);
			_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->graph(0)->setPen(barColor_[3]);
		}

		_VectPlot[_musclesNames.size() + _dofNames.size() + 1 + cptM]->xAxis->setTickVectorLabels(labels);
	}

	//timeDemo_.clear();

	for (int i = 0; i < 3; i++)
	{
		demoBar_[3][_musclesNames.size() + i]->setPen(WLColor_[3]);
		demoBar_[2][_musclesNames.size() + i]->setPen(WLColor_[2]);
		demoBar_[1][_musclesNames.size() + i]->setPen(WLColor_[1]);
		demoBar_[0][_musclesNames.size() + i]->setPen(WLColor_[0]);
		demoBar_[3][_musclesNames.size() + i]->setBrush(WLColor_[3].color());
		demoBar_[2][_musclesNames.size() + i]->setBrush(WLColor_[2].color());
		demoBar_[1][_musclesNames.size() + i]->setBrush(WLColor_[1].color());
		demoBar_[0][_musclesNames.size() + i]->setBrush(WLColor_[0].color());

		barDataDemo_[_musclesNames.size() + i][0] = barDataDemo_[_musclesNames.size() + i][1];
		barDataDemo_[_musclesNames.size() + i][1] = barDataDemo_[_musclesNames.size() + i][2];
		barDataDemo_[_musclesNames.size() + i][2] = barDataDemo_[_musclesNames.size() + i][3];
		barDataDemo_[_musclesNames.size() + i][3] = barDataDemo_[_musclesNames.size() + i][4];
		_VectPlot[_musclesNames.size() + _dofNames.size() + _musclesNames.size() + 1 + i]->xAxis->setTickVectorLabels(labels);
	}
	
	forceDemoSave_[0] = forceDemoSave_[1];
	forceDemoSave_[1] = forceDemoSave_[2];
	forceDemoSave_[2] = forceDemoSave_[3];
	forceDemoSave_[3] = forceDemo_;

	fiberLengthDemoSave_[0] = fiberLengthDemoSave_[1];
	fiberLengthDemoSave_[1] = fiberLengthDemoSave_[2];
	fiberLengthDemoSave_[2] = fiberLengthDemoSave_[3];
	fiberLengthDemoSave_[3] = fiberLengthDemo_;

	timeDemoWLSave_[0] = timeDemoWLSave_[1];
	timeDemoWLSave_[1] = timeDemoWLSave_[2];
	timeDemoWLSave_[2] = timeDemoWLSave_[3];
	timeDemoWLSave_[3] = timeDemoWL_;

	//timeDemoWL_.clear();

	for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
	{
		const int& cptM = std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), it);
		//fiberLengthDemo_[cptM].clear();
		//forceDemo_[cptM].clear();

		demoWLCurve_[3][cptM]->setName(labelDemo_[3]);
		demoWLCurve_[2][cptM]->setName(labelDemo_[2]);
		demoWLCurve_[1][cptM]->setName(labelDemo_[1]);
		demoWLCurve_[0][cptM]->setName(labelDemo_[0]);

		demoWLCurve_[3][cptM]->setPen(WLColor_[3]);
		demoWLCurve_[2][cptM]->setPen(WLColor_[2]);
		demoWLCurve_[1][cptM]->setPen(WLColor_[1]);
		demoWLCurve_[0][cptM]->setPen(WLColor_[0]);

		if (timeDemoWLSave_[3].size() != 0)
		{
			demoWLCurve_[3][cptM]->setData(timeDemoWLSave_[3], fiberLengthDemoSave_[3].at(cptM), forceDemoSave_[3].at(cptM));
		}
		if (timeDemoWLSave_[2].size() != 0)
		{
			demoWLCurve_[2][cptM]->setData(timeDemoWLSave_[2], fiberLengthDemoSave_[2].at(cptM), forceDemoSave_[2].at(cptM));
		}
		if (timeDemoWLSave_[1].size() != 0)
		{
			demoWLCurve_[1][cptM]->setData(timeDemoWLSave_[1], fiberLengthDemoSave_[1].at(cptM), forceDemoSave_[1].at(cptM));
		}
		if (timeDemoWLSave_[0].size() != 0)
		{
			demoWLCurve_[0][cptM]->setData(timeDemoWLSave_[0], fiberLengthDemoSave_[0].at(cptM), forceDemoSave_[0].at(cptM));
		}
	}

}

void MainWindow::updateTiming()
{
	if (_timeEmg.size() > 1)
	{
		for (std::vector<double>::const_iterator it = _timeEmg.begin(); it != _timeEmg.end() - 1; it++)
		{
			if (*(it + 1) - *it != 0)
			{
				nbofEMG_++;
				emgTotalPeriod_ += *(it + 1) - *it;
			}
		}
		_timeEmg.clear();
	}
	if (_timeLMT.size() > 1)
	{
		for (std::vector<double>::const_iterator it = _timeLMT.begin(); it != _timeLMT.end() - 1; it++)
		{
			if (*(it + 1) - *it != 0)
			{
				nbofLMT_++;
				lmtTotalPeriod_ += *(it + 1) - *it;
			}
		}
		_timeLMT.clear();
	}
	if (_timeTorque.size() > 1)
	{
		for (std::vector<double>::const_iterator it = _timeTorque.begin(); it != _timeTorque.end() - 1; it++)
		{
			if (*(it + 1) - *it != 0)
			{
				nbofTorque_++;
				torqueTotalPeriod_ += *(it + 1) - *it;
			}
		}
	}
	if (_timePos.size() > 1)
	{
		for (std::vector<double>::const_iterator it = _timePos.begin(); it != _timePos.end() - 1; it++)
		{
			if (*(it + 1) - *it != 0)
			{
				nbofPos_++;
				posTotalPeriod_ += *(it + 1) - *it;
			}
		}
		_timePos.clear();
	}
	QString text = QString(
		"NMS Computation Time:\n%1 ms\n\nMTU Spline Computation time:\n%2 ms\n\n Total Delay:\n%3ms\n\nTotal MTU frames:\n%4\n\nTotal NMS frames:\n%5\n\nTotal EMG frames:\n%6\n\nGUI time: \n%7ms\n\nEMG (processed) Framerate:\n%8Hz\n\nPosition Framerate:\n%9Hz\n\nMTU Framerate:\n %10Hz\n\nNMS Framerate:\n%11Hz\n").arg(
		(_nmsTimeComput / _nbNMS) * 1000).arg((_mtuTimeComput / _nbMTU) * 1000).arg((_totalTimeComput / _nbEMG) * 1000).arg(_nbMTU).arg(_nbNMS).arg(_nbEMG).arg(timeComptGUI_ * 1000).arg(1 / (emgTotalPeriod_ / nbofEMG_)).arg(1 / (posTotalPeriod_ / nbofPos_)).arg(1 / (lmtTotalPeriod_ / nbofLMT_)).arg(1 / (torqueTotalPeriod_ / nbofTorque_));
	_label->setText(text);
}

void MainWindow::updatePlot()
{
	unsigned int cpt = 0;
	float xWindows = 5;

	if (addMuscleForceBar_)
	{
		if (_muscleForceGui.size() != 0)
		{
			if (_checkBoxBool[cpt])
			{
				QVector<double> barData;
				QVector<double> ticks;

				for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
				{
					const int& i = std::distance < std::vector<std::string>::const_iterator >(_musclesNames.begin(), it);

					//std::set<std::string>::const_iterator itFind = _musclesNames.find(*it);

					//if (itFind != _musclesNames.end())
					//{
						barData << _muscleForceGui.at(i);
						//const int& j = std::distance<std::set<std::string>::const_iterator>(_musclesNames.begin(), itFind);
						//ticks << j;
						ticks << i;
					//}
				}

				_bar->setData(ticks, barData);

				_bar->rescaleValueAxis(true);

				if (!_VectDock.at(cpt)->isHidden())
					_VectPlot.at(cpt)->replot();
			}
		}

		cpt = 1;
	}

	//std::cout << "ok" << std::endl;
	if (AddEMG_)
	{
		//std::cout << "ok1: " << _emgGui.size() << " : " << _timeEmg.size() << std::endl;
		if (_emgGui.size() != 0 && _timeEmg.size() != 0)
		{
			//std::cout << "ok2" << std::endl;
			QVector<QVector<double> > QEmg;
			QEmg.resize(_musclesNames.size());
			QVector<double> Qtime;

			for (std::vector<std::vector<double> >::const_iterator it = _emgGui.begin(); it < _emgGui.end(); it++)
			{
				for (std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++)
				{
					const int& i = std::distance<std::vector<double>::const_iterator>(it->begin(), it2);
					QEmg[i].push_back(*it2);
					//QEmg[i].push_back(0.5);
				}
			}
			
			if (_firstPassTimeEmg)
			{
				_firstPassTimeEmg = false;
				_timeInitEmg = _timeEmg[0];
			}

			for (std::vector<double>::const_iterator itVect = _timeEmg.begin(); itVect < _timeEmg.end(); itVect++)
				Qtime.push_back(*itVect - _timeInitEmg);

			for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
			{
				//std::cout << *it <<": " << QEmg.at ( std::distance < std::vector<std::string>::const_iterator > ( _musclesNames.begin(), it ) ).back() <<"\t";
				//if (_muscleNameOnChannel.find(*it) != _muscleNameOnChannel.end())
				//{
					if (_checkBoxBool[cpt])
					{
						const int& i = std::distance < std::vector<std::string>::const_iterator >(_musclesNames.begin(), it);
/*						//COUT << QEmg.at(0).at(0) << std::endl << std::flush;
#ifdef WIN32 //On windows addData does not work
						_VectPlot.at(cpt)->graph(0)->setData(Qtime, QEmg.at(i));
#else*/
						_VectPlot.at(cpt)->graph(0)->addData(Qtime, QEmg.at(i));
						_VectPlot.at(cpt)->graph(0)->removeDataBefore(Qtime.back() - xWindows);

						//std::cout << "EMG" << Qtime.size() << std::endl;
//#endif
						_VectPlot.at(cpt)->xAxis->setRange(Qtime.back() + 0.25, xWindows + 0.5, Qt::AlignRight);
						_VectPlot.at(cpt)->yAxis->setRange(1, 0);
						if (!_VectDock.at(cpt)->isHidden())
							_VectPlot.at(cpt)->replot();
					}

					cpt++;
				//}
			}
			//std::cout << std::endl;
		}

		cpt = _musclesNames.size() + 1;
	}

	if (addTorqueID_ || addTorqueCEINMS_ || addTorqueMotor_)
	{
		if (_timeTorque.size() != 0)
		{
			
			QVector<QVector<double> > Qtorque;
			Qtorque.resize(_dofNames.size());
			QVector<QVector<double> > QIDtorque;
			QIDtorque.resize(_dofNames.size());
			QVector<QVector<double> > QtorqueMotor;
			QIDtorque.resize(_dofNames.size());
			QVector<double> Qtime;

			if (_torqueGui.size() != 0)
			{
				
				for (std::vector<std::vector<double> >::const_iterator it = _torqueGui.begin(); it < _torqueGui.end(); it++)
				{
					for (std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++)
					{
						const int& i = std::distance<std::vector<double>::const_iterator>(it->begin(), it2);
						Qtorque[i].push_back(*it2);
					}
				}
			}

			if (_idGui.size() != 0)
			{
				
				for (std::vector<std::vector<double> >::const_iterator it = _idGui.begin(); it < _idGui.end(); it++)
				{
					for (std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++)
					{
						const int& i = std::distance<std::vector<double>::const_iterator>(it->begin(), it2);
						QIDtorque[i].push_back(*it2);
					}
				}
			}

			if (_torqueMotor.size() != 0)
			{
				for (std::vector<std::vector<double> >::const_iterator it = _torqueMotor.begin(); it < _torqueMotor.end(); it++)
				{
					for (std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++)
					{
						const int& i = std::distance<std::vector<double>::const_iterator>(it->begin(), it2);
						QtorqueMotor[i].push_back(*it2);
					}
				}
			}

			if (_firstPassTimeTorque)
			{
				_firstPassTimeTorque = false;
				_timeInitTorque = _timeTorque[0];
			}

			for (std::vector<double>::const_iterator itVect = _timeTorque.begin(); itVect < _timeTorque.end(); itVect++)
				Qtime.push_back(*itVect - _timeInitTorque);

			for (std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++)
			{
				if (_checkBoxBool[cpt])
				{
					const int& i = std::distance < std::vector<std::string>::const_iterator >(_dofNames.begin(), it);

					if (_torqueGui.size() != 0)
					{

						//COUT << _torqueGui.size() << std::endl << std::flush;
/*#ifdef WIN32
						_VectPlot.at(cpt)->graph(0)->setData(Qtime, Qtorque.at(i));
#else*/
						//std::cout << "Torque" << Qtime.size() << std::endl;
						_VectPlot.at(cpt)->graph(0)->addData(Qtime, Qtorque.at(i));
						_VectPlot.at(cpt)->graph(0)->removeDataBefore(Qtime.back() - xWindows);
//#endif

						if (_idGui.size() != 0)
						{
							_VectPlot.at(cpt)->graph(0)->rescaleAxes();
/*#ifdef WIN32
							_VectPlot.at(cpt)->graph(1)->setData(Qtime, QIDtorque.at(i));
#else*/
							_VectPlot.at(cpt)->graph(1)->addData(Qtime, QIDtorque.at(i));
							_VectPlot.at(cpt)->graph(1)->removeDataBefore(Qtime.back() - xWindows);
//#endif
							//_VectPlot.at(cpt)->graph(1)->rescaleAxes(true);
						}

						if (_torqueMotor.size() != 0)
						{
							_VectPlot.at(cpt)->graph(0)->rescaleAxes();
/*#ifdef WIN32
							_VectPlot.at(cpt)->graph(2)->setData(Qtime, QtorqueMotor.at(i));
#else*/
							_VectPlot.at(cpt)->graph(2)->addData(Qtime, QtorqueMotor.at(i));
							_VectPlot.at(cpt)->graph(2)->removeDataBefore(Qtime.back() - xWindows);
//#endif
							//_VectPlot.at(cpt)->graph(2)->rescaleAxes(true);
						}
					}
				/*	else if (_idGui.size() != 0)
					{

						_VectPlot.at(cpt)->graph(0)->addData(Qtime, QIDtorque.at(i));
						_VectPlot.at(cpt)->graph(0)->removeDataBefore(Qtime.back() - xWindows);

						if (_torqueMotor.size() != 0)
						{
							_VectPlot.at(cpt)->graph(0)->rescaleAxes();

							_VectPlot.at(cpt)->graph(1)->addData(Qtime, QtorqueMotor.at(i));
							_VectPlot.at(cpt)->graph(1)->removeDataBefore(Qtime.back() - xWindows);
						}
					}*/

					_VectPlot.at(cpt)->xAxis->setRange(Qtime.back() + 0.25, xWindows + 0.5, Qt::AlignRight);
					if (!_VectDock.at(cpt)->isHidden())
					{
						_VectPlot.at(cpt)->rescaleAxes(true);
						_VectPlot.at(cpt)->replot();
					}
				}

				cpt++;
			}
		}
		cpt = _musclesNames.size() + _dofNames.size() + 1;
	}
	if (addDemo_)
	{
		if (_emgGui.size() != 0 && _timeEmg.size() != 0)
		{

			QVector<double> barData;
			QVector<double> ticks;

			for (std::vector<double>::const_iterator itVect = _timeEmg.begin(); itVect < _timeEmg.end(); itVect++)
			{
				cpt = _musclesNames.size() + _dofNames.size() + 1;
				const int& cptTime = std::distance<std::vector<double>::const_iterator>(_timeEmg.begin(), itVect);
				if (timeDemo_.size()> 0 && * itVect - timeDemo_[0] >= 10)
				{
					for (std::vector<double>::const_iterator it = _emgGui[cptTime].begin(); it < _emgGui[cptTime].end(); it++)
					{
						const int& cptEMG = std::distance<std::vector<double>::const_iterator>(_emgGui[cptTime].begin(), it);
						EmgDemo_[cptEMG].push_back(*it);
						EmgDemo_[cptEMG].pop_front();
						//_VectPlot.at(cpt)->graph(0)->addData(*itVect - timeDemo_[0], *it);
						//_VectPlot.at(cpt)->graph(0)->removeDataBefore(0);
						cpt++;
					}
					timeDemo_.push_back(*itVect);
					timeDemo_.pop_front();
				}
				else
				{
					timeDemo_.push_back(*itVect);
					nbSampleDemo_++;
					for (std::vector<double>::const_iterator it = _emgGui[cptTime].begin(); it < _emgGui[cptTime].end(); it++)
					{
						const int& cptEMG = std::distance<std::vector<double>::const_iterator>(_emgGui[cptTime].begin(), it);
						EmgDemo_[cptEMG].push_back(*it);
						//_VectPlot.at(cpt)->graph(0)->addData(*itVect - timeDemo_[0], *it);
						cpt++;
					}
				}
			}
			cpt = _musclesNames.size() + _dofNames.size() + 1;
			for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
			{
				
				const int& cptM = std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), it);

				barDataDemo_[cptM][4] = std::accumulate(EmgDemo_[cptM].begin(), EmgDemo_[cptM].end(), 0.0) / EmgDemo_[cptM].size();
				barData.clear();
				ticks.clear();
				barData << barDataDemo_[cptM][0];
				ticks << 1;
				demoBar_[0][cptM]->setData(ticks, barData);
				barData.clear();
				ticks.clear();
				barData << barDataDemo_[cptM][1];
				ticks << 2;
				demoBar_[1][cptM]->setData(ticks, barData);
				barData.clear();
				ticks.clear();
				barData << barDataDemo_[cptM][2];
				ticks << 3;
				demoBar_[2][cptM]->setData(ticks, barData);
				barData.clear();
				ticks.clear();
				barData << barDataDemo_[cptM][3];
				ticks << 4;
				demoBar_[3][cptM]->setData(ticks, barData);
				barData.clear();
				ticks.clear();
				barData << barDataDemo_[cptM][4];
				ticks << 5;
				demoBar_[4][cptM]->setData(ticks, barData);
				if (!_VectDock.at(cpt)->isHidden())
				{
					_VectPlot.at(cpt)->rescaleAxes(true);
					_VectPlot.at(cpt)->xAxis->setRange(0, 6);	
					
					QVector<double> temp = timeDemo_;
					double initValTime = timeDemo_[0];
					for (QVector<double>::iterator it = temp.begin(); it != temp.end(); it++)
						*it -= initValTime;
					_VectPlot.at(cpt)->graph(0)->setData(temp, EmgDemo_[cptM]);
					_VectPlot.at(cpt)->xAxis2->setRange(0, 10);
					_VectPlot.at(cpt)->yAxis2->setRange(0, 1.1);
					_VectPlot.at(cpt)->replot();
				}
				cpt++;
			}
			barData.clear();
			std::vector<std::string>::const_iterator itSol = std::find(_musclesNames.begin(), _musclesNames.end(), "soleus_l");
			std::vector<std::string>::const_iterator itMed = std::find(_musclesNames.begin(), _musclesNames.end(), "med_gas_l");
			std::vector<std::string>::const_iterator itLat = std::find(_musclesNames.begin(), _musclesNames.end(), "lat_gas_l");
			if (itSol != _musclesNames.end() && itMed != _musclesNames.end() && itLat != _musclesNames.end())
				barDataDemo_[_musclesNames.size()][4] =
				(barDataDemo_[std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), itSol)][4]
				+ barDataDemo_[std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), itMed)][4]
				+ barDataDemo_[std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), itLat)][4]) / 3;
			else
				barDataDemo_[_musclesNames.size()][4] = 0;

			int cptM = _musclesNames.size();
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][0];
			ticks << 1;
			demoBar_[0][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][1];
			ticks << 2;
			demoBar_[1][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][2];
			ticks << 3;
			demoBar_[2][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][3];
			ticks << 4;
			demoBar_[3][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][4];
			ticks << 5;
			demoBar_[4][cptM]->setData(ticks, barData);
			if (!_VectDock.at(cpt)->isHidden())
			{
				_VectPlot.at(cpt)->rescaleAxes(true);
				_VectPlot.at(cpt)->xAxis->setRange(0, 6);
				_VectPlot.at(cpt)->xAxis2->setRange(0, 10);
				_VectPlot.at(cpt)->yAxis2->setRange(0, 1.1);
				_VectPlot.at(cpt)->replot();
			}
			cpt++;

			
			barData.clear();
			std::vector<std::string>::const_iterator itTib = std::find(_musclesNames.begin(), _musclesNames.end(), "tib_ant_l");
			if (itTib != _musclesNames.end())
				barDataDemo_[_musclesNames.size() + 1][4] =
				(barDataDemo_[std::distance<std::vector<std::string>::const_iterator>(_musclesNames.begin(), itTib)][4]);
			else
				barDataDemo_[_musclesNames.size() + 1][4] = 0;

			cptM = _musclesNames.size() + 1;
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][0];
			ticks << 1;
			demoBar_[0][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][1];
			ticks << 2;
			demoBar_[1][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][2];
			ticks << 3;
			demoBar_[2][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][3];
			ticks << 4;
			demoBar_[3][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][4];
			ticks << 5;
			demoBar_[4][cptM]->setData(ticks, barData);
			if (!_VectDock.at(cpt)->isHidden())
			{
				_VectPlot.at(cpt)->rescaleAxes(true);
				_VectPlot.at(cpt)->xAxis->setRange(0, 6);
				_VectPlot.at(cpt)->xAxis2->setRange(0, 10);
				_VectPlot.at(cpt)->yAxis2->setRange(0, 1.1);
				_VectPlot.at(cpt)->replot();
			}
			cpt++;

			
			barData.clear();
			//std::vector<std::string>::iterator itTot = barDataDemo_.begin() + _musclesNames.size();
			barDataDemo_[_musclesNames.size() + 2][4] = 0;
			
			for (std::vector<std::vector<double> >::const_iterator it = barDataDemo_.begin(); it != barDataDemo_.end() - 3; it++)
			{
				barDataDemo_[_musclesNames.size() + 2][4] += (*it)[4];
				//std::cout << (*it)[4] << std::endl;
			}

			cptM = _musclesNames.size() + 2;
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][0];
			ticks << 1;
			demoBar_[0][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][1];
			ticks << 2;
			demoBar_[1][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][2];
			ticks << 3;
			demoBar_[2][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][3];
			ticks << 4;
			demoBar_[3][cptM]->setData(ticks, barData);
			barData.clear();
			ticks.clear();
			barData << barDataDemo_[cptM][4];
			ticks << 5;
			demoBar_[4][cptM]->setData(ticks, barData);
			if (!_VectDock.at(cpt)->isHidden())
			{
				_VectPlot.at(cpt)->rescaleAxes(true);
				_VectPlot.at(cpt)->xAxis->setRange(0, 6);
				_VectPlot.at(cpt)->xAxis2->setRange(0, 10);
				_VectPlot.at(cpt)->yAxis2->setRange(0, 1.1);
				_VectPlot.at(cpt)->replot();
			}
			cpt++;
		}

		cpt = 2 * _musclesNames.size() + _dofNames.size() + 1 + 3; // plantar, dorsi and total
	}
	//WORKLOOP
	if (WLFiberLength_.size() != 0 && WLForce_.size() != 0 && timeWorkLoop_.size() != 0)
	{
		for (std::vector<double>::const_iterator itVect = timeWorkLoop_.begin(); itVect < timeWorkLoop_.end(); itVect++)
		{
			const int& cptTime = std::distance<std::vector<double>::const_iterator>(timeWorkLoop_.begin(), itVect);
			if (timeDemoWL_.size()> 0 && * itVect - timeDemoWL_[0] >= 5)
			{
				for (std::vector<double>::const_iterator it = WLForce_[cptTime].begin(); it < WLForce_[cptTime].end(); it++)
				{
					const int& cptEMG = std::distance<std::vector<double>::const_iterator>(WLForce_[cptTime].begin(), it);
					forceDemo_[cptEMG].push_back(*it / isometricsMaxForce_[cptEMG]);
					forceDemo_[cptEMG].pop_front();
					fiberLengthDemo_[cptEMG].push_back(WLFiberLength_[cptTime][cptEMG] / optimalFiberLength_[cptEMG]);
					fiberLengthDemo_[cptEMG].pop_front();
					demoWLCurve_[4][cptEMG]->addData(*itVect, WLFiberLength_[cptTime][cptEMG] / optimalFiberLength_[cptEMG], *it / isometricsMaxForce_[cptEMG]);
					demoWLCurve_[4][cptEMG]->removeDataBefore(timeDemoWL_[0]);
				}
				timeDemoWL_.push_back(*itVect);
				timeDemoWL_.pop_front();
			}
			else
			{
				timeDemoWL_.push_back(*itVect);
				for (std::vector<double>::const_iterator it = WLForce_[cptTime].begin(); it < WLForce_[cptTime].end(); it++)
				{
					const int& cptEMG = std::distance<std::vector<double>::const_iterator>(WLForce_[cptTime].begin(), it);
					forceDemo_[cptEMG].push_back(*it / isometricsMaxForce_[cptEMG]);
					fiberLengthDemo_[cptEMG].push_back(WLFiberLength_[cptTime][cptEMG] / optimalFiberLength_[cptEMG]);
					demoWLCurve_[4][cptEMG]->addData(*itVect, WLFiberLength_[cptTime][cptEMG] / optimalFiberLength_[cptEMG], *it / isometricsMaxForce_[cptEMG]);
				}
			}
		}
		for (std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++)
		{
			if (_checkBoxBool[cpt])
			{
				const int& i = std::distance < std::vector<std::string>::const_iterator >(_musclesNames.begin(), it);
				_VectPlot.at(cpt)->rescaleAxes(true);
				_VectPlot.at(cpt)->replot();
			}

			cpt++;
		}
		cpt = 3 * _musclesNames.size() + _dofNames.size() + 1 + 3; // plantar, dorsi and total
	}
}

void MainWindow::thread()
{
	bool end = InterThread::getEndThread();

	if (end)
	{
		stop();
	}

	if (firstPass_)
	{
		InterThread::readyToStart->wait();
		this->show();
		firstPass_ = false;
	}

	
	timePrevious_ = rtb::getTime();
	//int timeForModulo = (timePrevious_ - timeInit_) * 100; // to have int precision to 100ms

	//std::cout << timeForModulo << " : " << timePrevious_ << std::endl;
	//if (timeForModulo % 2) // every 200ms we update the value of the graph with the whole dataset
	//{
		InterThread::getGuiEMG(_emgGui, _timeEmg);
		//std::cout << "Size EMG: " << _emgGui.size() <<  " : " << _timeEmg.size() << std::endl;

		/*for (int i = _emgGui.size() - 1; i != 0; i--)
		{
			if (i % 3 == 0)
			{
				_emgGui.erase(_emgGui.begin() + i);
				_timeEmg.erase(_timeEmg.begin() + i);
			}
		}*/
		_torqueGui = InterThread::getGuiTorque();
		_torqueMotor = InterThread::getGuiMultTorqueMotor();
		_idGui = InterThread::getGuiID();
		InterThread::getGuiWorkLoop(WLForce_, WLFiberLength_, timeWorkLoop_);
	//}

	std::vector < double > temp;
	temp = InterThread::getGuiTimeLMT();
	_timeLMT.insert(_timeLMT.end(),temp.begin(),temp.end());
	temp = InterThread::getGuiTimePos();
	_timePos.insert(_timePos.end(), temp.begin(), temp.end());
	_muscleForceGui = InterThread::getGuiMuscleForce();
	_timeTorque = InterThread::getGuiTimeTorque();
	_nmsTimeComput = InterThread::getTimeConsumeNMS();
	_nbNMS = InterThread::getNbFrameNMS();
	_totalTimeComput = InterThread::getTimeEndNMS();
	_mtuTimeComput = InterThread::getTimeConsumeMTU();
	_nbMTU = InterThread::getNbFrameMTU();
	_nbEMG = InterThread::getNbFrameEMG();

	_positionMap = InterThread::getPositionMap();

	checkTreeBox();
	checkSaveonPDF();
	updatePlot();

	updateTiming(); // need to be last because clear timeEMG and time LMT (create issue with plotting)


#ifdef USE_OPENSIM

	if (add3DIK_)
	{
		_openglWin->setPositionfromMap1(_positionMap);
		_openglWin->updateGL();
	}

#endif
	timeComptGUI_ = rtb::getTime() - timePrevious_;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	InterThread::setEndThread(true);
}

void MainWindow::stop()
{
	//	this->close();
	//	_dataTimer.stop();
}