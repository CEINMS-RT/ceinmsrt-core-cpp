/*
 * Gui.h
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#ifndef GUI_H_
#define GUI_H_

#include "myGLWidget.h" // always on top when using windows
#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h"
#include "SyncTools.h"
#include "ui_mainwindow.h"
#include <sstream>
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QMainWindow>
#include <QDockWidget>
#include <QScrollArea>
#include <QBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QSize>
#include <QCheckBox>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QLabel>
#include <QAction>
#include <QMenu>
#include <cmath>
#include <QListWidget>
#include <QTreeWidget>
#include <QVector>
#include <QString>
#include <QSplitter>
#include <QPushButton>
#include <set>
#include <boost/unordered_map.hpp>
#include <deque>
#include <algorithm>
#include <getTime.h>

#define SIZEFONT 12
#define SIZEFONTLABEL 17
#define SIZEFONTTITLE 19

#ifdef USE_OPENSIM
using namespace gl;
#endif

namespace Ui
{
	class MainWindow;
}

class MainWindow: public QMainWindow
{
		Q_OBJECT
	public:
		MainWindow ( QWidget* parent = 0 );
		virtual ~MainWindow();
		void stop();

		void addEMG();
		void addTorqueCEINMS();
		void addTorqueID();
		void addTorqueMotor();
		void addMuscleForceBar();
		void add3DIK();
		void addDemo();
		void addTimingPanel();
		void addCalibrationPanel();
		void start();
		void close();   // add a public function to close, so that it can be load by external code

	protected slots:
		void viewPlotPanel();
		void viewTreePlotPanel();
		void viewOpenGLPanel();
		void viewTimingPanel();
		void viewCalibrationPanel();
		void exit();
		void about();
		void viewplot ( int plotNumber );
		void thread();
		void savePressed();
		void resetPressed();

	protected:

		typedef std::map<std::string, std::vector <std::string> > MapStrVectStr;
		typedef boost::unordered_map<std::string, std::vector <std::string> > UnMapStrVectStr;

		void constructTreePlotPanel ( const QSize& sizeOfThePanel );
		void constructTimingPanel ( const QSize& sizeOfThePanel );
		void constructOpenGLPanel ( const QSize& sizeOfThePanel, const std::string& modelName );
		void constructPlotPanel ( const QSize& sizeOfThePanel );
		void constructCalibrationPanel ( const QSize& sizeOfThePanel );
		void constructBar();
		void constructEMGPlot();
		void constructTorquePlot();
		void constructDemoBar();
		void constructWorkLoopPlot();
		void createMenus();
		void createActions();
		void closeEvent ( QCloseEvent* event );
		void checkTreeBox();
		void checkSaveonPDF();
		void updateTiming();
		void updatePlot();

		bool firstPass_;
		bool _firstPassTimeTorque;
		bool _firstPassTimeEmg;
		bool treePlotPanelBool_;
		bool plotPanelBool_;
		bool timingPanelBool_;
		bool openGLPanelBool_;
		bool addTorqueCEINMS_;
		bool addTorqueID_;
		bool addTorqueMotor_;
		bool addMuscleForceBar_;
		bool add3DIK_;
		bool AddEMG_;
		bool addDemo_;
		bool addTimingPanel_;
		bool addCalibrationPanel_;

		std::string modelName_;

		unsigned int _nbOfPlot;
		unsigned int _cptWin;
		unsigned int _nbNMS;
		unsigned int _nbMTU;
		unsigned int _nbEMG;
		unsigned int _strengthCoeff;
		unsigned int _mpiIteration;
		unsigned int nbSampleDemo_;

		int _verbose;

		long int nbofEMG_;
		long int nbofLMT_;
		long int nbofTorque_;
		long int nbofPos_;

		double _timeInitTorque;
		double _timeInitEmg;
		double _maxForce;
		double _minForce;
		double _nmsTimeCompu;
		double _mtuTimeComput;
		double _totalTimeComput;
		double _nmsTimeComput;
		double timePrevious_;
		double timeInit_;
		double timeComptGUI_;
		double deskHeight_;
		double emgTotalPeriod_;
		double lmtTotalPeriod_;
		double torqueTotalPeriod_;
		double posTotalPeriod_;
		

		std::vector < double > _timeEmg;
		std::vector < double > _timeLMT;
		std::vector < double > _timePos;
		std::vector < double >  _timeTorque;
		std::vector < double > _muscleForceGui;
		std::vector < double > _vecParamBase;
		std::vector < double > _vecParamCal;
		std::vector < double > _vecParamLB;
		std::vector < double > _vecParamUB;
		std::vector < double > timeWorkLoop_;
		std::vector < double > isometricsMaxForce_;
		std::vector < double > optimalFiberLength_;

		std::vector<unsigned int> _musclesIndexToCalibrate;
		std::vector<unsigned int> _dofIndexToCalibrate;

		std::vector< std::vector < double > > _emgGui;
		std::vector< std::vector < double > >  _torqueGui;
		std::vector< std::vector < double > >  _torqueMotor;
		std::vector< std::vector < double > > _idGui;
		std::vector< std::vector < double > > _timeIKBase;
		std::vector< std::vector < double > > _timeIDBase;
		std::vector< std::vector < double > > barDataDemo_;
		std::vector< std::vector < double > > WLDataDemo_;
		std::vector< std::vector < double > > WLForce_;
		std::vector< std::vector < double > > WLFiberLength_;
		
		std::vector< std::vector < std::vector < double > > > _torqueCal;
		std::vector< std::vector < std::vector < double > > > _torqueBase;

		std::vector<std::string> _dofNames;
		std::vector<std::string> _musclesNames;
		std::vector<std::string> _trialNames;

		std::vector<bool> _checkBoxBool;
		std::map<std::string, double> _positionMap;
		std::set<std::string> _muscleNameOnChannel;

		std::vector<QCustomPlot*> _VectPlot;
		std::vector<QDockWidget*> _VectDock;
		std::vector<QCheckBox*> _VectCheckBox;
		std::vector<QTreeWidgetItem*> _VectTreeItem;
		std::vector<QPushButton*> _VectButtonItem;
		std::vector<QAction*> plotAct_;
		std::vector<QCPPlotTitle*> title_;
		std::vector<std::vector<QCPBars*> > demoBar_;
		std::vector<std::vector<QCPCurve*> > demoWLCurve_;
		std::vector<QString> labelDemo_;
		std::vector<QPen> WLColor_;
		std::vector<QPen> barColor_;

		std::vector<QVector<double> > EmgDemo_;
		std::vector<QVector<double> > forceDemo_;
		std::vector<QVector<double> > fiberLengthDemo_;
		std::vector<QVector<double> > timeDemoWLSave_;
		std::vector<QVector<double> > timeDemoSave_;
		std::vector<std::vector<QVector<double> > > forceDemoSave_;
		std::vector<std::vector<QVector<double> > > EMGDemoSave_;
		std::vector<std::vector<QVector<double> > > fiberLengthDemoSave_;
		QVector<double> timeDemoWL_;
		QVector<double> timeDemo_;

		Ui::MainWindow* _ui;

		QWidget* _main;
		QWidget* _viewport;
		QWidget* _rightVLayoutWidget;
		QWidget* _leftVLayoutWidget;

		QVBoxLayout* _rightVLayout;
		QVBoxLayout* _leftVLayout;
		QVBoxLayout* _layout;

		QScrollArea* _scrollArea;
		QScrollArea* _scrollAreaCheck;

		QCPBars* _bar;
		QLabel* _label;
		QSplitter* _splitter;
		QTimer _dataTimer;
		QSignalMapper* signalMapperPlot_;
		QTreeWidget* _treeViewRight;
		QLineEdit* lineEditDemo_;

#ifdef USE_OPENSIM
		GLWidget* _openglWin;
#endif

		QMenu* fileMenu_;
		QMenu* helpMenu_;
		QMenu* windowsMenu_;
		QMenu* plotMenu_;

		QAction* aboutAct_;
		QAction* openAct_;
		QAction* exitAct_;
		QAction* plotPanel_;
		QAction* treePlotPanel_;
		QAction* openGLPanel_;
		QAction* timingPanel_;
		QAction* calibrationPanel_;

};

#endif /* GUI_H_ */
