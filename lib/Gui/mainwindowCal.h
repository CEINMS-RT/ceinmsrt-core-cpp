/*
 * Gui.h
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#ifndef GUICAL_H_
#define GUICAL_H_

#include <CommonCEINMS.h>
#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h"
#include "SyncToolsCal.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <ostream>
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
#include <QListWidget>
#include <QTreeWidget>
#include <cmath>
#include <QVector>
#include <QString>
#include <QSplitter>
#include <QPushButton>
//#include "SimulatedAnnealing.h"
#include "ExecutionSimulatedAnnealing.h"

namespace Ui {
class MainWindow;
}

class MainWindow: public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(const ExecutionSimulatedAnnealing* executionSimulatedAnnealing, QWidget *parent = 0);
	virtual ~MainWindow();
	void stop();
	void stopRefresh();
	inline void setSimAnnExec(const ExecutionSimulatedAnnealing* executionSimulatedAnnealing)
	{
		executionSimulatedAnnealing_ = executionSimulatedAnnealing;
	}
private:
	std::vector<QCustomPlot*> _VectPlot;
	std::vector<QDockWidget*> _VectDock;
	std::vector<QTreeWidgetItem*> _VectItem;
	std::vector<QTreeWidgetItem*> _VectTreeItem;
	std::vector<QPushButton*> _VectButtonItem;
	std::vector<QCPPlotTitle*> title_;
	Ui::MainWindow* _ui;
	QWidget* _main;
	//QHBoxLayout* _mainHLayout;
	QVBoxLayout* _rightVLayout;
	QVBoxLayout* _layout;
	QScrollArea* _scrollArea;
	QScrollArea* _scrollAreaCheck;
	QScrollArea* _scrollAreaParam;
	QTreeWidget* _treeViewRight;
	QTreeWidget* _treeView;
	QWidget* _viewport;
	QWidget* _rightVLayoutWidget;
	QWidget* _leftVLayoutWidget;
	QSplitter* _splitter;
	std::vector<std::string> _dofNames;
	std::vector<std::string> _musclesNames;
	std::vector<std::string> _trialNames;
	std::vector < double > _vecParamBase;
	std::vector < double > _vecParamCal;
	std::vector < double > _vecParamLB;
	std::vector < double > _vecParamUB;
	std::vector<unsigned int> _musclesIndexToCalibrate;
	std::vector<unsigned int> _dofIndexToCalibrate;
	std::vector< std::vector < std::vector < double > > > _torqueBase;
	std::vector< std::vector < double > > _timeIKBase;
	std::vector< std::vector < double > > _timeIDBase;
	std::vector< std::vector < std::vector < double > > > _torqueCal;
	const ExecutionSimulatedAnnealing* executionSimulatedAnnealing_;
	std::vector<bool> _checkBoxBool;
	unsigned int _nbOfPlot;
	unsigned int _strengthCoeff;
	unsigned _mpiIteration;
	QTimer _dataTimer;
	void closeEvent(QCloseEvent *event);
private slots:
	void _thread();
};

#endif /* GUI_H_ */
