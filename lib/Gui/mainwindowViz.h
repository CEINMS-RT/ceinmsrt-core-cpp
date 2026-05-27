// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#ifndef GUI_H_
#define GUI_H_

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
#include <cmath>
#include <QListWidget>
#include <QTreeWidget>
#include <QVector>
#include <QString>
#include <QSplitter>
#include <QPushButton>
#include <QCustomPlotMI.h>

namespace Ui
{
	class MainWindow;
}

class MainWindow: public QMainWindow
{
		Q_OBJECT
	public:
		MainWindow ( const std::vector<string>& collName, const std::vector<std::vector<double> >& data, const std::vector<double>& time, QWidget* parent = 0 );
		virtual ~MainWindow();
		void stop();
	protected:

		Ui::MainWindow* _ui;

		QScrollArea* _scrollArea;
		QWidget* _viewport;
		QVBoxLayout* _layout;

		std::vector<QCustomPlot*> _VectPlot;
		std::vector<QDockWidget*> _VectDock;
		std::vector<QCPPlotTitle*> title_;

		void closeEvent ( QCloseEvent* event );

};

#endif /* GUI_H_ */
