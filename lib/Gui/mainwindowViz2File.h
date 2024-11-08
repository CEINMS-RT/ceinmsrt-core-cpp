/*
 * Gui.h
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

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

namespace Ui
{
	class MainWindow;
}

class MainWindow: public QMainWindow
{
		Q_OBJECT
	public:
		MainWindow ( const std::vector<string>& plotName, const std::vector<string>& collName, const std::vector<std::vector<double> >& data, const std::vector<double>& time, const std::vector<std::vector<double> >& data2, const std::vector<double>& time2, QWidget* parent = 0 );
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
