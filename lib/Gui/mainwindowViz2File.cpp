/*
 * Gui.cpp
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#include "mainwindowViz2File.h"

MainWindow::MainWindow ( const std::vector<string>& plotName, const std::vector<string>& collName, const std::vector<std::vector<double> >& data, const std::vector<double>& time, const std::vector<std::vector<double> >& data2, const std::vector<double>& time2, QWidget* parent ) :
	QMainWindow ( parent ), _ui ( new Ui::MainWindow )
{
	
	QRect rec = QApplication::desktop()->screenGeometry();

	resize ( rec.width(), rec.height() );
	
	QSize sizeWin = size();
	
	_scrollArea = new QScrollArea ( this );
	_scrollArea->setWidgetResizable ( true );
	_viewport = new QWidget ( _scrollArea );
	_layout = new QVBoxLayout ( _viewport );
	_viewport->setMinimumHeight ( collName.size() * 300 );
	_scrollArea->setWidget ( _viewport );
	_scrollArea->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOn );
	
	_scrollArea->resize ( sizeWin.width() - 70, sizeWin.height() - 30 );

	QVector<double> qTime;
	for ( std::vector<double>::const_iterator it2 = time.begin(); it2 < time.end(); it2++ )
	{
		qTime.push_back(*it2);
	}
	
	QVector<double> qTime2;
	for ( std::vector<double>::const_iterator it2 = time2.begin(); it2 < time2.end(); it2++ )
	{
		qTime2.push_back(*it2);
	}

	for ( std::vector<std::string>::const_iterator it = collName.begin();
			it != collName.end(); it++ )
	{
		const int& cpt = std::distance<std::vector<std::string>::const_iterator> ( collName.begin(), it );
		_VectDock.push_back ( new QDockWidget ( this ) );
		_VectPlot.push_back ( new QCustomPlot ( _VectDock.back() ) );
		title_.push_back ( new QCPPlotTitle ( _VectPlot.back() ) );
		_layout->addWidget ( _VectDock.back() );
		std::stringstream torqueText;
		torqueText << "Torque " + *it;
		title_.back()->setText ( torqueText.str().c_str() );
		title_.back()->setFont ( QFont ( "sans", 12, QFont::Bold ) );
		title_.back()->setTextColor ( Qt::darkGreen );
		_VectPlot.back()->plotLayout()->insertRow ( 0 );
		_VectPlot.back()->plotLayout()->addElement ( 0, 0, title_.back() );
		_VectPlot.back()->addGraph(); // blue line
		_VectPlot.back()->addGraph(); // red line
		
		_VectPlot.back()->legend->setVisible(true);
		
		QPen bluePen;
		bluePen.setColor ( Qt::blue );
		bluePen.setWidthF ( 3 );
		_VectPlot.back()->graph ( 0 )->setPen ( bluePen );
		_VectPlot.back()->graph ( 0 )->setAntialiasedFill ( false );
		_VectPlot.back()->graph ( 0 )->setName(QString(plotName.at(0).c_str()));
		
		QPen redPen;
		bluePen.setColor ( Qt::red );
		bluePen.setWidthF ( 3 );
		_VectPlot.back()->graph ( 1 )->setPen ( bluePen );
		_VectPlot.back()->graph ( 1 )->setAntialiasedFill ( false );
		_VectPlot.back()->graph ( 1 )->setName(QString(plotName.at(1).c_str()));
		
		QVector<double> qData;
		QVector<double> qData2;
		
		for ( std::vector<double>::const_iterator it2 = data[cpt].begin(); it2 < data[cpt].end(); it2++ )
		{
			qData.push_back(*it2);
		}
		
		for ( std::vector<double>::const_iterator it2 = data2[cpt].begin(); it2 < data2[cpt].end(); it2++ )
		{
			qData2.push_back(*it2);
		}
		
		_VectPlot.back()->xAxis2->setVisible(true);
		_VectPlot.back()->xAxis2->setTickLabels(false);
		_VectPlot.back()->yAxis2->setVisible(true);
		_VectPlot.back()->yAxis2->setTickLabels(false);
		connect(_VectPlot.back()->xAxis, SIGNAL(rangeChanged(QCPRange)), _VectPlot.back()->xAxis2, SLOT(setRange(QCPRange)));
		_VectPlot.back()->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
		connect(_VectPlot.back()->yAxis, SIGNAL(rangeChanged(QCPRange)), _VectPlot.back()->yAxis2, SLOT(setRange(QCPRange)));
		
		_VectPlot.back()->graph(0)->setData(qTime, qData);
		
		_VectPlot.back()->graph(1)->setData(qTime2, qData2);
		
		_VectPlot.back()->graph(0)->rescaleAxes();
		
		_VectDock.back()->setWidget ( _VectPlot.back() );
	}
}

MainWindow::~MainWindow()
{
	delete _ui;
	std::cout << "\033[1;31mGUI " << this << "\033[0m" << std::endl;
}

void MainWindow::closeEvent ( QCloseEvent* event )
{
	InterThread::setEndThread(true);
}

void MainWindow::stop()
{
	this->close();
}

