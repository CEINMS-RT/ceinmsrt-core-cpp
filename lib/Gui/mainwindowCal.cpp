/*
 * Gui.cpp
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#include "mainwindowCal.h"

MainWindow::MainWindow (const  ExecutionSimulatedAnnealing* executionSimulatedAnnealing, QWidget* parent ) :
	QMainWindow ( parent ), _ui ( new Ui::MainWindow )
{
	QRect rec = QApplication::desktop()->screenGeometry();
	
	setSimAnnExec(executionSimulatedAnnealing);

	resize ( rec.width(), rec.height() );

	std::vector<std::string> muscleTemp;

	QIcon icon ( "CEINMS-RT_V1.png" );
//     icon.addFile(QString::fromUtf8("CEINMS-RT_V1.png"), QSize(), QIcon::Normal, QIcon::Off);
	this->setWindowIcon ( icon );

	SyncToolsCal::Shared::dofNamesSem.waitOnce();
	SyncToolsCal::Shared::musclesNamesSem.waitOnce();
	SyncToolsCal::Shared::trialNamesSem.waitOnce();
	SyncToolsCal::Shared::torqueBaseReady.waitOnce();
	SyncToolsCal::Shared::vecParamBaseReady.waitOnce();
	SyncToolsCal::Shared::vecParamLBReady.waitOnce();
	SyncToolsCal::Shared::vecParamUBReady.waitOnce();
	SyncToolsCal::Shared::strengthCoeffReady.waitOnce();
	SyncToolsCal::Shared::timeIKReady.waitOnce();
	SyncToolsCal::Shared::timeIDReady.waitOnce();
	SyncToolsCal::Shared::musclesIndexToCalibrateSem.waitOnce();
	SyncToolsCal::Shared::dofIndexToCalibrateSem.waitOnce();
	SyncToolsCal::Shared::maxEvalReady.waitOnce();
	SyncToolsCal::Shared::minConvReady.waitOnce();

	int maxEval = SyncToolsCal::Shared::maxEval;
	double minConv = SyncToolsCal::Shared::minConv;
	muscleTemp = SyncToolsCal::Shared::musclesNames;
	_dofNames = SyncToolsCal::Shared::dofNames;
	_trialNames = SyncToolsCal::Shared::trialNames;
	_torqueBase = SyncToolsCal::Shared::torqueBase;
	_vecParamBase = SyncToolsCal::Shared::vecParamBase;
	_vecParamLB = SyncToolsCal::Shared::vecParamLB;
	_vecParamUB = SyncToolsCal::Shared::vecParamUB;
	_strengthCoeff = SyncToolsCal::Shared::strengthCoeff;
	_timeIKBase = SyncToolsCal::Shared::timeIKBase;
	_timeIDBase = SyncToolsCal::Shared::timeIDBase;
	_musclesIndexToCalibrate = SyncToolsCal::Shared::musclesIndexToCalibrate;
	_dofIndexToCalibrate = SyncToolsCal::Shared::dofIndexToCalibrate;


//	std::cout << "_dofIndexToCalibrate: " << _dofIndexToCalibrate.size() << std::endl;
//	std::cout << "_dofNames: " << _dofNames.size() << std::endl;

	for ( vector<unsigned int>::const_iterator it = _musclesIndexToCalibrate.begin(); it < _musclesIndexToCalibrate.end(); it++ )
	{
		_musclesNames.push_back ( muscleTemp.at ( *it ) );
	}

//	std::cout << endl;

	SyncToolsCal::Shared::mpiIterationMutex.lock();
	SyncToolsCal::Shared::mpiIteration = 0;
	SyncToolsCal::Shared::mpiIterationMutex.unlock();
	_mpiIteration = 0;

	//_main = new QWidget(this);
	_splitter = new QSplitter ( this );
	//_main->setLayout(_splitter);
	//_main->setWidget(_splitter);

	QSize sizeWin = size();

	_scrollAreaCheck = new QScrollArea ( _splitter );
	_splitter->addWidget ( _scrollAreaCheck );
	_scrollAreaCheck->setWidgetResizable ( true );
//	_rightVLayoutWidget = new QWidget(_scrollAreaCheck);
	_treeViewRight = new QTreeWidget ( _scrollAreaCheck );
	_treeViewRight->setHeaderLabels (
	    QStringList() << "Name" << "save" );
	_treeViewRight->header()->close();
	_treeViewRight->header()->resizeSection ( 0, 180 );
	_treeViewRight->header()->resizeSection ( 1, 10 );
	_treeViewRight->header()->setStretchLastSection ( false );
	_scrollAreaCheck->setWidget ( _treeViewRight );
	//rightVLayoutWidget->setMaximumWidth(sizeWin.width() * 0.1);
	//_scrollAreaCheck->setMaximumWidth(sizeWin.width() * 0.1);
//	rightVLayoutWidget->setMinimumHeight((musclesNames.size() + dofNames.size() + 1) * 400);

	QSize sizeHlay = _scrollAreaCheck->size();

	_scrollAreaParam = new QScrollArea ( _splitter );
	//_leftVLayoutWidget = new QWidget(_scrollAreaParam);
	_treeView = new QTreeWidget ( _scrollAreaParam );
	_treeView->setHeaderLabels (
	    QStringList() << "Name" << "Parameters" << "Base Parameters"
	    << "Upper Bond" << "Lower Bond" );

//	_treeView->header()->close();
	_treeView->header()->resizeSection ( 0, 180 );
	_treeView->header()->resizeSection ( 1, 100 );
	_treeView->header()->resizeSection ( 2, 100 );
	_treeView->header()->resizeSection ( 3, 100 );
	_treeView->header()->resizeSection ( 4, 100 );
	_treeView->header()->setStretchLastSection ( false );
	_scrollAreaParam->setWidget ( _treeView );
	_scrollAreaParam->setWidgetResizable ( true );
	//_scrollAreaParam->setMaximumWidth(sizeWin.width() * 0.1);
	//_scrollAreaParam->resize(sizeWin.width() * 0.3,
	//			sizeHlay.height());

	QSize sizeHleft = _scrollAreaParam->size();

	_scrollArea = new QScrollArea ( _splitter );
	_splitter->addWidget ( _scrollArea );
	_scrollArea->setWidgetResizable ( true );
	_viewport = new QWidget ( _scrollArea );
	_layout = new QVBoxLayout ( _viewport );
//	_viewport->setMaximumWidth(
//			sizeWin.width() - sizeHlay.width() - sizeHleft.width());
	_nbOfPlot = ( _dofNames.size() * _trialNames.size() );
	_viewport->setMinimumHeight ( _nbOfPlot * 400 );
	_scrollArea->setWidget ( _viewport );

	//_splitter->addWidget(_leftVLayoutWidget);

	_scrollArea->resize ( sizeWin.width() - sizeHlay.width() - 50,
	                      sizeHlay.height() );


	_splitter->addWidget ( _scrollAreaParam );

	setCentralWidget ( _splitter );

	{
		unsigned int cpt = 0;

		for ( std::vector<std::string>::const_iterator itTrial =
		            _trialNames.begin(); itTrial < _trialNames.end(); itTrial++ )
		{
			std::stringstream text;
			text << *itTrial;
			_VectItem.push_back (
			    new QTreeWidgetItem ( _treeViewRight ) );
			_VectItem.back()->setText ( 0, QString ( text.str().c_str() ) );
			_VectItem.back()->setData ( 0, Qt::CheckStateRole, Qt::Checked );
			_checkBoxBool.push_back ( true );

			for ( std::vector<std::string>::const_iterator it =
			            _dofNames.begin(); it != _dofNames.end(); it++ )
			{
				std::stringstream text;
				text << *itTrial << ": " << *it;
				_VectItem.push_back (
				    new QTreeWidgetItem ( _VectItem.at ( cpt ) ) );
				_VectItem.back()->setText ( 0, QString ( ( *it ).c_str() ) );
				_VectItem.back()->setData ( 0, Qt::CheckStateRole, Qt::Unchecked );
				_VectButtonItem.push_back ( new QPushButton ( "Save", _treeViewRight ) );
				_treeViewRight->setItemWidget ( _VectItem.back(), 1, _VectButtonItem.back() );
				_VectDock.push_back ( new QDockWidget ( this ) );
				_VectPlot.push_back ( new QCustomPlot ( _VectDock.back() ) );
				title_.push_back ( new QCPPlotTitle ( _VectPlot.back() ) );
				_layout->addWidget ( _VectDock.back() );
				title_.back()->setText ( text.str().c_str() );
				title_.back()->setFont ( QFont ( "sans", 12, QFont::Bold ) );
//				title_.back()->setTextColor(Qt::blue);
				_VectPlot.back()->plotLayout()->insertRow ( 0 );
				_VectPlot.back()->plotLayout()->addElement ( 0, 0, title_.back() );
				_VectPlot.back()->addGraph();
				QPen bluePen;
				bluePen.setColor ( Qt::blue );
				bluePen.setWidthF ( 3 );
				_VectPlot.back()->graph ( 0 )->setPen ( bluePen );
				_VectPlot.back()->graph ( 0 )->setAntialiasedFill ( false );
				_VectPlot.back()->graph ( 0 )->setName ( "Reference" );
				_VectPlot.back()->addGraph();
				QPen redPen;
				redPen.setColor ( Qt::red );
				redPen.setWidthF ( 3 );
				_VectPlot.back()->graph ( 1 )->setPen ( redPen );
				_VectPlot.back()->graph ( 0 )->setBrush ( QBrush ( QColor ( 240, 255, 200 ) ) );
				_VectPlot.back()->graph ( 1 )->setAntialiasedFill ( false );
				_VectPlot.back()->graph ( 1 )->setName ( "Calibration" );
				_VectPlot.back()->graph ( 0 )->setChannelFillGraph ( _VectPlot.back()->graph ( 1 ) );
//
//				std::cout << "_torqueBase: " << _torqueBase.at(std::distance<
//						std::vector<std::string>::const_iterator>(
//						_trialNames.begin(), itTrial)).size() << std::endl;

				_VectPlot.back()->graph ( 0 )->setData (
				    QVector<double>::fromStdVector ( _timeIDBase.at ( std::distance <
				                                     std::vector<std::string>::const_iterator > (
				                                             _trialNames.begin(), itTrial ) ) ),
				    QVector<double>::fromStdVector ( _torqueBase.at ( std::distance <
				                                     std::vector<std::string>::const_iterator > (
				                                             _trialNames.begin(), itTrial ) ).at (
				                                                     _dofIndexToCalibrate.at ( std::distance<std::vector<std::string>::const_iterator> ( _dofNames.begin(), it ) ) ) )
				);
//				_VectPlot.back()->graph(1)->setData(
//						QVector<double>::fromStdVector(_timeBase.at(std::distance<
//														std::vector<std::string>::const_iterator>(
//															_trialNames.begin(), itTrial))),
//						QVector<double>::fromStdVector(_torqueBase.at(std::distance<
//								std::vector<std::string>::const_iterator>(
//								_trialNames.begin(), itTrial)).at(
//								std::distance<std::vector<std::string>::const_iterator>(_dofNames.begin(), it)))
//						);

				_VectPlot.back()->graph ( 0 )->rescaleAxes();
				_VectPlot.back()->graph ( 1 )->rescaleAxes ( true );
				_VectPlot.back()->setInteractions ( QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables );
				_VectPlot.back()->xAxis->setTickLabelType ( QCPAxis::ltDateTime );
				_VectPlot.back()->xAxis->setDateTimeFormat ( "hh:mm:ss:zzz" );
//				_VectPlot.back()->xAxis->setAutoTickStep(false);
//				_VectPlot.back()->xAxis->setTickStep(2);
				_VectPlot.back()->yAxis->setLabel ( "Newton/Meters" );
				_VectPlot.back()->xAxis->setLabel ( "Time" );
				_VectPlot.back()->legend->setVisible ( true );
				_VectDock.back()->setWidget ( _VectPlot.back() );
				_VectDock.back()->hide();
				_VectPlot.back()->replot();
				_nbOfPlot--;
				_checkBoxBool.push_back ( false );
			}

			cpt += _dofNames.size() + 1;
		}
	}

	{

		if ( executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single || executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
		{
			unsigned int cpt = 0;

			_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
			_VectTreeItem.back()->setText ( 0, "Strength coefficients" );

			for ( int i = 0; i < _strengthCoeff; i++ )
			{
				_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem.at ( 0 ) ) );
				_VectTreeItem.back()->setText ( 0, QString::number ( i ) );
				_VectTreeItem.back()->setText ( 1, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 2, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 3, QString::number ( _vecParamUB.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 4, QString::number ( _vecParamLB.at ( cpt ) ) );
				cpt++;
			}

			_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
			_VectTreeItem.back()->setText ( 0, "Shape Factor" );

// 		for (std::vector<std::string>::const_iterator it =
// 				_musclesNames.begin(); it != _musclesNames.end(); it++)
// 		{
// 			_VectTreeItem.push_back(
// 					new QTreeWidgetItem(
// 							_VectTreeItem[_strengthCoeff + 1]));
// 			_VectTreeItem.back()->setText(0, QString(it->c_str()));
			for ( std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++ )
			{
				_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem[_strengthCoeff + 1] ) );
				_VectTreeItem.back()->setText ( 0, QString ( it->c_str() ) );
				_VectTreeItem.back()->setText ( 1, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 2, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 3, QString::number ( _vecParamUB.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 4, QString::number ( _vecParamLB.at ( cpt ) ) );
				cpt++;
			}

// 		}
			//	COUT << "a" << std::endl << std::flush ;

			_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
			_VectTreeItem.back()->setText ( 0, "Tendon Slack Lengths" );

			for ( std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++ )
			{
				_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem[ _musclesNames.size() + _strengthCoeff + 2] ) );
				_VectTreeItem.back()->setText ( 0, QString ( it->c_str() ) );
				_VectTreeItem.back()->setText ( 1, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 2, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 3, QString::number ( _vecParamUB.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 4, QString::number ( _vecParamLB.at ( cpt ) ) );
				cpt++;
			}

			/*COUT << "_vecParamBase.size()" << _vecParamBase.size() << std::endl << std::flush ;
			COUT << "_vecParamUB.size()" << _vecParamUB.size() << std::endl << std::flush ;
			COUT << "_vecParamLB.size()" << _vecParamLB.size() << std::endl << std::flush ;
			COUT << "cpt" << cpt << std::endl << std::flush ;*/

			_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
			_VectTreeItem.back()->setText ( 0, "Optimal Fiber Lengths" );

			for ( std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++ )
			{
				//COUT << "cpt" << cpt << std::endl << std::flush ;
				_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem[_strengthCoeff + 2 * _musclesNames.size() + 3] ) );
				_VectTreeItem.back()->setText ( 0, QString ( it->c_str() ) );
				_VectTreeItem.back()->setText ( 1, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 2, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 3, QString::number ( _vecParamUB.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 4, QString::number ( _vecParamLB.at ( cpt ) ) );
				cpt++;
			}
		}
 		else if ( executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor )
		{
			unsigned int cpt = 0;

			_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
			_VectTreeItem.back()->setText ( 0, "Shape Factor" );

			for ( std::vector<std::string>::const_iterator it = _musclesNames.begin(); it != _musclesNames.end(); it++ )
			{
				_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem[0] ) );
				_VectTreeItem.back()->setText ( 0, QString ( it->c_str() ) );
				_VectTreeItem.back()->setText ( 1, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 2, QString::number ( _vecParamBase.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 3, QString::number ( _vecParamUB.at ( cpt ) ) );
				_VectTreeItem.back()->setText ( 4, QString::number ( _vecParamLB.at ( cpt ) ) );
				cpt++;
			}

		}

		//COUT << "a" << std::endl << std::flush ;

		_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
		_VectTreeItem.back()->setText ( 0, "FOpt" );
		_VectTreeItem.back()->setText ( 1, "0" );

		_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
		_VectTreeItem.back()->setText ( 0, "MPI Iteration" );
		_VectTreeItem.back()->setText ( 1, "0" );

		_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
		_VectTreeItem.back()->setText ( 0, "Number of Eval" );
		_VectTreeItem.back()->setText ( 1, "0" );
		_VectTreeItem.back()->setText ( 2, QString::number ( maxEval ) );

		_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
		_VectTreeItem.back()->setText ( 0, "Convergence" );
		_VectTreeItem.back()->setText ( 1, "0" );
		_VectTreeItem.back()->setText ( 2, QString::number ( minConv ) );

		_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeView ) );
		_VectTreeItem.back()->setText ( 0, "Finish Optimization" );
		_VectTreeItem.back()->setText ( 1, "No" );
	}

	SyncToolsCal::Shared::endGuiMutex.lock();
	SyncToolsCal::Shared::endGui = false;
	SyncToolsCal::Shared::endGuiMutex.unlock();



// 	SyncToolsCal::Shared::readyToStart.wait();

	connect ( &_dataTimer, SIGNAL ( timeout() ), this, SLOT ( _thread() ) );
	_dataTimer.start ( 100 ); // 20Hz

	//COUT << "a" << std::endl << std::flush ;

}

MainWindow::~MainWindow()
{
//	for (std::vector<QCPPlotTitle*>::const_iterator it = title_.begin();
//			it != title_.end(); it++)
//		delete *it;
//
//	for (std::vector<QCustomPlot*>::const_iterator it = _VectPlot.begin();
//			it != _VectPlot.end(); it++)
//		delete *it;
//
//	for (std::vector<QDockWidget*>::const_iterator it = _VectDock.begin();
//			it != _VectDock.end(); it++)
//		delete *it;

//	for (std::vector<QTreeWidgetItem*>::const_iterator it = _VectItem.begin();
//			it != _VectItem.end(); it++)
//		delete *it;
//
//	for (std::vector<QTreeWidgetItem*>::const_iterator it = _VectTreeItem.begin();
//			it != _VectTreeItem.end(); it++)
//		delete *it;

//	delete _layout;
//	delete _rightVLayout;
//	//delete _rightVLayoutWidget;
//	delete _leftVLayoutWidget;
//	delete _viewport;
//	delete _treeViewRight;
//	delete _treeView;
//	//delete _mainHLayout;
//	delete _scrollAreaParam;
//	delete _scrollArea;
//	delete _scrollAreaCheck;
//	delete _splitter;
	delete _ui;
}

void MainWindow::_thread()
{

	SyncToolsCal::Shared::endGuiMutex.lock();

	if ( SyncToolsCal::Shared::endGui )
		stop();

	SyncToolsCal::Shared::endGuiMutex.unlock();


	double fOpt = 0;
	SyncToolsCal::Shared::fOptMutex.lock();
	fOpt = SyncToolsCal::Shared::fOpt;
	SyncToolsCal::Shared::fOptMutex.unlock();

	SyncToolsCal::Shared::torqueCalMutex.lock();
	_torqueCal = SyncToolsCal::Shared::torqueCal;
	SyncToolsCal::Shared::torqueCalMutex.unlock();

	SyncToolsCal::Shared::vecParamCalMutex.lock();
	_vecParamCal = SyncToolsCal::Shared::vecParamCal;
	SyncToolsCal::Shared::vecParamCalMutex.unlock();

	SyncToolsCal::Shared::mpiIterationMutex.lock();
	_mpiIteration = SyncToolsCal::Shared::mpiIteration;
	SyncToolsCal::Shared::mpiIterationMutex.unlock();

	SyncToolsCal::Shared::nbOfEvalMutex.lock();
	int noEval = SyncToolsCal::Shared::nbOfEval;
	SyncToolsCal::Shared::nbOfEvalMutex.unlock();

	SyncToolsCal::Shared::convergenceMutex.lock();
	double convergence = SyncToolsCal::Shared::convergence;
	SyncToolsCal::Shared::convergenceMutex.unlock();

	SyncToolsCal::Shared::finishMutex.lock();
	bool finishOpt =  SyncToolsCal::Shared::finish;
	SyncToolsCal::Shared::finishMutex.unlock();

	if ( finishOpt )
	{
		_VectTreeItem.back()->setText ( 1, "Yes" );
		//_dataTimer.stop();
	}
	else
	{

		unsigned int cptDock = 0;
		unsigned int cptItem = 0;

		for ( std::vector<std::string>::const_iterator itTrial =
		            _trialNames.begin(); itTrial < _trialNames.end(); itTrial++ )
		{
			if ( _VectItem.at ( cptItem )->checkState ( 0 ) == Qt::Unchecked
			        && _checkBoxBool.at ( cptItem ) == true )
			{
				_checkBoxBool[cptDock] = false;
				cptItem++;

				for ( std::vector<std::string>::const_iterator it =
				            _dofNames.begin(); it != _dofNames.end(); it++ )
				{
					_VectDock.at ( cptDock )->hide();

					if ( _nbOfPlot != 0 )
						_nbOfPlot--;

					_checkBoxBool[cptItem] = false;
					_VectItem.at ( cptItem )->setCheckState ( 0, Qt::Unchecked );
					cptDock++;
					cptItem++;

				}
			}
			else if ( _VectItem.at ( cptItem )->checkState ( 0 ) == Qt::Checked
			          && _checkBoxBool.at ( cptItem ) == false )
			{
				_checkBoxBool[cptItem] = true;
				cptItem++;

				for ( std::vector<std::string>::const_iterator it =
				            _dofNames.begin(); it != _dofNames.end(); it++ )
				{
					_VectDock.at ( cptDock )->show();
					_nbOfPlot++;
					_checkBoxBool[cptItem] = true;
					_VectItem.at ( cptItem )->setCheckState ( 0, Qt::Checked );
					cptDock++;
					cptItem++;

				}
			}
			else
			{
				cptItem++;

				for ( std::vector<std::string>::const_iterator it =
				            _dofNames.begin(); it != _dofNames.end(); it++ )
				{
					if ( _VectItem.at ( cptItem )->checkState ( 0 ) == Qt::Unchecked
					        && _checkBoxBool.at ( cptItem ) == true )
					{
						_VectDock.at ( cptDock )->hide();

						if ( _nbOfPlot != 0 )
							_nbOfPlot--;

						_checkBoxBool[cptItem] = false;
						cptDock ++;
						cptItem++;

					}
					else if ( _VectItem.at ( cptItem )->checkState ( 0 ) == Qt::Checked
					          && _checkBoxBool.at ( cptItem ) == false )
					{
						_VectDock.at ( cptDock )->show();
						_nbOfPlot++;
						_checkBoxBool[cptItem] = true;
						cptDock++;
						cptItem++;

					}
					else
					{
						cptItem++;
						cptDock++;
					}
				}
			}
		}

		_viewport->setMinimumHeight ( _nbOfPlot * 400 );

		unsigned int cpt = 0;

		for ( std::vector<std::string>::const_iterator itTrial =
		            _trialNames.begin(); itTrial < _trialNames.end(); itTrial++ )
		{
			for ( std::vector<std::string>::const_iterator it =
			            _dofNames.begin(); it != _dofNames.end(); it++ )
			{
				if ( _VectButtonItem.at ( cpt )->isDown() == true )
				{
					QString date = QDateTime::currentDateTime().toString();
					std::stringstream text;
					text << "torque_" << *itTrial << "_" << *it << "_" << fOpt << "_" << date.toStdString() << ".pdf";
					_VectPlot.at ( cpt )->savePdf ( QString ( text.str().c_str() ) );
				}

				cpt++;
			}
		}

		if ( _vecParamCal.size() != 0 && _vecParamCal.size() >= _musclesNames.size() )
		{
			unsigned int cpt = 0;

			if ( executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single || executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{



				for ( int i = 0; i < _strengthCoeff; i++ )
				{
					_VectTreeItem.at ( cpt + 1 )->setText ( 1, QString::number ( _vecParamCal.at ( cpt ) ) );
					cpt++;
				}

				for ( std::vector<std::string>::const_iterator it =
				            _musclesNames.begin(); it != _musclesNames.end(); it++ )
				{
					_VectTreeItem.at ( cpt + 2 )->setText ( 1, QString::number ( _vecParamCal.at ( cpt ) ) );
					cpt++;
				}

				for ( std::vector<std::string>::const_iterator it =
				            _musclesNames.begin(); it != _musclesNames.end(); it++ )
				{
					_VectTreeItem.at ( cpt + 3 )->setText ( 1, QString::number ( _vecParamCal.at ( cpt ) ) );
					cpt++;
				}

				for ( std::vector<std::string>::const_iterator it =
				            _musclesNames.begin(); it != _musclesNames.end(); it++ )
				{
					_VectTreeItem.at ( cpt + 4 )->setText ( 1, QString::number ( _vecParamCal.at ( cpt ) ) );
					cpt++;
				}

				cpt += 4;

			}
			else if ( executionSimulatedAnnealing_->GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor )
			{
				for ( std::vector<std::string>::const_iterator it =
				            _musclesNames.begin(); it != _musclesNames.end(); it++ )
				{
					_VectTreeItem.at ( cpt + 1 )->setText ( 1, QString::number ( _vecParamCal.at ( cpt ) ) );
					cpt++;
				}
			}

			_VectTreeItem.at ( cpt)->setText ( 1, QString::number ( fOpt ) );
			_VectTreeItem.at ( cpt + 1 )->setText ( 1, QString::number ( _mpiIteration ) );
			_VectTreeItem.at ( cpt + 2 )->setText ( 1, QString::number ( noEval ) );
			_VectTreeItem.at ( cpt + 3 )->setText ( 1, QString::number ( convergence ) );

		}

		if ( _torqueCal.size() != 0 )
		{
			int cptPlot = 0;

			

			for ( std::vector<std::string>::const_iterator itTrial =
			            _trialNames.begin(); itTrial < _trialNames.end(); itTrial++ )
			{
//			std::cout << "_torqueCal: " << _torqueCal.at(std::distance<
//							std::vector<std::string>::const_iterator>(
//							_trialNames.begin(), itTrial)).size() << std::endl;

				

				
				for ( std::vector<std::string>::const_iterator it =
				            _dofNames.begin(); it != _dofNames.end(); it++ )
				{
					const int cpt = std::distance<std::vector<std::string>::const_iterator>(_dofNames.begin(), it);

					int startSample = 0;
				/*	if (_torqueBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).at(_dofIndexToCalibrate.at(cpt)).size() > 200)
						startSample = 50;
					else
						startSample = 10;
				*/
					std::vector<double> dataID, dataTorque, timeID, timeTorque;
					dataID.assign(_torqueBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).at(_dofIndexToCalibrate.at(cpt)).begin() + startSample, _torqueBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).at(_dofIndexToCalibrate.at(cpt)).end());

					dataTorque.assign(_torqueCal.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).at(_dofIndexToCalibrate.at(cpt)).begin() + startSample, _torqueCal.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).at(_dofIndexToCalibrate.at(cpt)).end());

					timeID.assign(_timeIDBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).begin() + startSample, _timeIDBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).end());

					timeTorque.assign(_timeIKBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).begin() + startSample, _timeIKBase.at(std::distance <
						std::vector<std::string>::const_iterator >(
						_trialNames.begin(), itTrial)).end());



					_VectPlot.at ( cptPlot )->graph ( 0 )->setData (
						QVector<double>::fromStdVector(timeID),
																 QVector<double>::fromStdVector(dataID)
					);
					_VectPlot.at ( cptPlot )->graph ( 1 )->setData (
						QVector<double>::fromStdVector(timeTorque),
																 QVector<double>::fromStdVector(dataTorque)
					);
					_VectPlot.at ( cptPlot )->graph ( 0 )->rescaleAxes();
					_VectPlot.at ( cptPlot )->graph ( 1 )->rescaleAxes ( true );
					_VectPlot.at ( cptPlot )->replot();
					cptPlot++;
				}
			}
		}
	}
}

void MainWindow::closeEvent ( QCloseEvent* event )
{
	this->close();
	_dataTimer.stop();
}

void MainWindow::stop()
{
	this->close();
	_dataTimer.stop();
}

void MainWindow::stopRefresh()
{
	_dataTimer.setInterval(1000);
}

