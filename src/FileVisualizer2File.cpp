#include <iostream>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <SyncTools.h>
#include "DataFromFile.h"
#include "mainwindowViz2File.h"




void CalSigintHandler(int sig)
{
	InterThread::setEndThread(true);
}

int main(int argc, char** argv)
{
	
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	QApplication::setGraphicsSystem ( "raster" );
#endif
	QApplication a ( argc, argv );
	
	DataFromFile angleDataFromFile ( argv[1] );
	
	DataFromFile angleDataFromFile2 ( argv[2] );
	
	std:vector<std::string> plotname;
	plotname.push_back(argv[1]);
	plotname.push_back(argv[2]);
	
// 	std::cout << argv[2] << std::endl;
	
	const std::vector<std::string>& colName = angleDataFromFile.getColumnNames();
	int nbOfSample = angleDataFromFile.getNoTimeSteps();
	
	const std::vector<std::string>& colName2 = angleDataFromFile2.getColumnNames();
	int nbOfSample2 = angleDataFromFile2.getNoTimeSteps();
	
	std::vector<std::vector<double> > data(colName.size()), data2(colName2.size());
	
// 	std::cout << colName2.size() << std::endl;
	
	std::vector<double> time, time2;
	bool firstPass = true;
	double initialTime = 0;
	
	for ( int j = 0; j < nbOfSample; ++j )
	{
		angleDataFromFile.readNextData();
		
		if(firstPass)
		{
			initialTime = angleDataFromFile.getCurrentTime();
			firstPass = false;
			time.push_back ( 0 );
		}
		else
		{
			time.push_back ( angleDataFromFile.getCurrentTime() - initialTime );
// 			std::cout << time.back() << " == ";
		}
		
		const std::vector<double>& angleDataTemp = angleDataFromFile.getCurrentData();
// 		std::cout << angleDataTemp.back() << std::endl;

		for ( std::vector<double>::const_iterator it = angleDataTemp.begin(); it != angleDataTemp.end(); it++ )
			data[std::distance<std::vector<double>::const_iterator> ( angleDataTemp.begin(), it )].push_back ( *it );
	}
	
	//firstPass = true;
	//initialTime = 0;
	
	for ( int j = 0; j < nbOfSample2; ++j )
	{
		angleDataFromFile2.readNextData();
		
	/*	if(firstPass)
		{
			initialTime = angleDataFromFile2.getCurrentTime();
			firstPass = false;
			time2.push_back ( 0 );
		}
		else
		{*/
			time2.push_back ( angleDataFromFile2.getCurrentTime() - initialTime );
// 			std::cout << time.back() << " == ";
//		}
		
		const std::vector<double>& angleDataTemp = angleDataFromFile2.getCurrentData();
//  		std::cout << angleDataTemp.size() << std::endl;

		for ( std::vector<double>::const_iterator it = angleDataTemp.begin(); it != angleDataTemp.end(); it++ )
			data2[std::distance<std::vector<double>::const_iterator> ( angleDataTemp.begin(), it )].push_back ( *it );
	}
	
	MainWindow gui(plotname, colName, data, time, data2, time2);
	gui.show();
	a.exec();

	return(EXIT_SUCCESS);
}
