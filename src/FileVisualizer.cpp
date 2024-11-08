#include <iostream>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <SyncTools.h>
#include "DataFromFile.h"
#include "mainwindowViz.h"




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
	
	const std::vector<std::string>& colName = angleDataFromFile.getColumnNames();
	int nbOfSample = angleDataFromFile.getNoTimeSteps();
	std::vector<std::vector<double> > data(colName.size());
	std::vector<double> time;
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
	
	MainWindow gui(colName, data, time);
	gui.show();
	a.exec();

	return(EXIT_SUCCESS);
}
