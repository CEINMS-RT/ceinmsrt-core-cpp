// $Id$
/**
 * @file LmtMaFromMTUSpline.h
 * 
 * @class LmtMaFromMTUSpline
 *
 * @brief Get DOF position, DOF torque and compute the Lmt and MA using Bspline
 * 
 * @details See Sartori, Massimo, et al. "Estimation of musculotendon kinematics in large musculoskeletal models using multidimensional B-splines." Journal of biomechanics 45.3 (2012): 595-601.
 *
 * @author Guillaume Durandau guillaume.durandau@bccn.uni-goettingen.de
 * 
 * @version 1.00
 * 
 * @date Feb 24, 2015
 */
// $Log$

#ifndef LMTMAFROMMTUSPLINE_H_
#define LMTMAFROMMTUSPLINE_H_

#include "CommonCEINMS.h"
#include <string>
#include <vector>
//#include <boost/shared_ptr.hpp>
#include <memory>
#include <ProducersPluginVirtual.h>
#include "DynLib.h"
#include "LmtMaFromX.h"
#include "SyncTools.h"
#include "MTUSplineDataRead.h"
#include <csignal>
#include <stddef.h>
#include <boost/filesystem.hpp>
#include <iomanip>
#include "OpenSimFileLogger.h"
#include "AngleAndComsumerPlugin.h"
#include "EmgAndAngleAndComsumerPlugin.h"
#include "ExecutionXmlReader.h"
#include "executionIK_ID.hxx"
#include <chrono>
#include <thread>
#ifdef WIN32
#include <windows.h>
#endif
#include <getTime.h>

//TODO: create a class for spline lmt and ma computation

using namespace std;

class LmtMaFromMTUSpline: public LmtMaFromX, public DynLib<ProducersPluginVirtual> {
public:
	
	// Constructor
	LmtMaFromMTUSpline() {}
	
	// Constructor
	LmtMaFromMTUSpline(	const std::string& subjectSpecificXml, /*!< CEINMS specific XML */
						const std::string& subjectName, /*!< Name of the subject */
						string executionName, /*!< Execution XML */
						bool record = false, /*!< Record on file */
						std::string recordDirectory = std::string(), /*!< Directory where the file will be save */ 
						std::string processDirectory = std::string(), /*!< Directory name if we read from file */
						bool process = false /*!< If we read from file */
					  );
	
	// Desctructor
	virtual ~LmtMaFromMTUSpline();
	
	// Function for threading 
	void operator()();
	
	// Set the name of external plugin that give DOF angle and take EMG-driven torque.
	void setPluginAAC(AngleAndComsumerPlugin* pluginAAC /*!< pointor to the external plugin */)
	{
		_pluginAAC = pluginAAC;
	}

	// Set the name of external plugin that give EMG and DOF angle and take EMG-driven torque.
	void setPluginEAC(EmgAndAngleAndComsumerPlugin* pluginEAC /*!< pointor to the external plugin */)
	{
		_pluginEAC = pluginEAC;
	}
	
protected:

	// Template function for Spline computation with order > 1 
	template<class T>
	void computeLmtMafromSplines(T& splines, int dim,
			const std::vector<double>& angles, std::vector<double>& lmt,
			std::vector<std::vector<double> >& ma);

	// Template function for Spline conputation with order 1
	void computeLmtMafromSplines(
			std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
			const std::vector<double>& angles, std::vector<double>& lmt,
			std::vector<std::vector<double> >& ma);
	
	// Get DOF angle, torque data and time of recording from plugin
	void getAngle(std::vector<double>& angleData, /*!< Angle data from plugin */
				  std::vector<double>& idData, /*!< Torque data from plufin */
				  double& angleTime /*!< time of the recording */
				 );
	
	// Initialisation of the external plugin
	void initAnglePlugin();
	
	std::vector<std::vector<std::string> > 	musclesNamesOnDof_; /*!< Name of the muscle on the different DOF from the model */
	std::vector<std::string> 				dofNames_; 			/*!< Dof name from the model */
	std::vector<std::string> 				muscleNames_;		/*!< Muscle name from model */
	std::string 							subjectSpecificXml_;/*!< CEINMS subject XML */
	std::string 							subjectName_;		/*!< name of the subject */
	std::string 							executionName_;		/*!< name of the execution XML */
	std::string 							_recordDirectory;	/*!< Directory where the file will be save */
	std::string 							_processDirectory;	/*!< Directory name if we read from file */
	double 									_lmtTimePast;		/*!< past time of the lmt Ma recording */
	int 									noMuscles_;			/*!< Number of muscle */
	int 									_verbose;			/*!< Level of verbise */
	bool 									_gui;				/*!< Use of the GUI */
	bool 									_record;			/*!< Record on filr */
	bool 									_process;			/*!< read from file */
	bool 									_useOfAngleAndComsumerPlugin;/*!< use of external plugin that need EMG-driven torque */
	bool									_useOfEmgAndAngleAndComsumerPlugin; /* if the AngleAndComsumerPlugin also gives EMG*/
	bool 									_pluginBool;		/*!< use of external plugin */
	DynLib<ProducersPluginVirtual> 			_anglePlugin;		/*!< class for external plugin */
	AngleAndComsumerPlugin* 				_pluginAAC;			/*!< Pointor to the external plugin that need EMG-driven torque */
	EmgAndAngleAndComsumerPlugin*			_pluginEAC;			/*!< Pointer to the external plugin that gives EMG and angle and needs torque*/
};

#endif /* LMTMAFROMMTUSPLINE_H_ */
