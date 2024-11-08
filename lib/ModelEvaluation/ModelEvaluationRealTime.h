// $Id$
/**
 * @file ModelEvaluationRealTime.h
 *
 * @class ModelEvaluationRealTime
 *
 * @brief Get EMg and compute the EMG-driven model parameters
 *
 * @details Sartori, Massimo, et al. "EMG-driven forward-dynamic estimation of muscle force and joint moment about multiple degrees of freedom in the human lower extremity." PloS one 7.12 (2012): e52618.
 *
 * @author Guillaume Durandau guillaume.durandau@bccn.uni-goettingen.de
 *
 * @version 1.00
 *
 * @date Feb 26, 2015
 */
// $Log$

#ifndef MODELEVALUATIONREALTIME_H_
#define MODELEVALUATIONREALTIME_H_

/*
#ifdef WIN32
#undef WIN32_LEAN_AND_MEAN  
#include <windows.h>
#endif*/

#include "CommonCEINMS.h"
#include "ModelEvaluationBase.h"
#include "DynLib.h"
#include "DynLibOptimization.h"
#include "execution.hxx"
#include "AngleAndComsumerPlugin.h"
#include "EmgAndAngleAndComsumerPlugin.h"
#include "ExecutionXmlReader.h"
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>
#include <boost/unordered_map.hpp>
#include "OpenSimFileLogger.h"
#include "SyncTools.h"
using std::cout;
using std::endl;
using std::vector;
using std::string;
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <Filter.h>
#include "ExecutionXmlReader.h"
#include <ctime>
#include <getTime.h>

template <typename NMSmodelT>
class ModelEvaluationRealTime: public ModelEvaluationBase, public DynLib<ComsumerPlugin>
{
	public:
		// Constructor
		ModelEvaluationRealTime ( std::string xmlName, /*!<  */
								  NMSmodelT& subject, /*!< EMG-driven model class */
								  std::string executionName, /*!< Name of the execution XML */
								  bool record = false, /*!< Record data to file */
								  std::string recordDirectory = std::string(), /*!< Name of the directory for recording on file */
								  std::string processDirectory = std::string(), /*!< Name of the directory if we read from file */
								  bool process = false /*!< if we process drom file */
								);
		
		// Desctructor
		virtual ~ModelEvaluationRealTime();
		
		// Function for threading 
		void operator() ();
		
		// Set the name of external plugin that give DOF angle and take EMG-driven torque.
		void setPluginAAC ( AngleAndComsumerPlugin* pluginAAC /*!< pointor to the external plugin */)
		{
			_pluginAAC = pluginAAC;
		}

		// Set the name of external plugin that give EMG and DOF angle and take EMG-driven torque.
		void setPluginEAC(EmgAndAngleAndComsumerPlugin* pluginEAC /*!< pointor to the external plugin */)
		{
			_pluginEAC = pluginEAC;
		}

	protected:

		typedef std::vector<std::string> VecStr;
		typedef std::vector<std::vector<double> > VecVecDouble;
		typedef std::map<std::string, std::vector <std::string> > MapStrVectStr;
		typedef boost::unordered_map<std::string, std::vector <std::string> > UnMapStrVectStr;

		NMSmodelT& 									subject_; 			/*!< EMG-driven model */
		std::vector< std::string > 					dofNames_;			/*!< DOF name from model */
		std::vector<std::string> 					muscleNames_;		/*!< Muscle name from model */
		std::vector< std::vector<std::string> > 	muscleNamesOnDof_;	/*!< Muscle name distributed on DOF name */
		std::vector<FilterKin::AvrFilt<double>* > 	_torqueFilter;		/*!< Filter for torque */
		int 										noDof_;				/*!< number of DOF */
		int 										cpt_;				/*!<  */
		int 										_verbose;			/*!< Level of verbose */
		bool 										_record;			/*!< Record on file */
		bool 										_gui;				/*!< Use of GUI */
		bool 										_process;			/*!< process from file */
		bool										_useOfEmgAndAngleAndComsumerPlugin; /* if the AngleAndComsumerPlugin also gives EMG*/
		std::string 								_recordDirectory;	/*!< name of directory for recording on file */
		std::string 								_executionName;		/*!< Execution XML file name */
		std::string 								xmlName_;			/*!< CEINMS subject xml name */
		std::string 								_processDirectory;	/*!< name of directory for reading from file */
		UnMapStrVectStr 							channelNameOnMuscle_;/*!< name of channnel use by muscle name */
		AngleAndComsumerPlugin* 					_pluginAAC;			/*!< Pointor to the external plugin that need EMG-driven torque */
		EmgAndAngleAndComsumerPlugin*				_pluginEAC;			/*!< Pointor to the external plugin that need EMG-driven torque */
		DynLib<ProducersPluginVirtual> 				_emgPlugin;			/*!< class for external plugin */
		DynLibOptimization<NMSmodelT>				_optimizationPlugin; /*!< clas fr optimization dynamic load optimization */

		// Wait for the producer thread to finish getting and proceesing the data
		void WaitForProducer();
		
		// Get the EMG data and time of recording from the plugin
		void getEMG ( std::vector<double>& EMGData, /*!< EMG from the plugin */
					  double& emgTime /*!< Time of recording of the EMG */
					);
		
		// Initialisation of the external plugin for the EMG
		void initEMGPlugin();

		void getMusclesNames ( std::vector< std::string >& muscleNamesFromModel )
		{
			muscleNamesFromModel = InterThread::getMusclesNames();
		}

		void setMusclesNames ( const std::vector< std::string >& musclesNames )
		{
			InterThread::setMusclesNames ( musclesNames );
		}
};

#include "ModelEvaluationRealTime.cpp"

#endif /* MODELEVALUATIONREALTIME_H_ */
