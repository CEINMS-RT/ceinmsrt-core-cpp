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

#ifndef MTUSPLINEINTERFACE_H_
#define MTUSPLINEINTERFACE_H_

#ifdef WIN32
#include <windows.h>
#endif

#include <string>
#include <vector>
#include "MTUSplineDataRead.h"
#include "ExecutionXmlReader.h"
#include "executionIK_ID.hxx"
//#include <boost/shared_ptr.hpp>
#include <memory>
#include "NMSmodel.h"

using namespace std;

class MTUSplineInterface
{
public:
	MTUSplineInterface();
	MTUSplineInterface(const std::string& subjectSpecificXml, /*!< CEINMS specific XML */const std::string& subjectName /*!< Name of the subject */);
	~MTUSplineInterface();

	vector< vector<double> > getMA();
	vector<double> getLMT();
	void setPosition(vector<double> position);
	void initialisation();
	void initialisationFromXML();
	void setDOFName(vector<string> DofName);
	void setMusclesNamesOnDof(std::vector<std::vector<std::string> > 	musclesNamesOnDof);
	void setMuscleName(std::vector<std::string> muscleNames);
	std::vector<std::string> getMuscleName();
	std::vector<std::vector<std::string> > getMusclesNamesOnDof();
	vector<string> getDOFName();


protected:

	// Template function for Spline computation with order > 1 
	template<class T>
	void computeLmtMafromSplines(
		T& splines, int dim,
		const std::vector<double>& angles, std::vector<double>& lmt,
		std::vector<std::vector<double> >& ma
	);

	// Template function for Spline conputation with order 1
	void computeLmtMafromSplines(
		std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
		const std::vector<double>& angles, std::vector<double>& lmt,
		std::vector<std::vector<double> >& ma
	);

	std::vector<std::vector<std::string> > 	musclesNamesOnDof_; /*!< Name of the muscle on the different DOF from the model */
	std::vector<std::string> 				dofNames_; 			/*!< Dof name from the model */
	std::vector<std::string>				muscleNames_;
	std::string 							subjectSpecificXml_;/*!< CEINMS subject XML */
	std::string 							subjectName_;		/*!< name of the subject */
	std::string 							executionName_;		/*!< name of the execution XML */
	std::vector<std::vector<double> >		ma_;
	std::vector<double>						lmt_;
	std::vector<MTUSplineDataRead::Task>	taskMTU_;
	int										noMuscles_;
	MTUSplineDataRead*						splineData_;
};

#endif