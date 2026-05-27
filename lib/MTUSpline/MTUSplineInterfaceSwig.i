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

%module MTUSplineInterfaceSwig
%{
#include "MTUSplineInterface.h"
%}

%include "std_vector.i"
%include "std_string.i"

using namespace std;

class MTUSplineInterface
{
public:
	MTUSplineInterface();
	MTUSplineInterface(const std::string& subjectSpecificXml, const std::string& subjectName );
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
};


namespace std {
   %template(vectors) vector<string>;
   %template(vectord) vector<double>;
   %template(vectordd) vector<vector<double> >;
   %template(vectorss) vector<vector<string> >;
};