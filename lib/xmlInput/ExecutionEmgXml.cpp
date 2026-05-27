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

#include "ExecutionEmgXml.h"
#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <boost/dll/runtime_symbol_info.hpp>


ExecutionEmgXml::ExecutionEmgXml ( const std::string& fileName )
{
	try
	{
		std::auto_ptr<ExecutionEMGType> temp ( executionEMG ( fileName, xml_schema::flags::dont_initialize ) );
		_executionPointer = temp;
	}
	catch ( const xml_schema::exception& e )
	{
		std::cout << e << std::endl;
		exit ( EXIT_FAILURE );
	}

	_filename = fileName;

	// Gte IP and port
	_ip = _executionPointer->ip();
	_port = _executionPointer->port();

	// Get filter parameters
	_aCoeffHP = _executionPointer->hpFilter().aCoeff();
	_bCoeffHP = _executionPointer->hpFilter().bCoeff();
	_aCoeffLP = _executionPointer->lpFilter().aCoeff();
	_bCoeffLP = _executionPointer->lpFilter().bCoeff();

	_maxAmp = _executionPointer->maxEMG();
}

ExecutionEmgXml::~ExecutionEmgXml()
{

}

void ExecutionEmgXml::writeEmgXmlFile ( const std::string& fileName )
{
	xml_schema::namespace_infomap map;
	map[""].name = "";
	map[""].schema = fs::absolute(boost::dll::program_location().parent_path().string() + "/../../../XSD/executionEMG.xsd").string() ;

	MaxEMG maxEmg;
	for(std::vector<double>::const_iterator it = _maxAmp.begin(); it < _maxAmp.end(); it++)
		maxEmg.push_back(*it);
	
	_executionPointer->maxEMG ( maxEmg );

	std::ofstream ofs ( fileName.c_str() );
	executionEMG ( ofs, *_executionPointer, map );
}

void ExecutionEmgXml::UpdateEmgXmlFile ()
{
	writeEmgXmlFile ( _filename );
}
