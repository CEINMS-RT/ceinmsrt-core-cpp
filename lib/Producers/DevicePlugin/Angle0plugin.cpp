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

#include "Angle0plugin.h"

Angle0plugin::Angle0plugin() {

}

Angle0plugin::~Angle0plugin() {

}

void Angle0plugin::init(string xmlName, string executionName)
{
#ifdef VERBOSE
//        	std::cout << "init angle lib" << std::endl;
#endif

	timeNow_ = 0;

	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(xmlName, xml_schema::flags::dont_initialize));
		// DOF iteration
		NMSmodelType::DoFs_type& dofs(subjectPointer->DoFs());
		DoFsType::DoF_sequence& dofSequence(dofs.DoF());
		for (DoFsType::DoF_iterator i = dofSequence.begin(); i != dofSequence.end(); ++i)
		{
			string currentDOF((*i).name());
			nameSet_.insert(currentDOF);
		}
	} catch (const xml_schema::exception& e)
	{
		std::cout << e << std::endl;
		exit(EXIT_FAILURE);
	}

	for(set<string>::const_iterator it = nameSet_.begin(); it != nameSet_.end(); it++)
		mapData_[*it] = 0;

	std::cout << "Plugin Angle0plugin, initialisation done." << std::endl;
}

const double& Angle0plugin::getTime()
{
#ifdef UNIX
		struct timeval now;

	gettimeofday ( &now, NULL );
	
	timeNow_ = ( now.tv_sec ) + 0.000001 * now.tv_usec;
#endif
#ifdef WIN32
	static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

	SYSTEMTIME  system_time;
	FILETIME    file_time;
	uint64_t    time;
	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);
	time = ((uint64_t)file_time.dwLowDateTime);
	time += ((uint64_t)file_time.dwHighDateTime) << 32;

	long tv_sec = (long)((time - EPOCH) / 10000000L);
	long tv_usec = (long)(system_time.wMilliseconds * 1000);

	timeNow_ = (tv_sec) + 0.000001 * tv_usec;

	//std::cout << std::setprecision(16) << timeNow_ << std::endl << std::flush;

#endif
	return timeNow_;
}

#ifdef UNIX
extern "C" ProducersPluginVirtual* create() {
    return new Angle0plugin;
}

extern "C" void destroy(ProducersPluginVirtual* p) {
    delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) ProducersPluginVirtual* __cdecl create() {
	return new Angle0plugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif