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

#include "EMG0plugin.h"
#include <ProducersPluginVirtual.h>
#include <cmath>
#include "time.h"
#include <getTime.h>

EMG0plugin::EMG0plugin() {
	// TODO Auto-generated constructor stub

}

EMG0plugin::~EMG0plugin() {
	// TODO Auto-generated destructor stub
}

void EMG0plugin::init(string xmlName, string executionName)
{
#ifdef VERBOSE
	COUT << "init EMG lib" << endl;
#endif

	timeNow_ = rtb::getTime();

	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(xmlName, xml_schema::flags::dont_initialize));
		// DOF iteration
		const NMSmodelType::Channels_type& channel = subjectPointer->Channels();
		const NMSmodelType::Channels_type::Channel_sequence& channelSeq = channel.Channel();
		for (NMSmodelType::Channels_type::Channel_const_iterator i = channelSeq.begin(); i != channelSeq.end(); ++i)
		{
			string currentChannel((*i).name());
			nameSet_.insert(currentChannel);
			mapData_[currentChannel] = 0;
		}
	} catch (const xml_schema::exception& e)
	{
		COUT << e << endl;
		exit(EXIT_FAILURE);
	}
#ifdef VERBOSE
	COUT << "Plugin EMG0plugin, initialisation done." << endl;
#endif
}

const double& EMG0plugin::getTime()
{
	return timeNow_;
}

#ifdef UNIX
extern "C" ProducersPluginVirtual* create() {
    return new EMG0plugin;
}

extern "C" void destroy(ProducersPluginVirtual* p) {
    delete p;
}
#endif
#ifdef WIN32
extern "C" __declspec (dllexport) ProducersPluginVirtual* __cdecl create() {
	return new EMG0plugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif