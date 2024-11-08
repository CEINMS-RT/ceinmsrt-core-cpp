/*
 * EMG0plugin.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

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