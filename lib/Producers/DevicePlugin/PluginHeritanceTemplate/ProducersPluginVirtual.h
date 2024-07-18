/*
 * ProducersPluginVirtual.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#ifndef PRODUCERSPLUGINVIRTUAL_H_
#define PRODUCERSPLUGINVIRTUAL_H_

#include <string>
#include <map>
#include <vector>
#include <set>

/**
 *@TODO Add joint Torque
 **/

using namespace std;

class ProducersPluginVirtual
{
	public:
		ProducersPluginVirtual() {};
		~ProducersPluginVirtual() {};
		virtual void init ( std::string executionXMLFile = std::string(), std::string subjectCEINMSXMLFile = std::string() ) = 0;
		virtual void reset() = 0;
		virtual void stop() = 0;
		virtual const map<string, double>& GetDataMap() = 0; //< For having Data and name correspondence
		virtual const map<string, double>& GetDataMapTorque() = 0; //< For having Data and name correspondence
		virtual const double& getTime() = 0;
		virtual void setDirectories ( std::string outDirectory, std::string inDirectory = std::string() ) = 0;
		virtual void setVerbose ( int verbose ) = 0;
		virtual void setRecord ( bool record ) = 0;
};

typedef ProducersPluginVirtual* create_t();
typedef void destroy_t ( ProducersPluginVirtual* );

#endif /* PRODUCERSPLUGINVIRTUAL_H_ */
