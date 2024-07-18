/*
 * DynLibBase.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#ifndef DYNLIBOPTIMIZATION_H_
#define DYNLIBOPTIMIZATION_H_

#include "CommonCEINMS.h"
#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <OptimizationPlugin.h>
#include <typeinfo>

#if defined(UNIX) || defined(APPLE)
#include <dlfcn.h>
#define HINSTANCE void*
#endif

#ifdef WINDOWS
#include <windows.h>
#endif

using namespace std;

template <typename NMSmodelT>
class DynLibOptimization
{
	public:
		DynLibOptimization()
		{

		}

		~DynLibOptimization();

		/**
		 * Load dynamically a library for getting the data from a driver.
		 */
		void setDynLib ( const string& libpath );

		void closeDynLib();

		OptimizationPlugin<NMSmodelT>* getPlugin()
		{
			return plugin_;
		}

	protected:

		void createDestroySpecialization(std::string libpath);
		void destroyPlugin();

		HINSTANCE handle_;
		OptimizationPlugin<NMSmodelT>* plugin_;
		destroy_cEASoff* destroyProducerEASoff_;
		destroy_cEAS* destroyProducerEAS_;
		destroy_cEAEB* destroyProducerEAEB_;
		destroy_cEAE* destroyProducerEAE_;
};

#endif /* DYNLIBBASE_H_ */
