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
