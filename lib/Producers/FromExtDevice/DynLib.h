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

#ifndef DynLib_H_
#define DynLib_H_

#include "CommonCEINMS.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <boost/shared_ptr.hpp>

//For template definition

#include <ProducersPluginVirtual.h>
#include <ComsumerPlugin.h>
#include <AngleAndComsumerPlugin.h>
#include "EmgAndAngleAndComsumerPlugin.h"
#include "EMGAndAnglePlugin.h"
#include "ProducersAndConsumerPlugin.h"

#if defined(UNIX) || defined(APPLE)
#include <dlfcn.h>
#define HINSTANCE void*
#endif

#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif


#include <boost/dll/runtime_symbol_info.hpp>

template<class pluginType>
class DynLib
{
public:
    DynLib()
    {

    }

    virtual ~DynLib();

    /**
     * Load dynamically a library for getting the data from a driver.
     *
     * @return False in case there was a problem loading the DLL
     */
    bool setDynLib(const string& libpath);

    void closeDynLib();

    pluginType* getPlugin()
    {
        return plugin_;
    }

protected:
    typedef pluginType* create_t();
    typedef void destroy_t(pluginType*);

    HINSTANCE handle_;
    pluginType* plugin_;
    destroy_t* destroyProducer_;
};

#endif /* DynLib_H_ */
