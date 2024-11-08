/*
 * DynLib.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

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
