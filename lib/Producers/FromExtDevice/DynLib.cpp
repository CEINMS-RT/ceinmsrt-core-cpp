/*
 * DynLib.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#include "DynLib.h"

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

template<class pluginType>
DynLib<pluginType>::~DynLib()
{}

template<class pluginType>
bool DynLib<pluginType>::setDynLib(const string& libpath)
{
    fs::path fsLibPath = libpath;
    fs::path resolvedPath;

    // Allow for plugins to be inserted without extension. This is relevant for tests, especially
    #ifdef WIN32
        std::string fileExtension = ".dll";
    #else
        std::string fileExtension = ".so";
    #endif

    if(fsLibPath.extension().string() == ""){
        fsLibPath = fs::path(fsLibPath.string() + fileExtension);
    }

    if(fsLibPath.is_absolute())   // If path is absolute, use it as it is
        resolvedPath = fsLibPath;
    else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
        resolvedPath = fs::absolute(fsLibPath.filename().string());
    else                                                // Otherwise, load from the same folder as the executable is running from. This allows the executable to be independent of current work directory for plugins.
        resolvedPath = fs::absolute(boost::dll::program_location().parent_path().string() + "/" + fsLibPath.filename().string());

    std::ifstream file_stream(resolvedPath);
    if (!file_stream.is_open()) {
        COUT << "Cannot open plugin because the file `" << resolvedPath.string()
            << "` cannot be opened - verify the path is correct and the file exists" << std::endl << std::flush;

        return false;
    }
    file_stream.close();

#ifdef WIN32
    handle_ = LoadLibrary(TEXT(resolvedPath.string().c_str()));

    if (!handle_) {
        COUT << "Failed to load library: " << resolvedPath.string() << std::endl << std::flush;
        return false;
    }

    COUT << "Loading library: " << resolvedPath.string() << std::endl << std::flush;
    auto* createProducer = (create_t*)GetProcAddress(handle_, "create");

    if (!createProducer)
    {
        DWORD errorMessageID = ::GetLastError();

        LPSTR messageBuffer = nullptr;
        size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, nullptr);

        std::string message(messageBuffer, size);
        COUT << "Error instantiating the library create function: " << resolvedPath.string()
            << ", message: " << message << std::endl << std::flush;
        COUT << "Make sure the `create()` function is defined for your plugin" << std::endl << std::flush;

        // Free the buffer.
        LocalFree(messageBuffer);

        return false;
    }

    destroyProducer_ = (destroy_t*)GetProcAddress(handle_, "destroy");

    if (!destroyProducer_)
    {
        COUT << "Error instantiating the library destroy function: " << resolvedPath.string() << std::endl << std::flush;
        COUT << "Make sure the `destroy(*p)` function is defined for your plugin" << std::endl << std::flush;
        return false;
    }

#else
    handle_ = dlopen(resolvedPath.string().c_str(), RTLD_NOW);

    if (!handle_)
    {
        COUT << "Cannot load library: " << dlerror() << endl;
        return false;
    }

    dlerror();

    create_t* createProducer = (create_t*)dlsym(handle_, "create");

    const char* dlsym_error = dlerror();
    //    if (!dlsym_error)
    //    {
    //        cerr << "Cannot load symbol create: " << dlsym_error << endl;
    //        exit(1);
    //    }
    dlerror();

    destroyProducer_ = (destroy_t*)dlsym(handle_, "destroy");
    dlsym_error = dlerror();

    if (dlsym_error)
    {
        COUT << "Cannot load symbol destroy: " << dlsym_error << '\n';
        return false;
    }
#endif

    plugin_ = createProducer();

    if (!plugin_) {
        COUT << "Error creating a library instance: " << resolvedPath.string() << std::endl << std::flush;
        return false;
    }

    return true;
}

template<class pluginType>
void DynLib<pluginType>::closeDynLib()
{
    destroyProducer_(plugin_);
#ifdef UNIX
    dlclose(handle_);
#endif
#ifdef WIN32
    FreeLibrary(handle_);
#endif
}

template class DynLib<ProducersPluginVirtual>;
template class DynLib<ComsumerPlugin>;
template class DynLib<AngleAndComsumerPlugin>;
template class DynLib<EmgAndAngleAndComsumerPlugin>;
template class DynLib<EMGAndAnglePlugin>;
template class DynLib<ProducersAndConsumerPlugin>;