/*
 * DynLibBase.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#include "DynLibOptimization.h"

template <typename NMSmodelT>
DynLibOptimization<NMSmodelT>::~DynLibOptimization()
{

}

template <typename NMSmodelT>
void DynLibOptimization<NMSmodelT>::setDynLib(const string& libpath)
{
#ifdef UNIX
	handle_ = dlopen ( libpath.c_str(), RTLD_NOW );

	if ( !handle_ )
	{
		COUT << "Cannot load library: " << dlerror() << endl;
		exit ( 1 );
	}

	createDestroySpecialization(libpath);
#endif
#ifdef WINDOWS
	handle_ = LoadLibrary(TEXT(libpath.c_str()));
	
	if (handle_ != NULL)
    { 
		createDestroySpecialization(libpath);
		COUT << "load library: " << libpath << std::endl;
	}
	else
	{
		COUT << "Cannot load library: " << libpath << endl;
		DWORD errorMessageID = ::GetLastError();

		LPSTR messageBuffer = nullptr;
		size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

		std::string message(messageBuffer, size);
		COUT << message << " Error loading library: " << libpath << std::endl << std::flush;
		//Free the buffer.
		LocalFree(messageBuffer);

		std::cout << "TEXT(libpath.c_str()): " << TEXT(libpath.c_str()) << std::endl;
		std::cout << "libpath.c_str(): " << libpath.c_str() << std::endl;
		std::cout << "handle_: " << handle_ << std::endl;
	}
#endif


}

template <typename NMSmodelT>
void DynLibOptimization<NMSmodelT>::createDestroySpecialization(std::string libpath)
{
	std::cout << "template: " << typeid(NMSmodelT).name() << " not implemneted." << std::endl;
	exit(0);
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >::createDestroySpecialization(std::string libpath)
{
#ifdef WINDOWS
	create_cEAS* createProducer = (create_cEAS*)GetProcAddress(handle_, "createEAS");

	if (NULL == createProducer)
	{

		DWORD errorMessageID = ::GetLastError();

		LPSTR messageBuffer = nullptr;
		size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

		std::string message(messageBuffer, size);
		COUT << message << " Error instanciation library: " << libpath << std::endl << std::flush;
		//Free the buffer.
		LocalFree(messageBuffer);
	}

	destroyProducerEAS_ = (destroy_cEAS*)GetProcAddress(handle_, "destroyEAS");
	if (NULL == destroyProducerEAS_)
		COUT << "Error destroy function loading library: " << libpath << std::endl << std::flush;


#endif
#ifdef UNIX
	create_cEAS* createProducer = (create_cEAS*)dlsym(handle_, "createEAS");
	destroyProducerEAS_ = (destroy_cEAS*)dlsym(handle_, "destroyEAS");
#endif
	plugin_ = createProducer();
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> >::createDestroySpecialization(std::string libpath)
{
#ifdef WINDOWS
	create_cEASoff* createProducer = (create_cEASoff*)GetProcAddress(handle_, "createEASoff");

	if (NULL == createProducer)
	{

		DWORD errorMessageID = ::GetLastError();

		LPSTR messageBuffer = nullptr;
		size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

		std::string message(messageBuffer, size);
		COUT << message << " Error instanciation library: " << libpath << std::endl << std::flush;
		//Free the buffer.
		LocalFree(messageBuffer);
	}

	destroyProducerEASoff_ = (destroy_cEASoff*)GetProcAddress(handle_, "destroyEASoff");
	if (NULL == destroyProducerEASoff_)
		COUT << "Error destroy function loading library: " << libpath << std::endl << std::flush;


#endif
#ifdef UNIX
	create_cEASoff* createProducer = (create_cEASoff*)dlsym(handle_, "createEASoff");
	destroyProducerEASoff_ = (destroy_cEASoff*)dlsym(handle_, "destroyEASoff");
#endif
	plugin_ = createProducer();
}


template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >::createDestroySpecialization(std::string libpath)
{
#ifdef WINDOWS
	create_cEAEB* createProducer = (create_cEAEB*)GetProcAddress(handle_, "createEAEB");

	if (NULL == createProducer)
	{

		DWORD errorMessageID = ::GetLastError();

		LPSTR messageBuffer = nullptr;
		size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

		std::string message(messageBuffer, size);
		COUT << message << " Error instanciation library: " << libpath << std::endl << std::flush;
		//Free the buffer.
		LocalFree(messageBuffer);
	}
	destroyProducerEAEB_ = (destroy_cEAEB*)GetProcAddress(handle_, "destroyEAEB");
	if (NULL == destroyProducerEAEB_)
		COUT << "Error destroy function loading library: " << libpath << std::endl << std::flush;

#endif
#ifdef UNIX
	create_cEAEB* createProducer = (create_cEAEB*)dlsym(handle_, "createEAEB");
	destroyProducerEAEB_ = (destroy_cEAEB*)dlsym(handle_, "destroyEAEB");
#endif
	plugin_ = createProducer();
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >::createDestroySpecialization(std::string libpath)
{
#ifdef WINDOWS
	create_cEAE* createProducer = (create_cEAE*)GetProcAddress(handle_, "createEAE");
	if (NULL == createProducer)
	{

		DWORD errorMessageID = ::GetLastError();

		LPSTR messageBuffer = nullptr;
		size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

		std::string message(messageBuffer, size);
		COUT << message << " Error instanciation library: " << libpath << std::endl << std::flush;
		//Free the buffer.
		LocalFree(messageBuffer);
	}
	destroyProducerEAE_ = (destroy_cEAE*)GetProcAddress(handle_, "destroyEAE");
	if (NULL == destroyProducerEAE_)
		COUT << "Error destroy function loading library: " << libpath << std::endl << std::flush;

#endif
#ifdef UNIX
	create_cEAE* createProducer = (create_cEAE*)dlsym(handle_, "createEAE");
	destroyProducerEAE_ = (destroy_cEAE*)dlsym(handle_, "destroyEAE");
#endif
	plugin_ = createProducer();
}

template <typename NMSmodelT>
void DynLibOptimization<NMSmodelT>::closeDynLib()
{
	destroyPlugin();

#ifdef UNIX
	dlclose ( handle_ );
#endif
#ifdef WINDOWS
	FreeLibrary(handle_);
#endif
	delete plugin_;
}

template <typename NMSmodelT>
void DynLibOptimization<NMSmodelT>::destroyPlugin()
{
	std::cout << "template: " << typeid(NMSmodelT).name() << " not implemneted." << std::endl;
	exit(0);
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >::destroyPlugin()
{
	destroyProducerEAS_(plugin_);
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> >::destroyPlugin()
{
	destroyProducerEASoff_(plugin_);
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >::destroyPlugin()
{
	destroyProducerEAEB_(plugin_);
}

template <>
void DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >::destroyPlugin()
{
	destroyProducerEAE_(plugin_);
}

template class DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class DynLibOptimization<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> >;
template class DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class DynLibOptimization<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;