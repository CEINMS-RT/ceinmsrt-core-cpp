// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "XMLInterpreter.h"

XMLInterpreter::XMLInterpreter ( const std::string& xmlFileName ) :
	idUse_ ( false ), ikMarkerUse_ ( false ), maxMarkerError_ ( 0.001 ), enfConstUse_ ( false ), r_ ( 1 ), p_ ( 1 ), sigma_da_ ( 1 ), externalLoadUse_ ( false )
{
	try
	{
#ifdef DONT_INITIALIZE
		std::auto_ptr<ExecutionIKType> executionPointer ( executionIK ( xmlFileName ) );
#pragma message ( "XML file open with xml_schema::flags::dont_initialize" )
#endif
#ifndef DONT_INITIALIZE
		std::auto_ptr<ExecutionIKType> executionPointer ( executionIK ( xmlFileName ) );
#endif
		ptr_execution_ = executionPointer;
	}
	catch ( const xml_schema::exception& e )
	{
		std::cout << e << std::endl;
		exit ( EXIT_FAILURE );
	}
}

void XMLInterpreter::readXML()
{
	try
	{
		const ExecutionIKType::id_type& idType = ptr_execution_->id();

		// get if we use the ID.
		if(idType.use())
			idUse_ = idType.use().get();
		else
			idUse_ = true;
		
		if(idUse_)
		{
			_aCoeffGrf = idType.lpFilterGRF().aCoeff();
			_bCoeffGrf = idType.lpFilterGRF().bCoeff();
		}

// 		std::cout << idType.plateForceBody().size() << std::endl;
		// Get the name of the body on which the ground reaction force is applied.
		for ( int i = 0; i < idType.plateForceBody().size(); i++ )
		{
// 			std::cout << idType.plateForceBody().at(i) << std::endl;
			appliedBody_.push_back ( idType.plateForceBody().at ( i ) );
		}

		const ExecutionIKType::ik_type::ikOption_type& ikOptionType = ptr_execution_->ik().ikOption();

		// Get if we enforced the IK constraint (joint limits).
		if(ikOptionType.enforceIKConstraintUse())
			enfConstUse_ = ikOptionType.enforceIKConstraintUse().get();
		else
			enfConstUse_ = false;
		
		if(ikOptionType.numberOfThread())
			_numberOfThread = (int)ikOptionType.numberOfThread().get();
		else
			_numberOfThread = 3;

		// Get if we use the kalman filter.
		if(ikOptionType.kalman().use())
			kalmanUse_ = ikOptionType.kalman().use().get();
		else
			kalmanUse_ = true;

		// Get the kalman filter parameters.
		if ( kalmanUse_ )
		{
			if(ikOptionType.kalman().kalmanOption().r())
				r_ = ikOptionType.kalman().kalmanOption().r().get();
			else
				r_ = 0.00001;
			if(ikOptionType.kalman().kalmanOption().p())
				p_ = ikOptionType.kalman().kalmanOption().p().get();
			else
				p_ = 5;
			if(ikOptionType.kalman().kalmanOption().sigma_da())
				sigma_da_ = ikOptionType.kalman().kalmanOption().sigma_da().get();
			else
				sigma_da_ = 50000;
			dt_ = ikOptionType.kalman().kalmanOption().dt();
		}

		const ExecutionIKType::ik_type& ikType = ptr_execution_->ik();

		// Get if we use the IMU type IK and get the parameters.
		if ( ikType.dataFrom().imus() )
		{
			ikType_ = IMU;
			typedef ExecutionIKType::ik_type::dataFrom_type::imus_type::imu_sequence IMUSeq;
			const IMUSeq& imuType = ikType.dataFrom().imus().get().imu();

			for ( IMUSeq::const_iterator it1 = imuType.begin();
					it1 != imuType.end(); it1++ )
			{
				imuNames_.push_back ( it1->name() );
				imuBodies_.push_back ( it1->body() );
			}

			// Get if we use the markers type IK and get the parameters.
		}
		else if ( ikType.dataFrom().markers().present() )
		{
			ikType_ = Marker;
			_aCoeffMarker = ikType.dataFrom().markers().get().lpFilterMarker().aCoeff();
			_bCoeffMarker = ikType.dataFrom().markers().get().lpFilterMarker().bCoeff();
			if(ikType.dataFrom().markers().get().maxError())
				maxMarkerError_ = ikType.dataFrom().markers().get().maxError().get();
			else
				maxMarkerError_ = 0.00001;
			typedef ExecutionIKType::ik_type::dataFrom_type::markers_type::markersList_type::marker_sequence MarkerSeq;
			const MarkerSeq& markerType = ikType.dataFrom().markers().get().markersList().marker();

			for ( MarkerSeq::const_iterator it1 = markerType.begin();
					it1 != markerType.end(); it1++ )
			{
				markerNames_.push_back ( it1->name() );
				markerWeights_.push_back ( it1->weight() );
			}
		}

		// Get the filename of the lab xml.
		fs::path fsLibPath = std::string(ptr_execution_->LabFile());
		fs::path resolvedPath;
		if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
			resolvedPath = fsLibPath;
		else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
			resolvedPath = fs::absolute(std::string(ptr_execution_->LabFile()));
		else
			std::cerr << "File: " << fsLibPath << " does not exist or is not found." << std::endl;
		labFile_ = resolvedPath.string();

		// Get the filename of the translation file
		fsLibPath = std::string(ptr_execution_->TranslateFile());
		if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
			resolvedPath = fsLibPath;
		else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
			resolvedPath = fs::absolute(std::string(ptr_execution_->TranslateFile()));
		else
			std::cerr << "File: " << fsLibPath << " does not exist or is not found." << std::endl;
		translateFile_  = resolvedPath.string();

		// Get the filename of the Osim file.
		fsLibPath = std::string(ptr_execution_->OsimFile());
		if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
			resolvedPath = fsLibPath;
		else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
			resolvedPath = fs::absolute(std::string(ptr_execution_->OsimFile()));
		else
			std::cerr << "File: " << fsLibPath << " does not exist or is not found." << std::endl;
		osimFile_ = resolvedPath.string();

		// Get the IP for the motion capture system.
		ip_ = ptr_execution_->ip();

		// Get the port for the motion capture system.
		port_ = ptr_execution_->port();

		fc_ = ptr_execution_->Fc().get();


		fsLibPath = std::string(ptr_execution_->externalLoadsXml());
		if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
			resolvedPath = fsLibPath;
		else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
			resolvedPath = fs::absolute(std::string(ptr_execution_->externalLoadsXml()));
		else
			std::cerr << "File: " << fsLibPath << " does not exist or is not found." << std::endl;
		externalLoadsXml_ = resolvedPath.string();


		fsLibPath = std::string(ptr_execution_->ikTaskFilename());
		if (fsLibPath.is_absolute())   // If path is absolute, use it as it is
			resolvedPath = fsLibPath;
		else if (fs::exists(fsLibPath))         // If it is relative, check if it exists as listed in libpath
			resolvedPath = fs::absolute(std::string(ptr_execution_->ikTaskFilename()));
		else
			std::cerr << "File: " << fsLibPath << " does not exist or is not found." << std::endl;
		ikTaskFilename_ = resolvedPath.string();


	}
	catch ( const xml_schema::exception& e )
	{
		std::cout << e << std::endl;
		exit ( EXIT_FAILURE );
	}

}

XMLInterpreter::~XMLInterpreter()
{
}

