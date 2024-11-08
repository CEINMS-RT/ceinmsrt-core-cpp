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

#ifndef XMLINTERPRETER_H_
#define XMLINTERPRETER_H_

#include "executionIK_ID.hxx"
#include <iostream>

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

/**
 * Class for reading the lab.xsd type xml and extract information for the IK and ID.
 */

class XMLInterpreter {
public:

	/**
	 * Constructor
	 */
	XMLInterpreter(const std::string& xmlFileName);

	/*
	 * Destructor
	 */
	virtual ~XMLInterpreter();

	/*
	 * enumerator for the type of IK.
	 */
	enum IKType
	{
		Marker,
		IMU
	};

	/**
	 * Read the XML file and extract information.
	 */
	void readXML();

	/**
	 * Get if we use the inverse dynamics.
	 */
	inline const bool& getUseID() const
	{
		return idUse_;
	}

	/*
	 * Get the marker name that we want to use for the IK.
	 */
	inline const std::vector<std::string>& getMarkersNames() const
	{
		return markerNames_;
	}

	/**
	 * Get the weigth of the marker.
	 */
	inline const std::vector<double>& getMarkersWeights() const
	{
		return markerWeights_;
	}

	/**
	 * Get the IMU names.
	 */
	inline const std::vector<std::string>& getImuNames() const
	{
		return imuNames_;
	}

	/**
	 * Get the bodies name correspondence for the IMU.
	 */
	inline const std::vector<std::string>& getImuBodies() const
	{
		return imuBodies_;
	}

	/**
	 * Get the maximum error for the IK using markers.
	 */
	inline const double& getMaxMarkerError() const
	{
		return maxMarkerError_;
	}

	/**
	 * Get the IK type.
	 * @return Enumerator for the IK type.
	 */
	inline const IKType& GetIKType() const
	{
		return ikType_;
	}

	/**
	 * Get if we enforced constraint for respecting the limit.
	 */
	inline const bool& getEnforceConstraintUse() const
	{
		return enfConstUse_;
	}

	/**
	 * Get if we use the kalman filter for filtering the position and compute the velocity and acceleration.
	 */
	inline const bool& getKalmanUse() const
	{
		return kalmanUse_;
	}

	/**
	 * Get the R parameters for th eKalman filter.
	 */
	inline const double& getR()
	{
		return r_;
	}

	/**
	 * Get the P parameters for th eKalman filter.
	 */
	inline const double& getP()
	{
		return p_;
	}

	/**
	 * Get the SigmaDa parameters for th eKalman filter.
	 */
	inline const double& getSigmaDa()
	{
		return sigma_da_;
	}

	/**
	 * Get the dt parameters for th eKalman filter.
	 */
	inline const double& getDt()
	{
		return dt_;
	}

	/**
	 * Get if we use the ground reaction force for the computation of the inverse dynamics.
	 */
	inline const bool& getExternalLoadUse()
	{
		return externalLoadUse_;
	}

	/**
	 * Get the name of the lab file xml2
	 */
	inline const std::string& getLabFile()
	{
		return labFile_;
	}

	/**
	 * Get the Osim file name.
	 */
	inline const std::string& getOsimFile()
	{
		return osimFile_;
	}

	/**
	 * Get the translation file name.
	 */
	inline const std::string& getTranslateFile()
	{
		return translateFile_;
	}

	/**
	 * Get the IP for the motion capture system.
	 */
	inline const std::string& getIP()
	{
		return ip_;
	}

	/**
	 * Get the port for the motion capture system.
	 */
	inline const int& getPort()
	{
		return port_;
	}

	/**
	 * Get the name of the body on which the ground reaction force will be applied.
	 */
	inline const std::vector<std::string>& getAppliedBody()
	{
		return appliedBody_;
	}
	
	inline const std::vector<double>& getACoeffMarker()
	{
		return _aCoeffMarker;
	}
	
	inline const std::vector<double>& getBCoeffMarker()
	{
		return _bCoeffMarker;
	}
	
	inline const std::vector<double>& getACoeffGrf()
	{
		return _aCoeffGrf;
	}
	
	inline const std::vector<double>& getBCoeffGrf()
	{
		return _bCoeffGrf;
	}
	
	inline const int& getNumberofThreadForIK()
	{
			return _numberOfThread;
	}

	inline const std::string& getIkTaskFilename()
	{
		return ikTaskFilename_;
	}

	inline const int& getFc()
	{
		return fc_;
	}

	inline const std::string& getExternalLoadsXml()
	{
		return externalLoadsXml_;
	}
protected:
	bool idUse_;
	bool ikMarkerUse_;
	bool enfConstUse_;
	bool kalmanUse_;
	bool externalLoadUse_;
	double r_; //!< Kalman parameter
	double p_; //!< Kalman parameter
	double sigma_da_; //!< Kalman parameter
	double dt_; //!< Kalman parameter
	int fc_; //!< low pass filter cut off for IK and ID
	std::auto_ptr<ExecutionIKType> ptr_execution_; //!< pointer to the XML file class
	std::vector<std::string> markerNames_; //!< Vector to the markers names that we use for the iK
	std::vector<double> markerWeights_; //!< vector of the marker weight
	std::vector<std::string> imuNames_; //!< Vector of the names of the IMU
	std::vector<std::string> imuBodies_; //!< Vector of the body name correspondent to the IMU
	double maxMarkerError_; //!< Maximum error for the IK using marker
	IKType ikType_; //!< Enumerator of the IK type use
	std::string labFile_; //!< Name of the lab file XML
	std::string translateFile_; //!< Name of the translation file
	std::string osimFile_; //!< Name of the Osim file
	std::string ip_; //!< IP for the motion capture system
	std::string ikTaskFilename_; //!< osim XML filenema for the IK task
	std::string externalLoadsXml_; //!<  osim XML filename for ID load
	std::vector<std::string> appliedBody_; //!< Name of the body on which the ground reaction force is applied
	int port_; //!< Port for the motion capture system
	std::vector<double> _aCoeffMarker;
	std::vector<double> _bCoeffMarker;
	std::vector<double> _aCoeffGrf;
	std::vector<double> _bCoeffGrf;
	int _numberOfThread;
};

#endif /* XMLINTERPRETER_H_ */
