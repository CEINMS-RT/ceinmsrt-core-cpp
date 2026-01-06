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

#ifndef COMPUTEPOSTURECONTROLPOINTS_H
#define COMPUTEPOSTURECONTROLPOINTS_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <boost/shared_ptr.hpp>
#include "NMSmodel.hxx"
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>

struct MuscleTendonScalingStruct
{
	std::string muscleName;
	std::vector<std::string> spanningDOF;
	std::vector<std::vector<double> > postureControlPoints;
	std::vector<double> unscaledMuscleTendonLength;
	std::vector<double> scaledMuscleTendonLength;
	std::vector<double> fiberLength;
	double optimalPennationAngle;
	double unscaledTendonSlackLength;
	double scaledTendonSlackLength;
	double unscaledOptimalFiberLength;
	double scaledOptimalFiberLength;
};

typedef std::vector<MuscleTendonScalingStruct> MTSS;

class ComputePostureControlPoints
{
	public:
		ComputePostureControlPoints ( const std::string& xmlFile, const OpenSim::Model& unscaledModel, MTSS& mtss, const std::string& translateFile );
		~ComputePostureControlPoints();
		
		/**
		 * Return the max angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMinAnglesDof ( const std::string& nameOfDOF ) const;

		/**
		 * Return the min angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMaxAnglesDof ( const std::string& nameOfDOF ) const;
		
	protected:
		typedef boost::bimap < boost::bimaps::multiset_of<std::string>, boost::bimaps::multiset_of<std::string> > DOFMuscleMap;
		typedef DOFMuscleMap::value_type position;
		const OpenSim::Model& _unscaledModel;
};

#endif

#endif // COMPUTEPOSTURECONTROLPOINTS_H
