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

#ifndef ANGLEANDCOMSUMER0_H
#define ANGLEANDCOMSUMER0_H

#include <AngleAndComsumerPlugin.h>

class AngleAndComsumer0: public AngleAndComsumerPlugin
{
	public:
		AngleAndComsumer0();
		~AngleAndComsumer0();
		
		void init(std::string& executableXMLFileName)
		{
			
		}
		
		void setDofName ( const std::vector<std::string>& dofName )
		{
			dofName_ = dofName;
		}
		
		void setDofTorque ( const std::vector<double>& dofTorque )
		{
			
		}
		
		void setDofStiffness ( const std::vector<double>& dofStiffness )
		{
			
		}
		
		void setMuscleName ( const std::vector<std::string>& muscleName )
		{
			
		}


		virtual void setMuscleForce(const std::vector<double>& muscleForce)
		{

		}

		virtual void setMuscleFiberLength(const std::vector<double>& muscleFiberLength)
		{

		}

		virtual void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity)
		{

		}

		void setMuscleForcePassive(const std::vector<double>& muscleForcePassive)
		{

		}

		void setMuscleForceActive(const std::vector<double>& muscleForceActive)
		{

		}
		
		void setTendonStrain(const std::vector<double>& tendonStrain)
		{
			
		}
		
		void setOutputTimeStamp ( const double& timeStamp )
		{
			
		}
		
		const double& GetAngleTimeStamp()
		{
			return timeStamp_;
		}
		
		const std::vector<std::string>& GetDofName()
		{
			return dofName_;
		}
		
		const std::map<std::string, double>& GetDataMap();
		
		const std::map<std::string, double>& GetDataMapTorque();
		
		void stop()
		{
			
		}

		void setDirectory ( std::string outDirectory, std::string inDirectory = std::string() )
		{
		}

		void setVerbose ( int verbose )
		{
		}

		void setRecord ( bool record )
		{
		}
		
protected:
	std::vector<std::string> dofName_;
	double timeStamp_;
	
	std::map<std::string, double> dataAngle_;
	std::map<std::string, double> dataTorque_;
};

#endif // ANGLEANDCOMSUMER0_H
