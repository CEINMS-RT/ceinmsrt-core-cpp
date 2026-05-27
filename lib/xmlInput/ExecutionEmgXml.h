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

#ifndef EXECUTIONEMGXML_H
#define EXECUTIONEMGXML_H

#include "executionEMG.hxx"
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

class ExecutionEmgXml
{
	public:
		ExecutionEmgXml ( const std::string& fileName );
		~ExecutionEmgXml();

		inline const std::string& getIP() const
		{
			return _ip;
		}

		inline const std::string& getPort() const
		{
			return _port;
		}

		inline const std::vector<double>& getACoeffHP() const
		{
			return _aCoeffHP;
		}

		inline const std::vector<double>& getBCoeffHP() const
		{
			return _bCoeffHP;
		}

		inline const std::vector<double>& getACoeffLP() const
		{
			return _aCoeffLP;
		}

		inline const std::vector<double>& getBCoeffLP() const
		{
			return _bCoeffLP;
		}

		inline const std::vector<double>& getMaxEmg() const
		{
			return _maxAmp;
		}

		inline void setMaxEmg ( const std::vector<double>& maxEmg )
		{
			_maxAmp = maxEmg;
		}

		void writeEmgXmlFile ( const std::string& fileName );

		void UpdateEmgXmlFile ();

	protected:
		std::string _ip;
		std::string _port;
		std::vector<double> _aCoeffHP;
		std::vector<double> _bCoeffHP;
		std::vector<double> _aCoeffLP;
		std::vector<double> _bCoeffLP;
		std::vector<double> _maxAmp;
		
		std::string _filename;
		std::auto_ptr<ExecutionEMGType> _executionPointer;

};

#endif // EXECUTIONEMGXML_H
