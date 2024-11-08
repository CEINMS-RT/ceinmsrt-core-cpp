/*
 * Copyright (c) 2015, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
