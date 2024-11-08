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

#ifndef CLIOPTION_H
#define CLIOPTION_H
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>
#include "boost/program_options.hpp"

class CLIOption
{
	public:
		CLIOption(const int& argc, char** argv);
		~CLIOption();

		inline const bool& getRecord() const
		{
			return _record;
		}

		inline const std::string& getRecordFilePath() const
		{
			return _recordFilePath;
		}

		inline const std::string& getxmlIKFilePath() const
		{
			return _xmlIKFilePath;
		}
		
		inline const bool& getPrintTime() const
		{
			return _time;
		}
		
		inline const std::string& getxmlEMGFilePath() const
		{
			return _xmlEMGFilePath;
		}
		
		inline const std::string& getxmlSubjectFilePath() const
		{
			return _xmlSubjectFilePath;
		}

	protected:
		std::string _recordFilePath;
		bool _record;
		std::string _xmlIKFilePath;
		bool _time;
		std::string _xmlEMGFilePath;
		std::string _xmlSubjectFilePath;
};

#endif // CLIOPTION_H
