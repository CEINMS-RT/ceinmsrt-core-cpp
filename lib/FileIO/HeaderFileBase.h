/*
 * Copyright (c) 2016, <copyright holder> <email>
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

#ifndef HEADERFILE_BASE_H_
#define HEADERFILE_BASE_H_

#include <CommonCEINMS.h>
#include <fstream>
#include <sstream>

class HeaderFileBase
{
	public:
		HeaderFileBase(){}
		virtual void readFile(std::istream& file, const std::string& fileName = "file") = 0;
		virtual void writeFile(std::ostream& file, const std::string& fileName = "file", const std::string& firstLine = "default") = 0;
		virtual void setNumberOfRow(const unsigned int& numberOfRow) = 0;
		virtual const unsigned int& getNumberOfRow() const = 0;
		virtual void setNumberOfColumn(const unsigned int& numberOfColumn) = 0;
		virtual const unsigned int& getNumberOfColumn() const = 0;
		virtual void setNameOfColumn(const std::vector<std::string>& nameOfColumn) = 0;
		virtual const std::vector<std::string>& getNameOfColumn() const = 0;
		virtual void setInDegrees(const bool& inDegress) = 0;
		virtual const bool& getInDegrees() const = 0;
	protected:
		virtual std::istream& safeGetline(std::istream& is, std::string& t) = 0;

		~HeaderFileBase(){}
};

#endif // HEADERFILE_H
