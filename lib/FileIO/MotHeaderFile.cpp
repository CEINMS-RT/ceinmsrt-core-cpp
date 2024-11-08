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

#include "MotHeaderFile.h"

MotHeaderFile::MotHeaderFile() : inDegress_ ( false ), numberOfRow_ ( 0 ), numberOfColumn_ ( 0 ), numberOfRowBool_(false), numberOfColumnBool_(false)
{

}

std::istream& MotHeaderFile::safeGetline(std::istream& is, std::string& t)
{
	t.clear();

	// The characters in the stream are read one-by-one using a std::streambuf.
	// That is faster than reading them one-by-one using the std::istream.
	// Code that uses streambuf this way must be guarded by a sentry object.
	// The sentry object performs various tasks,
	// such as thread synchronization and updating the stream state.

	std::istream::sentry se(is, true);
	std::streambuf* sb = is.rdbuf();

	for (;;) {
		int c = sb->sbumpc();
		switch (c) {
		case '\n':
			return is;
		case '\r':
			if (sb->sgetc() == '\n')
				sb->sbumpc();
			return is;
		case EOF:
			// Also handle the case when the last line has no line ending
			if (t.empty())
				is.setstate(std::ios::eofbit);
			return is;
		default:
			t += (char)c;
		}
	}
}

void MotHeaderFile::readFile ( std::istream& file, const std::string& fileName )
{
	int cpt = 0;
	std::string line;
	std::string item;
	inDegress_ = true; // .mot files are in degrees by default. An entry on the file header overrides this assignment


	while ( true )
	{
		std::vector<std::string> elems;
		

		safeGetline(file, line);
		
		line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
		line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
		line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
		//line.erase(std::remove(line.begin(), line.end(), ' '), line.end()); Space delimiter for mot files is a space
		line.erase(std::remove(line.begin(), line.end(), 13), line.end());
// 		file >> line;
		std::stringstream ss ( line );

		while ( std::getline ( ss, item, ' ' ) )
		{
			elems.push_back ( item );
		}
		
		// Type 1 header: spaces and "datarows" and "datacolumns" and "inDegrees" entries
		if(elems.size() > 1)
		{
			if ( elems.at ( 0 ) == "datarows" )
			{
				numberOfRow_ = atoi ( elems.at ( 1 ).c_str() );
				numberOfRowBool_ = true;
			}

			if ( elems.at ( 0 ) == "datacolumns" )
			{
				numberOfColumn_ = atoi ( elems.at ( 1 ).c_str() );
				numberOfColumnBool_ = true;
			}

			if ( elems.at ( 0 ) == "inDegrees" )
			{
				if ( elems.at ( 1 ) == "yes" )
					inDegress_ = true;
				else if ( elems.at ( 1 ) == "no" )
					inDegress_ = false;
				else
				{
					COUT << "inDegrees line in file " << fileName << " incorrect. Find " << elems.at ( 1 ) << " only yes or no are accepted." << std::endl;
					exit ( EXIT_FAILURE );
				}
			}
		}

		// Type 2 header: "=" as delimiter and "nRows", "nColumns" and "inDegrees" entries
		elems.clear();
		std::stringstream ss2 ( line );
		while ( std::getline ( ss2, item, '=' ) )
		{
			elems.push_back ( item );
		}
		
		if(elems.size() > 1)
		{
			if ( elems.at ( 0 ) == "nRows" )
			{
				numberOfRow_ = atoi ( elems.at ( 1 ).c_str() );
				numberOfRowBool_ = true;
			}

			if ( elems.at ( 0 ) == "nColumns" )
			{
				numberOfColumn_ = atoi ( elems.at ( 1 ).c_str() );
				numberOfColumnBool_ = true;
			}

			if ( elems.at ( 0 ) == "inDegrees" )
			{
				if ( elems.at ( 1 ) == "yes" )
					inDegress_ = true;
				else if ( elems.at ( 1 ) == "no" )
					inDegress_ = false;
				else
				{
					COUT << "inDegrees line in file " << fileName << " incorrect. Find " << elems.at ( 1 ) << " only yes or no are accepted." << std::endl;
					exit ( EXIT_FAILURE );
				}
			}
		}

		if ( line == "endheader" )
			break;
		
		if(cpt > 100)
		{
			COUT << "endheader not found in file " << fileName << std::endl;
			exit(1);
		}
		
		cpt++;
	}

//  	std::getline ( file, line );
// 	file >> line;
// 	COUT << line << std::endl << std::flush;
	do{
		std::getline ( file, line, '\n' );
	}while(line == "");
	
// 	COUT << line << std::endl << std::flush;

	std::stringstream myStream ( line );
	std::string nextColumnName;
	
	myStream >> nextColumnName;

	nameOfColumn_.clear();

	while ( !myStream.eof() )
	{
		std::getline ( myStream, nextColumnName, '\t' );
		nextColumnName.erase(std::remove(nextColumnName.begin(), nextColumnName.end(), '\r'), nextColumnName.end());
		nextColumnName.erase(std::remove(nextColumnName.begin(), nextColumnName.end(), ' '), nextColumnName.end());
		nextColumnName.erase(std::remove(nextColumnName.begin(), nextColumnName.end(), 13), nextColumnName.end());
		if(!nextColumnName.empty())
			nameOfColumn_.push_back ( nextColumnName );
	}

	if(!numberOfColumnBool_)
	{
		COUT << "Numbers of column not found. Something is wrong in " << fileName << ". the correct formating is nColumns=10 for example." << std::endl << std::flush;
		exit ( EXIT_FAILURE );
	}
	 
	if(!numberOfRowBool_)
	{
		COUT << "Numbers of rows not found. Something is wrong in " << fileName<< ". the correct formating is nRows=10 for example."  << std::endl << std::flush;
		exit ( EXIT_FAILURE );
	}

	if ( numberOfColumn_ - 1 != nameOfColumn_.size() )
	{
		COUT << "Something is wrong in " << fileName << std::endl << numberOfColumn_ - 1 << " column should be in  " << fileName
				<< "and we have : " << nameOfColumn_.size() << std
::endl;

		COUT << "Name of Column: ";
		for ( std::vector<std::string>::const_iterator it = nameOfColumn_.begin(); it != nameOfColumn_.end(); ++it )
			std::cout << *it << " \t";

		std::cout << std::endl;

		exit ( EXIT_FAILURE );
	}

}

void MotHeaderFile::writeFile ( std::ostream& file, const std::string& fileName, const std::string& firstLine )
{
	if ( numberOfColumn_ - 1 != nameOfColumn_.size() ) // -1 for the time
	{
		COUT << "Something is wrong when writing the header for " << fileName << std::endl << numberOfColumn_ - 1 << " column should be in the file "
				<< "and we have : " << nameOfColumn_.size() << std::endl;

		COUT << "Name of Column: ";

		for ( std::vector<std::string>::const_iterator it = nameOfColumn_.begin(); it != nameOfColumn_.end(); ++it )
			std::cout << *it << " \t";

		std::cout << std::endl;

		exit ( EXIT_FAILURE );
	}
	
	if(numberOfRow_ < 0)
	{
		COUT << "Something is wrong when writing the header for " << fileName << std::endl <<  "Negative numbers of rows are in this file." << std::endl;
				
		exit ( EXIT_FAILURE );
	}
	
	if(numberOfColumn_ < 0)
	{
		COUT << "Something is wrong when writing the header for " << fileName << std::endl <<  "Negative numbers of columns are in this file." << std::endl;
				
		exit ( EXIT_FAILURE );
	}
	
	file << firstLine << std::endl;
	file << "nRows=" << numberOfRow_ << std::endl;
	file << "nColumns=" << numberOfColumn_ << std::endl;
	file << "endheader" << std::endl;
	if(inDegress_)
		file << "inDegrees=yes" << std::endl;
	file << "time\t";
	for ( std::vector<std::string>::const_iterator it = nameOfColumn_.begin(); it != nameOfColumn_.end(); it++ )
		file << *it << "\t";

	file << std::endl;
}
