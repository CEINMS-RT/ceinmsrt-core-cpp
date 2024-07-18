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

#include "CLIOption.h"

CLIOption::CLIOption(const int& argc, char** argv):
_record(false), _recordFilePath("output"), _time(false)
{
	boost::program_options::options_description desc ( "Options" );
	desc.add_options()
	( "help,h,H", "Output a small usage guide and exit successfully. No other output is generated." )
	( "executionIK,i,I", boost::program_options::value<std::string> ( &_xmlIKFilePath ), "XML file for the inverse kinematic. The format can be found in the XSD directory." )
	( "executionEMG,e,E", boost::program_options::value<std::string> ( &_xmlEMGFilePath ), "XML file for the EMG processing. The format can be found in the XSD directory." )
	( "subject,s,S", boost::program_options::value<std::string> ( &_xmlSubjectFilePath ), "XML file for the subject specific model. The format can be found in the XSD directory." )
	( "record,r,R", boost::program_options::value<std::string> ( &_recordFilePath ), "Save the output in a directory. The name of the directory is arg (string)." )
	( "printTime,p,P", "Print the time between two frames." );
	boost::program_options::variables_map vm;
	boost::program_options::store ( boost::program_options::parse_command_line ( argc, argv, desc ), vm );
	boost::program_options::notify ( vm );
	
	if ( vm.count ( "help" ) )
	{
		std::cout << desc << std::endl;;
		exit(1);
	}

// 	if ( !vm.count ( "executionEMG" ) )
// 	{
// 		std::cout << "Need --executionEMG option." << std::endl;
// 		std::cout << desc << std::endl;;
// 		exit(1);
// 	}
	
	if ( !vm.count ( "executionIK" ) )
	{
		std::cout << "Need --executionIK option." << std::endl;
		std::cout << desc << std::endl;;
		exit(1);
	}
	
// 	if ( !vm.count ( "subject" ) )
// 	{
// 		std::cout << "Need --subject option." << std::endl;
// 		std::cout << desc << std::endl;;
// 		exit(1);
// 	}


	if ( vm.count ( "record" ) )
		_record = true;
	
	if ( vm.count ( "printTime" ) )
		_time = true;
}

CLIOption::~CLIOption()
{

}
