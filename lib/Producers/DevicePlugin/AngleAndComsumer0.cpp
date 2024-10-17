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

#include "AngleAndComsumer0.h"

AngleAndComsumer0::AngleAndComsumer0(): timeStamp_(0)
{

}

AngleAndComsumer0::~AngleAndComsumer0()
{

}

const std::map<std::string, double>& AngleAndComsumer0::GetDataMap()
{
	for(std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		dataAngle_[*it] = 0.;
	
	timeStamp_ += 0.01;
	
	return dataAngle_;
}
		
const std::map<std::string, double>& AngleAndComsumer0::GetDataMapTorque()
{
	for(std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		dataTorque_[*it] = 0.;
	
	return dataTorque_;
}

extern "C" AngleAndComsumerPlugin* create() {
    return new AngleAndComsumer0;
}

extern "C" void destroy(AngleAndComsumerPlugin* p) {
    delete p;
}