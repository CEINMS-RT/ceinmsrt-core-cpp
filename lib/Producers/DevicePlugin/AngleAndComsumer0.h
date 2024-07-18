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
