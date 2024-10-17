// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "MTUSplineDataRead.h"
#include <boost/timer/timer.hpp>
// #define PRINT_DOF_LIST

#define PRECISION 6
#define LENGTH 8048
/*
class DimException: public exception
{
  virtual const char* what() const throw()
  {
    return "The dimension is bigger than 8 which is the maximum accepted.";
  }
} dimException2;*/

MTUSplineDataRead::MTUSplineDataRead(): MTUSplineData("Subject1")
{
	// TODO Auto-generated constructor stub

}

MTUSplineDataRead::MTUSplineDataRead(string& configurationFile, const string& subjectName): MTUSplineData(subjectName)
{
	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(configurationFile));
		NMSmodelType::muscles_type& muscles(subjectPointer->muscles());
		MusclesType::muscle_sequence& muscleSequence(muscles.muscle());
		for (MusclesType::muscle_iterator i(muscleSequence.begin()); i != muscleSequence.end(); ++i)
			vectMuscleNameList_.push_back((*i).name());

		// DOF iteration
		NMSmodelType::DoFs_type& dofs(subjectPointer->DoFs());
		DoFsType::DoF_sequence& dofSequence(dofs.DoF());
		for (DoFsType::DoF_iterator i = dofSequence.begin(); i != dofSequence.end(); ++i)
		{
			string currentDOF((*i).name());
			vectDOFNameList_.push_back(currentDOF);

			// iteration in the muscle containing in each DOF. See end of subject specific XML.
			MuscleSequenceType currentSequence = (*i).muscleSequence();
			MuscleSequenceType::iterator muscleIt;
			for (muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt)
			{
				string currentMuscle(*muscleIt);
				DOFMuscleConnect_.insert(position(currentDOF, currentMuscle));
			}
		}
	} catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

	// Create unique list of unique DOF name list that is cross by the same muscle.
	multimap<set<string>, string> DOFsetToMuscle;
	set<set<string> > uniqueDOFlist;
	int maxSizeSet;
	for (vector<string>::const_iterator it = vectMuscleNameList_.begin(); it != vectMuscleNameList_.end();
			it++)
	{
		string muscleName(*it);
		set<string> temp;
		for (DOFMuscleMap::right_const_iterator itBimap = DOFMuscleConnect_.right.lower_bound(muscleName);
				itBimap != DOFMuscleConnect_.right.upper_bound(muscleName); itBimap++)
			temp.insert(itBimap->second);
		DOFsetToMuscle.insert(pair<set<string>, string>(temp, *it));
		uniqueDOFlist.insert(temp);
	}

	vectTask_.resize(uniqueDOFlist.size());
	for (set<set<string> >::const_iterator it1 = uniqueDOFlist.begin(); it1 != uniqueDOFlist.end(); it1++)
	{
		vectTask_[distance<set<set<string> >::const_iterator>(uniqueDOFlist.begin(), it1)].uniqueDOFlist.insert((*it1).begin(), (*it1).end());
		for(multimap<set<string>, string>::const_iterator it2 = DOFsetToMuscle.lower_bound(*it1); it2 != DOFsetToMuscle.upper_bound(*it1); it2++)
			vectTask_[distance<set<set<string> >::const_iterator>(uniqueDOFlist.begin(), it1)].uniqueMuscleList.push_back(it2->second);
	}

	// print the groups of DOF.

	vector<Task> takVectTemp;
	for (vector<Task>::iterator it1 = vectTask_.begin(); it1 != vectTask_.end(); it1++)
		if((*it1).uniqueDOFlist.size() != 0)
			takVectTemp.push_back(*it1);
	vectTask_ = takVectTemp;

#ifdef PRINT_DOF_LIST
	cout << "Group of DOF found:" << endl;
	for (vector<Task>::iterator it1 = vectTask_.begin(); it1 != vectTask_.end(); it1++)
	{
		for (set<string>::iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end(); it2++)
			cout << *it2 << " ";
		cout << ": ";
		for (set<string>::iterator it2 = (*it1).uniqueMuscleList.begin(); it2 != (*it1).uniqueMuscleList.end(); it2++)
			cout << *it2 << " ";
		cout << endl;
	}
	cout << endl;
#endif


}

MTUSplineDataRead::~MTUSplineDataRead()
{
	// TODO Auto-generated destructor stub
}

void MTUSplineDataRead::readTaskCoefficients()
{
//	cout << "Begin to read coefficients...";
	for (vector<Task>::iterator it1 = vectTask_.begin(); it1 != vectTask_.end(); it1++)
	{
		stringstream temp_str;
		temp_str<<(distance<vector<Task>::iterator>(vectTask_.begin(), it1));
		string temp = temp_str.str();
		string filename("cfg/SplineCoeff/" + subjectName_ + "_Coefficients_" + temp + ".bin");

		std::vector<double> coeff;
		coeff.resize(1);
		float sum = 0;

		FILE *inputCoeffFile = NULL;
		inputCoeffFile = fopen(filename.c_str(), "rb");
		//ifstream inputCoeffFile(filename.c_str(), std::ifstream::binary);

		//if (!inputCoeffFile.is_open())
		if (inputCoeffFile == NULL)
		{
			cout << "ERROR: " << filename << " could not be open\n";
			exit(EXIT_FAILURE);
		}

		{
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
		}
		{
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
			//getline(inputCoeffFile, line, '\n');
			//getline(inputCoeffFile, line, '\n');

			string line(lineChar);
			stringstream myStream(line);
			int nbOfDOF;
			myStream >> subjectName_;
			myStream >> nbOfDOF;
			myStream >> noMuscles_;

			//std::cout << subjectName_ << std::endl;
			//std::cout << nbOfDOF << std::endl;
			//std::cout << noMuscles_ << std::endl;

			char lineChar2[LENGTH];
			fgets(lineChar2, LENGTH, inputCoeffFile);
			//getline(inputCoeffFile, line, '\n'); //Blank line

			for (int i = 0; i < nbOfDOF; ++i)
			{
				char lineChar3[LENGTH];
				fgets(lineChar3, LENGTH, inputCoeffFile);
				string lineb(lineChar3);
				stringstream myStream(lineb);
				myStream >> dofName_[i];
				myStream >> std::fixed >> std::setprecision(PRECISION) >> a_[i];
				
				myStream >> std::fixed >> std::setprecision(PRECISION) >> b_[i];
				myStream >> n_[i];
				/*inputCoeffFile >> dofName_[i];
				inputCoeffFile >> std::fixed >> std::setprecision(PRECISION) >> a_[i];
				inputCoeffFile >> std::fixed >> std::setprecision(PRECISION) >> b_[i];
				inputCoeffFile >> n_[i];*/
				//getline(inputCoeffFile, line, '\n');
			}
		}

		{
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
		}
		{
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
			//getline(inputCoeffFile, line, '\n'); //Blank line
			//getline(inputCoeffFile, line, '\n'); //Blank line

			string line2(lineChar);
			stringstream muscleStream(line2);
			string nextMuscleName;

			muscleNames_.clear();
			while (!muscleStream.eof())
			{
				muscleStream >> nextMuscleName;
				//std::cout << nextMuscleName << std::endl;
				muscleNames_.push_back(nextMuscleName);
			};

			muscleNames_.pop_back(); // copy twice the last elements
		}

		{
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
		}
		//getline(inputCoeffFile, line, '\n'); //Blank line

		splines_.clear();
		splines_ = (*it1).splines_ = createSplineDim((*it1).uniqueDOFlist.size());
		for (vector<string>::iterator it2 = (*it1).uniqueMuscleList.begin(); it2 != (*it1).uniqueMuscleList.end(); it2++)
			mapMuscleToSpline_[*it2] = splines_[distance<vector<string>::iterator>((*it1).uniqueMuscleList.begin(), it2)];

		std::vector<double> splineCoeff;
		double tempCoefficients;
		int nbMuscle;
		int cptRecur;

		for (int i = 0; i < noMuscles_; ++i)
		{
			
			char lineChar[LENGTH];
			fgets(lineChar, LENGTH, inputCoeffFile);
			//getline(inputCoeffFile, line, '\n');

			string line3(lineChar);
			stringstream tempStream(line3);
			tempStream >> nbMuscle;
			//std::cout << nbMuscle << std::endl;
			int sizeOfCoeff = 1;
			splineCoeff.clear();
			for (int j = (*it1).uniqueDOFlist.size() - 1; j >= 0; --j)
			{
				switch((*it1).uniqueDOFlist.size())
				{
					case 1:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<1>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 2:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<2>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 3:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<3>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 4:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<4>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 5:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<5>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 6:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<6>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 7:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<7>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					case 8:
					{
						sizeOfCoeff *= (dynamic_cast<MTUSpline<8>*>(boost::get_pointer(splines_[i]))->getN()[j] + 3);
						break;
					}
					//default:
					//	throw dimException2;
				}
			}
			for (int j = 0; j < sizeOfCoeff; j++)
			{
				tempStream >> std::fixed >> std::setprecision(20)
						>> tempCoefficients;
				//std::cout << tempCoefficients << std::endl;
				splineCoeff.push_back(tempCoefficients);

			}

			switch((*it1).uniqueDOFlist.size())
			{
				case 1:
				{
					dynamic_cast<MTUSpline<1>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 2:
				{
					dynamic_cast<MTUSpline<2>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 3:
				{
					dynamic_cast<MTUSpline<3>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 4:
				{
					dynamic_cast<MTUSpline<4>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 5:
				{
					dynamic_cast<MTUSpline<5>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 6:
				{
					dynamic_cast<MTUSpline<6>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 7:
				{
					dynamic_cast<MTUSpline<7>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				case 8:
				{
					dynamic_cast<MTUSpline<8>*>(boost::get_pointer(splines_[i]))->setCoefficients(splineCoeff);
					break;
				}
				//default:
				//	throw dimException2;
			}
			cptRecur = (*it1).uniqueDOFlist.size() - 1;

			if (cptRecur > 0)
			{
				switch((*it1).uniqueDOFlist.size())
				{
					case 1:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<1>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<1>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 2:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<2>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<2>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 3:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<3>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<3>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 4:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<4>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<4>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 5:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<5>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<5>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 6:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<6>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<6>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 7:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<7>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<7>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					case 8:
					{
						readCoefficientsFirstPhase(tempStream, dynamic_cast<MTUSpline<8>*>(boost::get_pointer(splines_[i]))->getSplineFirstPhase(), dynamic_cast<MTUSpline<8>*>(boost::get_pointer(splines_[i]))->getCoefficientsSplineSecondPhase(), cptRecur);
						break;
					}
					//default:
					//	throw dimException2;
				}
			}
		}
		//inputCoeffFile.close();
		fclose(inputCoeffFile);
	}
//	cout << " Done" << endl << endl;
}

void MTUSplineDataRead::readCoefficients(const string& inputCoeffFilename)
{
	std::vector<double> coeff;
	coeff.resize(1);
	float sum = 0;
	ifstream inputCoeffFile(inputCoeffFilename.c_str(), std::ifstream::binary);

	if (!inputCoeffFile.is_open())
	{
		cout << "ERROR: " << inputCoeffFilename << " could not be open\n";
		exit(EXIT_FAILURE);
	}

	string line;
	getline(inputCoeffFile, line, '\n');
	getline(inputCoeffFile, line, '\n');
	stringstream myStream(line);
	int nbOfDOF;
	myStream >> subjectName_;
	myStream >> nbOfDOF;
	myStream >> noMuscles_;
	getline(inputCoeffFile, line, '\n'); //Blank line

	for (int i = 0; i < nbOfDOF; ++i)
	{
		inputCoeffFile >> dofName_[i];
		inputCoeffFile >> std::fixed >> std::setprecision(PRECISION) >> a_[i];
		inputCoeffFile >> std::fixed >> std::setprecision(PRECISION) >> b_[i];
		inputCoeffFile >> n_[i];
		getline(inputCoeffFile, line, '\n');
	}

	getline(inputCoeffFile, line, '\n'); //Blank line
	getline(inputCoeffFile, line, '\n'); //Blank line
	stringstream muscleStream(line);
	string nextMuscleName;

	muscleNames_.clear();
	while (!muscleStream.eof())
	{
		muscleStream >> nextMuscleName;
		muscleNames_.push_back(nextMuscleName);
	};

	muscleNames_.pop_back(); // copy twice the last elements

	splines_.clear();
	getline(inputCoeffFile, line, '\n'); //Blank line
	for (int i = 0; i < noMuscles_; ++i)
	{
		std::shared_ptr<MTUSpline<N_DOF> > newSpline(new MTUSpline<N_DOF>(a_, b_, n_));
		splines_.push_back(newSpline);
	}

	std::vector<double> splineCoeff;
	double tempCoefficients;
	int nbMuscle;
	int cptRecur;

	for (int i = 0; i < noMuscles_; ++i)
	{
		getline(inputCoeffFile, line, '\n');
		stringstream tempStream(line);
		tempStream >> nbMuscle;
		int sizeOfCoeff = 1;
		splineCoeff.clear();
		for (int j = N_DOF - 1; j >= 0; --j)
			sizeOfCoeff *= (splines_[i]->getN()[j] + 3);
		for (int j = 0; j < sizeOfCoeff; j++)
		{
			tempStream >> std::fixed >> std::setprecision(20)
					>> tempCoefficients;
			splineCoeff.push_back(tempCoefficients);

		}

		splines_[i]->setCoefficients(splineCoeff);
		cptRecur = N_DOF - 1;
		readCoefficientsFirstPhase(tempStream,
				splines_[i]->getSplineFirstPhase(),
				splines_[i]->getCoefficientsSplineSecondPhase(), cptRecur);
	}

	inputCoeffFile.close();
}

template<class T>
void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		T& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur)
{
	if (cptRecur == 1)
	{
		FirstPhase.getCoefficients();
		double tempCoefficients;
		std::vector<double> splineCoeff;
		int sizeOfCoeff = 1;
		sizeOfCoeff = FirstPhase.getN()[cptRecur - 1] + 3; // verifier correspondance entre dim et cptRecur
		for (int i = 0; i < sizeOfCoeff; i++)
		{
			myStream >> std::fixed >> std::setprecision(20) >> tempCoefficients;
			splineCoeff.push_back(tempCoefficients);
		}
		FirstPhase.setCoefficients(splineCoeff);
	}
	else
	{
		double tempCoefficients;
		std::vector<double> splineCoeff;
		int sizeOfCoeff = 1;

		for (int i = cptRecur - 1; i >= 0; --i)
			sizeOfCoeff *= (FirstPhase.getN()[i] + 3);

		for (int i = 0; i < sizeOfCoeff; i++)
		{
			myStream >> std::fixed >> std::setprecision(20) >> tempCoefficients;
			splineCoeff.push_back(tempCoefficients);
		}
		FirstPhase.setCoefficients(splineCoeff);

		splineCoeff.clear();

		sizeOfCoeff = FirstPhase.getN()[cptRecur - 1] + 3; // verifier correspondance entre dim et cptRecur
		for (int i = 0; i < sizeOfCoeff; i++)
		{
			myStream >> std::fixed >> std::setprecision(20) >> tempCoefficients;
			splineCoeff.push_back(tempCoefficients);
		}
		secondPhaseCoefficients = splineCoeff;
		cptRecur--;
		readCoefficientsFirstPhase(myStream, FirstPhase.getSplineFirstPhase(),
				FirstPhase.getCoefficientsSplineSecondPhase(), cptRecur);
	}
}

void MTUSplineDataRead::evalTaskLmt(bool betweenNode)
{
//	cout << "Begin to compute lmt..." << endl;

//	boost::timer::cpu_timer t;
//	t.start();
//	t.stop();

	for (vector<Task>::const_iterator it1 = vectTask_.begin(); it1 != vectTask_.end(); it1++)
	{
		stringstream temp_str;
		temp_str<<(distance<vector<Task>::const_iterator>(vectTask_.begin(), it1));
		string temp = temp_str.str();
		string fileName;
		string outputDataFileName;
		if(!betweenNode)
		{
			fileName = string("cfg/SplineCoeff/angles_" + temp + ".in");
			outputDataFileName = string("cfg/SplineCoeff/lmt_" + temp + ".out");
		}
		else
		{
			fileName = string("cfg/SplineCoeff/angles_BetweenNode_" + temp + ".in");
			 outputDataFileName = string("cfg/SplineCoeff/lmt_BetweenNode_" + temp + ".out");
		}
		angles_.clear();
		readEvalAngles(fileName, (*it1).uniqueDOFlist.size());
		noMuscles_ = (*it1).uniqueMuscleList.size();

//		t.resume();
		switch((*it1).uniqueDOFlist.size())
		{
			case 1:
			{
				vector<std::shared_ptr<MTUSpline<1> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<1> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 2:
			{
				vector<std::shared_ptr<MTUSpline<2> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<2> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 3:
			{
				vector<std::shared_ptr<MTUSpline<3> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<3> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 4:
			{
				vector<std::shared_ptr<MTUSpline<4> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<4> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 5:
			{
				vector<std::shared_ptr<MTUSpline<5> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<5> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 6:
			{
				vector<std::shared_ptr<MTUSpline<6> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<6> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 7:
			{
				vector<std::shared_ptr<MTUSpline<7> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<7> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			case 8:
			{
				vector<std::shared_ptr<MTUSpline<8> > > temp;
				temp.resize((*it1).splines_.size());
				for(int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<8> >((*it1).splines_[i]);
				evalLmt(outputDataFileName, temp);
				break;
			}
			//default:
			//	throw dimException2;
		}
//		t.stop();
	}

//	boost::timer::cpu_times cpuTime =  t.elapsed();

//	cout << boost::timer::format(cpuTime) << endl;

//	cpuTime.wall = boost::int_least64_t(cpuTime.wall / cpt_);
//	cpuTime.user = boost::int_least64_t(cpuTime.user / cpt_);
//	cpuTime.system = boost::int_least64_t(cpuTime.system / cpt_);

//	cout << "Mean Timing on " << cpt_ << " samples: " << boost::timer::format(cpuTime) << endl;

	cpt_ = 0;


//	cout << "Finish to compute lmt" << endl;
}

void MTUSplineDataRead::evalTaskMa(bool betweenNode)
{
//	cout << "Begin to compute Ma..." << endl;

//	boost::timer::cpu_timer t;
//	t.start();
//	t.stop();

	for (vector<Task>::iterator it1 = vectTask_.begin(); it1 != vectTask_.end(); it1++)
	{
		stringstream temp_str;
		temp_str<<(distance<vector<Task>::const_iterator>(vectTask_.begin(), it1));
		string temp = temp_str.str();
		string fileName;
		if(!betweenNode)
			fileName = string("cfg/SplineCoeff/angles_" + temp + ".in");
		else
			fileName = string("cfg/SplineCoeff/angles_BetweenNode_" + temp + ".in");
		angles_.clear();
		readEvalAngles(fileName, (*it1).uniqueDOFlist.size());
		noMuscles_ = (*it1).uniqueMuscleList.size();
		for(set<string>::iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end(); it2++)
		{
			string outputDataFileName;
			if(!betweenNode)
				outputDataFileName = string("cfg/SplineCoeff/ma_" + *it2 + "_" + temp + ".out");
			else
				outputDataFileName = string("cfg/SplineCoeff/ma_BetweenNode_" + *it2 + "_" + temp + ".out");

//			t.resume();
			switch((*it1).uniqueDOFlist.size())
			{
				case 1:
				{
					vector<std::shared_ptr<MTUSpline<1> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<1> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 2:
				{
					vector<std::shared_ptr<MTUSpline<2> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<2> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 3:
				{
					vector<std::shared_ptr<MTUSpline<3> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<3> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 4:
				{
					vector<std::shared_ptr<MTUSpline<4> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<4> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 5:
				{
					vector<std::shared_ptr<MTUSpline<5> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<5> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 6:
				{
					vector<std::shared_ptr<MTUSpline<6> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<6> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 7:
				{
					vector<std::shared_ptr<MTUSpline<7> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<7> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				case 8:
				{
					vector<std::shared_ptr<MTUSpline<8> > > temp;
					temp.resize((*it1).splines_.size());
					for(int i = 0; i < (*it1).splines_.size(); i++)
						temp[i] = std::dynamic_pointer_cast<MTUSpline<8> >((*it1).splines_[i]);
					evalMa(outputDataFileName, temp, distance<set<string>::iterator>((*it1).uniqueDOFlist.begin(), it2));
					break;
				}
				//default:
				//	throw dimException2;
			}
		}
//		t.stop();
	}

//	boost::timer::cpu_times cpuTime =  t.elapsed();
//
//	cout << boost::timer::format(cpuTime) << endl;
//
//	cpuTime.wall = boost::int_least64_t(cpuTime.wall / cpt_);
//	cpuTime.user = boost::int_least64_t(cpuTime.user / cpt_);
//	cpuTime.system = boost::int_least64_t(cpuTime.system / cpt_);
//
//	cout << "Mean Timing on " << cpt_ << " samples: " << boost::timer::format(cpuTime) << endl;

	cpt_ = 0;

//	cout << "Finish to compute Ma" << endl;
}


template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<1>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<2>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<3>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<4>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<5>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<6>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<7>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);
template void MTUSplineDataRead::readCoefficientsFirstPhase(stringstream& myStream,
		MTUSpline<8>& FirstPhase, std::vector<double>& secondPhaseCoefficients,
		int& cptRecur);

