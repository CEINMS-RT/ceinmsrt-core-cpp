#include "MTUSplineInterface.h"

MTUSplineInterface::MTUSplineInterface()
{


}

MTUSplineInterface::MTUSplineInterface(
	const std::string& subjectSpecificXml, /*!< CEINMS specific XML */
	const std::string& subjectName /*!< Name of the subject */
	) : subjectSpecificXml_(subjectSpecificXml), subjectName_(subjectName)
{

}

MTUSplineInterface::~MTUSplineInterface()
{
	delete splineData_;
}

void MTUSplineInterface::initialisation()
{
	splineData_ = new MTUSplineDataRead(subjectSpecificXml_, subjectName_);
	splineData_->readTaskCoefficients();
	taskMTU_ = splineData_->getTaskVect();
	//delete splineData;
}

void MTUSplineInterface::initialisationFromXML()
{
	splineData_ = new MTUSplineDataRead(subjectSpecificXml_, subjectName_);
	splineData_->readTaskCoefficients();
	taskMTU_ = splineData_->getTaskVect();
	//delete splineData;
	/*
	*/try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(subjectSpecificXml_));
		NMSmodelType::muscles_type& muscles(subjectPointer->muscles());
		MusclesType::muscle_sequence& muscleSequence(muscles.muscle());
		for (MusclesType::muscle_iterator i(muscleSequence.begin()); i != muscleSequence.end(); ++i)
			muscleNames_.push_back((*i).name());
		//noMuscles_ = muscleNames_.size();
		// DOF iteration
		NMSmodelType::DoFs_type& dofs(subjectPointer->DoFs());
		DoFsType::DoF_sequence& dofSequence(dofs.DoF());
		for (DoFsType::DoF_iterator i = dofSequence.begin(); i != dofSequence.end(); ++i)
		{
			string currentDOF((*i).name());
			dofNames_.push_back(currentDOF);

			// iteration in the muscle containing in each DOF. See end of subject specific XML.
			MuscleSequenceType currentSequence = (*i).muscleSequence();
			MuscleSequenceType::iterator muscleIt;
			vector<string> muscles;
			for (muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt)
			{
				string currentMuscle(*muscleIt);
				muscles.push_back(currentMuscle);
			}
			musclesNamesOnDof_.push_back(muscles);
		}

		lmt_.resize(muscleNames_.size());


		ma_.resize(musclesNamesOnDof_.size());

		for (vector<vector<string> >::iterator it1 = musclesNamesOnDof_.begin(); it1 != musclesNamesOnDof_.end(); it1++)
			ma_[distance<vector<vector<string> >::iterator>(musclesNamesOnDof_.begin(), it1)].resize(it1->size());
	}
	catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}
//	*/
}

void MTUSplineInterface::setDOFName(vector<std::string> DofName)
{
	dofNames_ = DofName;
}

void MTUSplineInterface::setMusclesNamesOnDof(std::vector<std::vector<std::string> > musclesNamesOnDof)
{
	musclesNamesOnDof_ = musclesNamesOnDof;
	ma_.resize(musclesNamesOnDof_.size());

	for (vector<vector<string> >::iterator it1 = musclesNamesOnDof_.begin(); it1 != musclesNamesOnDof_.end(); it1++)
		ma_[distance<vector<vector<string> >::iterator>(musclesNamesOnDof_.begin(), it1)].resize(it1->size());
}

void MTUSplineInterface::setMuscleName(std::vector<std::string> muscleNames)
{
	muscleNames_ = muscleNames;
	lmt_.resize(muscleNames_.size());
	//noMuscles_ = muscleNames_.size();
}

std::vector<std::string> MTUSplineInterface::getMuscleName()
{
	return muscleNames_;
}

std::vector<std::vector<std::string> > MTUSplineInterface::getMusclesNamesOnDof()
{
	return musclesNamesOnDof_;
}
vector<string> MTUSplineInterface::getDOFName()
{
	return dofNames_;
}

vector< vector<double> > MTUSplineInterface::getMA()
{
	return ma_;
}

vector<double> MTUSplineInterface::getLMT()
{
	return lmt_;
}

void MTUSplineInterface::setPosition(vector<double> position)
{
	for (std::vector<MTUSplineDataRead::Task>::iterator it1 =
		taskMTU_.begin(); it1 != taskMTU_.end(); it1++)
	{

		std::vector<double> lmt;
		std::vector<std::vector<double> > ma;
		std::vector<double> angle;

		//std::cout << (*it1).uniqueDOFlist.size() << std::endl;

		// sort angle data to correspond to the BSpline DOF list
		for (set<string>::const_iterator it2 = it1->uniqueDOFlist.begin(); it2 != it1->uniqueDOFlist.end(); it2++)
			for (vector<string>::const_iterator it3 = dofNames_.begin(); it3 != dofNames_.end(); it3++)
			{
				if (*it2 == *it3)
				{
					angle.push_back(position[distance < vector<string>::const_iterator >(dofNames_.begin(), it3)]);
					break;
				}
			}

		noMuscles_ = it1->uniqueMuscleList.size();

		switch ((*it1).uniqueDOFlist.size()) // Do BSpline magic depending of the order of the spline
		{
		case 1:
		{
			vector<std::shared_ptr<MTUSpline<1> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<1> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, angle, lmt, ma);
		//	std::cout << angle.size() << std::endl;
		//	std::cout << lmt.size() << std::endl;
			break;
		}

		case 2:
		{
			vector<std::shared_ptr<MTUSpline<2> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<2> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 2, angle, lmt, ma);
			break;
		}

		case 3:
		{
			vector<std::shared_ptr<MTUSpline<3> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<3> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 3, angle, lmt, ma);
			break;
		}

		case 4:
		{
			vector<std::shared_ptr<MTUSpline<4> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<4> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 4, angle, lmt, ma);
			break;
		}

		case 5:
		{
			vector<std::shared_ptr<MTUSpline<5> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<5> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 5, angle, lmt, ma);
			break;
		}

		case 6:
		{
			vector<std::shared_ptr<MTUSpline<6> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<6> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 6, angle, lmt, ma);
			break;
		}

		case 7:
		{
			vector<std::shared_ptr<MTUSpline<7> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<7> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 7, angle, lmt, ma);
			break;
		}

		case 8:
		{
			vector<std::shared_ptr<MTUSpline<8> > > temp;
			temp.resize((*it1).splines_.size());

			for (int i = 0; i < (*it1).splines_.size(); i++)
				temp[i] = std::dynamic_pointer_cast<MTUSpline<8> > ((*it1).splines_[i]);

			computeLmtMafromSplines(temp, 8, angle, lmt, ma);
			break;
		}

		default:
#ifdef VERBOSE
				COUT << "The dimension is bigger than 8 which is the maximum accepted." << endl;

#endif
			exit(1);
		}
		// Sort Lmt data to follow the model muscle sorting
		for (vector<string>::const_iterator it2 = muscleNames_.begin(); it2 != muscleNames_.end(); it2++)
		{
			//set<string>::iterator it3 = it1->uniqueMuscleList.find(*it2);
			vector<string>::iterator it3 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it2);

			if (it3 != it1->uniqueMuscleList.end())
			{
				//std::cout << distance < set<string>::iterator >(it1->uniqueMuscleList.begin(), it3) << " ok " << lmt.size()<< std::endl;
				//std::cout << lmt[distance < set<string>::iterator >(it1->uniqueMuscleList.begin(), it3)] << std::endl;
				lmt_[distance<vector<string>::const_iterator>(muscleNames_.begin(), it2)] = lmt[distance <
					vector<string>::iterator >(it1->uniqueMuscleList.begin(), it3)];
			}
		}

		// sort MA data to follow the model muscle on DOF sorting
		for (std::vector<std::vector<std::string> >::const_iterator it2 = musclesNamesOnDof_.begin();
			it2 != musclesNamesOnDof_.end(); it2++)
		{
			set<string>::iterator it4 =
				it1->uniqueDOFlist.find(dofNames_[distance < std::vector<std::vector<std::string> >::const_iterator >(
					musclesNamesOnDof_.begin(), it2)]);

			if (it4 != it1->uniqueDOFlist.end())
			{
				for (vector<string>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++)
				{

					//vector<string>::iterator it5 = it1->uniqueMuscleList.find(*it3);
					vector<string>::iterator it5 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it3);

					if (it5 != it1->uniqueMuscleList.end())
					{
						ma_[distance < std::vector<std::vector<std::string> >::const_iterator >(musclesNamesOnDof_.begin(), it2)][distance <
							vector<string>::const_iterator >(it2->begin(), it3)] = ma[distance<set<string>::iterator>(
								it1->uniqueDOFlist.begin(), it4)][distance < vector<string>::iterator >(it1->uniqueMuscleList.begin(), it5)];
					}
				}
			}
		}
	}
}

template<class T>
void MTUSplineInterface::computeLmtMafromSplines(T& splines, int dim,
	const std::vector<double>& angles, std::vector<double>& lmt,
	std::vector<std::vector<double> >& ma)
{
	lmt.clear();
	ma.clear();

	for (int i = 0; i < noMuscles_; ++i)
		lmt.push_back(splines[i]->getValue(angles));

	ma.resize(dim);

	for (int k = 0; k < dim; ++k)
		for (int i = 0; i < noMuscles_; ++i)
			ma[k].push_back(-splines[i]->getFirstDerivative(angles, k));
}

void MTUSplineInterface::computeLmtMafromSplines(
	std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
	const std::vector<double>& angles, std::vector<double>& lmt,
	std::vector<std::vector<double> >& ma)
{
	lmt.clear();
	ma.clear();

	for (int i = 0; i < noMuscles_; ++i)
		lmt.push_back(splines[i]->getValue(angles[0]));

	ma.resize(1);

	for (int i = 0; i < noMuscles_; ++i)
		ma[0].push_back(-splines[i]->getFirstDerivative(angles[0]));
}