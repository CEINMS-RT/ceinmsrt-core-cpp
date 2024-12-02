// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software, regulated by the license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots"
//

#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <stdio.h>
#include <fstream>

#include "NMSmodel.h"
#include "MTU.h"
#include "DoF.h"
#include "Curve.h"
#include "SetupDataStructure.h"

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <boost/dll/runtime_symbol_info.hpp>


template<typename NMSmodelT, typename CurveM>
SetupDataStructure<NMSmodelT, CurveM>::SetupDataStructure(const string& configurationFile)
:subjectPointer(subject(configurationFile, xml_schema::flags::dont_initialize))  {  }


template<typename NMSmodelT, typename CurveM>
void SetupDataStructure<NMSmodelT, CurveM>::createCurves() {
    
    NMSmodelType::muscleDefault_type muscleDefault(subjectPointer->muscleDefault());
    MuscleDefaultType::Curve_sequence curveSequence( muscleDefault.Curve());
    for ( MuscleDefaultType::Curve_iterator i = curveSequence.begin(); i != curveSequence.end(); ++i) {
        // each i is a curve
        string curveName = (*i).name();
        //cout << curveName <<std::endl;
        vector<double> x;
    
        PointsSequenceType xPoints =(*i).xPoints();
        PointsSequenceType::iterator pointsIt;
    
        for (pointsIt = xPoints.begin(); pointsIt != xPoints.end(); ++pointsIt) {
            double currentX = (*pointsIt); 
            x.push_back(currentX); 
        }
        
        vector<double> y;
        
        PointsSequenceType yPoints =(*i).yPoints();
    
        for (pointsIt = yPoints.begin(); pointsIt != yPoints.end(); ++pointsIt) {
            double currentY = (*pointsIt); 
            y.push_back(currentY); 
        }
        
    //     for (int i = 0; i < x.size(); ++i)
    //       cout << x[i] << " " << y[i] <<std::endl;
    
        if (curveName == "activeForceLength")  activeForceLengthCurve_.resetPointsWith(x,y);
        if (curveName == "passiveForceLength") passiveForceLengthCurve_.resetPointsWith(x,y);
        if (curveName == "forceVelocity")      forceVelocityCurve_.resetPointsWith(x,y);
    }    
  
//    cout << "activeForceLength" <<std::endl <<  activeForceLengthCurve_ <<std::endl;
//    cout << "passiveForceLength" <<std::endl <<  passiveForceLengthCurve_ <<std::endl;
//    cout <<  "forceVelocity" <<std::endl <<  forceVelocityCurve_ <<std::endl;
}   


template<typename NMSmodelT, typename CurveM>
void SetupDataStructure<NMSmodelT, CurveM>::createMuscles(NMSmodelT& mySubject) {
    
	std::set<std::string> muscleUse;
	
	NMSmodelType::DoFs_type dofs(subjectPointer->DoFs());
    DoFsType::DoF_sequence dofSequence( dofs.DoF());
    DoFsType::DoF_iterator i;
    for (i = dofSequence.begin(); i != dofSequence.end(); ++i) {
    
        string dofName = (*i).name();
        DoFT newDoF(dofName);
        
        // Now have a look at the muscles we have
        MuscleSequenceType currentSequence =(*i).muscleSequence();
        MuscleSequenceType::iterator muscleIt;
    
        for (muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt) {
            string currentMuscle(*muscleIt);
            muscleUse.insert(currentMuscle);
        }  
    }      
    
    NMSmodelType::muscleDefault_type muscleDefault(subjectPointer->muscleDefault());
    NMSmodelType::muscles_type muscles(subjectPointer->muscles());
    MusclesType::muscle_sequence muscleSequence( muscles.muscle() );
    for (MusclesType::muscle_iterator i(muscleSequence.begin()); i != muscleSequence.end(); ++i) {
        string muscleName = (*i).name();
		if(muscleUse.find(muscleName) != muscleUse.end())
		{
			MTUT newMuscle(muscleName);
			double c1 = (*i).C1();
			double c2 = (*i).C2();
			double shapeFactor = (*i).shapeFactor();
			double optimalFiberLength = (*i).optimalFiberLength();
			double pennationAngle = (*i).pennationAngle();
// 			COUT << muscleName << " : " << pennationAngle << std::endl << std::flush;
			double tendonSlackLength = (*i).tendonSlackLength();
			double percentageChange = muscleDefault.percentageChange();
			double damping = muscleDefault.damping();
			double maxIsometricForce = (*i).maxIsometricForce();
			double strengthCoefficient = (*i).strengthCoefficient();
			newMuscle.setParametersToComputeActivation(c1, c2, shapeFactor);
			newMuscle.setCurves(activeForceLengthCurve_, passiveForceLengthCurve_, forceVelocityCurve_);
			newMuscle.setParametersToComputeForces(optimalFiberLength, pennationAngle, 
					tendonSlackLength, percentageChange, damping, maxIsometricForce, strengthCoefficient);
			mySubject.addMuscle(newMuscle);
		}
    } 
}


template<typename NMSmodelT, typename CurveM>
void SetupDataStructure<NMSmodelT, CurveM>::createDoFs(NMSmodelT& mySubject) {

    NMSmodelType::DoFs_type dofs(subjectPointer->DoFs());
    DoFsType::DoF_sequence dofSequence( dofs.DoF());
    DoFsType::DoF_iterator i;
    for (i = dofSequence.begin(); i != dofSequence.end(); ++i) {
    
        string dofName = (*i).name();
        DoFT newDoF(dofName);
        
        // Now have a look at the muscles we have
        MuscleSequenceType currentSequence =(*i).muscleSequence();
        MuscleSequenceType::iterator muscleIt;
    
        for (muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt) {
            string currentMuscle(*muscleIt);
            //cout << currentMuscle <<std::endl;
            vectorMTUitrT found; 
            if (!mySubject.haveThisMuscle(currentMuscle, found)) {
                cout << currentMuscle << " not configured. Sorry, we have to exit!\n";
                exit(EXIT_FAILURE);
            }
            else 
                newDoF.addNewMuscle(found);
        }   
 
        mySubject.addDoF(newDoF);   
    }      
}
 

template<typename NMSmodelT, typename CurveM>
void SetupDataStructure<NMSmodelT, CurveM>::writeXMLCalibratedFile(NMSmodelT& mySubject, const string& XMLfilename) {
    // try copy
    NMSmodelType::muscleDefault_type& muscleDefault(subjectPointer->muscleDefault());
    NMSmodelType::muscles_type& muscles(subjectPointer->muscles());
    MusclesType::muscle_sequence& muscleSequence( muscles.muscle() );  
  
    for (MusclesType::muscle_iterator i(muscleSequence.begin()); i != muscleSequence.end(); ++i) {
        string muscleName = (*i).name();
//         MTUT currentMuscle;
//         mySubject.getMuscle(currentMuscle, muscleName);
		vectorMTUitrT found; 
		if(mySubject.haveThisMuscle(muscleName, found))
		{
			const MTUT& currentMuscle = mySubject.getMuscleConst(muscleName);
			double temp = currentMuscle.getC1();
			(*i).C1(temp);
			temp = currentMuscle.getC2();
			(*i).C2(temp);
			temp = currentMuscle.getShapeFactor();
			(*i).shapeFactor(temp);
			temp = currentMuscle.getOptimalFiberLength();
			(*i).optimalFiberLength(temp);
			temp = currentMuscle.getPennationAngle();
// 			COUT << muscleName << " : " << temp << std::endl << std::flush;
			(*i).pennationAngle(temp);
			temp = currentMuscle.getTendonSlackLength();
			(*i).tendonSlackLength(temp);
			temp = currentMuscle.getPercentageChange();
			muscleDefault.percentageChange(temp);
			temp = currentMuscle.getDamping();
			muscleDefault.damping(temp);
			temp = currentMuscle.getMaxIsometricForce();
			(*i).maxIsometricForce(temp);
			temp = currentMuscle.getStrengthCoefficient();
			(*i).strengthCoefficient(temp); 
		}
   }  
    xml_schema::namespace_infomap map;
    map[""].name = "";
    map[""].schema = fs::absolute(boost::dll::program_location().parent_path().string() + "/../../../XSD/NMSmodel.xsd").string() ;
    
    cout << "Calibrated model save in: " << XMLfilename <<std::endl;
        // Serialize to a file.
        //
    std::ofstream ofs (XMLfilename.c_str());
    subject(ofs, *subjectPointer, map);
    
}

template<typename NMSmodelT, typename CurveM>
void SetupDataStructure<NMSmodelT, CurveM>::createMusclesNamesOnChannel(NMSmodelT& mySubject)
{
	std::map<std::string, std::vector <std::string> > musclesNamesOnChannel;
	NMSmodelType::Channels_type channels(subjectPointer->Channels());
	ChannelsType::Channel_sequence channelSequence(channels.Channel());

	for (ChannelsType::Channel_iterator it = channelSequence.begin(); it != channelSequence.end(); it++)
	{
		MuscleSequenceType& muscleSequence(it->muscleSequence());
		std::vector<std::string> muscleNameVect;
		for (MuscleSequenceType::iterator it2 = muscleSequence.begin(); it2 != muscleSequence.end(); it2++)
			muscleNameVect.push_back(string(*it2));
		musclesNamesOnChannel[string(it->name())] = muscleNameVect;
	}
	mySubject.setMusclesNamesOnChannel(musclesNamesOnChannel);
}

