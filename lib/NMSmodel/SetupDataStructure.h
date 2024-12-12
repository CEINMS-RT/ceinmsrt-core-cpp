// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#ifndef SetupDataStructure_
#define SetupDataStructure_

#include <string>
#include "NMSmodel.hxx"
#include "Curve.h"
#include <set>

template<typename NMSmodelT, typename CurveM>
class SetupDataStructure
{
public:
    typedef typename NMSmodelT::MTUtype MTUT; 
    typedef typename NMSmodelT::DoFtype DoFT;
    typedef typename NMSmodelT::vectorMTUitr vectorMTUitrT;
    SetupDataStructure(const std::string& inputFile);
    void createCurves();
    void createMuscles(NMSmodelT& mySubject);
    void createDoFs(NMSmodelT& mySubject);
    void writeXMLCalibratedFile(NMSmodelT& mySubject, const std::string& XMLfilename);
    void createMusclesNamesOnChannel(NMSmodelT& mySubject);

private:
    // Create the body structure from the Input
    std::auto_ptr<NMSmodelType> subjectPointer;
    CurveM forceVelocityCurve_;
    CurveM activeForceLengthCurve_;
    CurveM passiveForceLengthCurve_;
};

#include "SetupDataStructure.cpp"

#endif
