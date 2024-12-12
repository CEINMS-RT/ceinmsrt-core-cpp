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

#ifndef Dof_h
#define Dof_h

#include <vector>
#include <list>
#include <string>
#include "MTU.h"
#include "Curve.h"
#include <CommonCEINMS.h>

template<typename Activation, typename Tendon, CurveMode::Mode mode>
class DoF;

template<typename Activation, typename Tendon, CurveMode::Mode mode>
std::ostream& operator<< (std::ostream& output, const DoF<Activation, Tendon, mode>& dof);
/** This class associates muscles to the corrisponding degree of freedom
 *  and compute the torque
*/
template<typename Activation, typename Tendon, CurveMode::Mode mode>
class DoF {
public:
    
    typedef MTU<Activation, Tendon, Curve<mode> > MTUtype;
    typedef typename std::vector< MTU<Activation, Tendon, Curve<mode> > >::iterator vectorMTUitr;
    DoF(); 
    DoF(const std::string& id);
    DoF(const DoF& orig);
    virtual ~DoF();
    std::string getName() const {return id_;}
    void addNewMuscle(const typename std::vector<MTUtype>::iterator newMuscle);
    friend std::ostream& operator<< <>(std::ostream& output, const DoF& dof);
    void setMomentArms(const std::vector<double>& momentArms);
    void getMomentArms(std::vector<double>& momentArms) const { momentArms = momentArms_;}
    double getTorque() const {return torque_;}
    bool compareMusclesNames (const std::vector<std::string>& muscleNames) const;
    void getMusclesNamesOnDof (std::vector<std::string>& musclesNames) const;
    void updateTorque();
private:
    std::string id_;
    std::vector<  typename std::vector< MTUtype >::iterator > muscles_;
    std::vector<double> momentArms_;
    double torque_;
};

#include "DoF.cpp"

#endif

