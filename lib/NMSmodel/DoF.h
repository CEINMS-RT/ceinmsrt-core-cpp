// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox". 
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software, 
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS. 
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621, 
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com 
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

