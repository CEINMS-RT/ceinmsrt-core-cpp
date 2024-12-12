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

#ifndef MuscleParameters_h
#define MuscleParameters_h

class MuscleParameters {
public:
    MuscleParameters();
    ~MuscleParameters();
    bool operator==(const MuscleParameters &other) const;
    double getC1()  const                    { return c1_; }
    double getC2()  const                    { return c2_; }
    double getShapeFactor() const            { return shapeFactor_; }
    double getOptimalFiberLength() const     { return optimalFiberLength_; }
    double getPennationAngle() const         { return pennationAngle_; }
    double getTendonSlackLenght() const      { return tendonSlackLength_; }
    double getMaxIsometricForce() const      { return maxIsometricForce_; }
    double getStrengthCoefficient() const    { return strengthCoefficient_; }
    
    void setC1(double c1)                                  { c1_ = c1; }
    void setC2(double c2)                                  { c2_ = c2; }
    void setShapeFactor(double shapeFactor)                { shapeFactor_ = shapeFactor; }
    void setOptimalFiberLength(double optimalFiberLength)  { optimalFiberLength_ = optimalFiberLength; }
    void setPennationAngle(double pennationAngle)          { pennationAngle_ = pennationAngle; }
    void setTendonSlackLength(double tendonSlackLength)    { tendonSlackLength_ = tendonSlackLength; }
    void setMaxIsometricForce(double maxIsometricForce)    { maxIsometricForce_ = maxIsometricForce; }
    void setStrengthCoefficient(double strengthCoefficient) { strengthCoefficient_ = strengthCoefficient; }
    
private:
    double c1_;
    double c2_;
    double shapeFactor_;
    double optimalFiberLength_;
    double pennationAngle_;   
    double tendonSlackLength_;
    double maxIsometricForce_;
    double strengthCoefficient_;
};

#endif
