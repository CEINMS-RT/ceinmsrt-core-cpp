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
