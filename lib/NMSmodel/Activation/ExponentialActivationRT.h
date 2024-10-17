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
#ifndef ExponentialActivationRT_h
#define ExponentialActivationRT_h

class ExponentialActivationRT {
  
public:
    ExponentialActivationRT();
    virtual ~ExponentialActivationRT();

    void   setShapeFactor(double shapeFactor);
    void   setFilterParameters(double c1, double c2);
    void   setEmg(double emg);
    void   updateActivation();
    void   resetState();
    void   pushState();
    double getActivation() const { return activation_; }
    double getEmg() const { return emg_; }
    double getPastEmg() const { return emgPast_; } //controllare se viene usata ancora 7Mar13
    double getNeuralActivation() const { return emg_; }

private:        
    double shapeFactor_;   /**< \f$A\f$ non-linear shape factor */
    double expShapeFactor_; //   exp(shapeFactor)
    double emg_;           /**< \f$e(t)\f$ high-pass filtered, full-wave rectified,
                            *   and low-pass filtered EMG
                            */
    double emgPast_;
    unsigned noOfEmg_;
    double activation_;    /**< \f$a(t)\f$ activation of the muscle */
};


#endif
