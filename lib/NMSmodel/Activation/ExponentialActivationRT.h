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
