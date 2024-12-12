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

#ifndef PiecewiseActivation_h
#define PiecewiseActivation_h

class PiecewiseActivation {
  
public:
    PiecewiseActivation();
    virtual ~PiecewiseActivation();
    
    void   setShapeFactor(double shapeFactor);
    void   setFilterParameters(double c1, double c2);
    void   setEmg(double emg);
    void   updateActivation();
    void   resetState();
    void   pushState();
    double getActivation() const { return activation_; }
    double getEmg() const { return emg_; }
    double getPastEmg() const { return pastEmgT1_; } //controllare se viene usata ancora 7Mar13
    double getNeuralActivation() const { return neuralActivation_; }
    
private:
    void   updateActivationParameters();
    double beta1_;         /**< \f$\beta_1\f$ recursive coefficient */
    double beta2_;         /**< \f$\beta_1\f$ recursive coefficient */
    double alpha_;         /**< \f$\alpha\f$ gain coefficient */
    double shapeFactor_;   /**< \f$A\f$ non-linear shape factor */

    double emg_;           /**< \f$e(t)\f$ high-pass filtered, full-wave rectified,
                            *   and low-pass filtered EMG
                            */
    unsigned noOfEmg_; 
    double pastEmgT1_;
    double pastEmgT2_;
    double neuralActivation_;   /**< \f$u(t)\f$ post-processed EMG of muscle at time t */
    double neuralActivationT1_; /**< \f$u(t-1)\f$ post-processed EMG of muscle at time t-1 */
    double neuralActivationT2_; /**< \f$u(t-2)\f$ post-processed EMG of muscle at time t-2 */
    double activation_;    /**< \f$a(t)\f$ activation of the muscle */
    double a_, b_, Xo_, m_, c_;
};


#endif