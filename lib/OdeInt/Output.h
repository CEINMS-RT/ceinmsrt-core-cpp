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
#ifndef Output_h
#define Output_h

#include <vector>

struct Output {
    
    int kMax_;
    int nVar_;
    int nSave_;
    bool dense_;
    int count_;
    double x1_, x2_, xOut_, dxOut_;
    std::vector<double> xSave_;
    std::vector< std::vector<double> > ySave_;
    Output() : kMax_(-1), dense_(false), count_(0) {}

    Output(int nSave) : kMax_(500), nSave_(nSave), count_(0), xSave_(500) {
        dense_ = nSave > 0 ? true : false;
    }

    void init(const int nEqn, const double xLo, const double xHi) {
        nVar_ = nEqn;
        if (kMax_ == -1) return;
        ySave_.resize(nVar_, std::vector<double>(kMax_));
//        for(unsigned i = 0; i < nVar_; ++i)
//            ySave_.at(i).resize(kMax_);
        if (dense_) {
            x1_    = xLo;
            x2_    = xHi;
            xOut_  = x1_;
            dxOut_ = (x2_ - x1_)/nSave_;
        }

    }
    
    void resize() {
        int kOld = kMax_;
        kMax_ *= 2;
        std::vector<double> tempVec(xSave_);
        xSave_.resize(kMax_);
        for (unsigned k = 0; k < (unsigned)kOld; ++k)
        xSave_.at(k) = tempVec.at(k);
        std::vector< std::vector<double> > tempMat(ySave_);
        ySave_.resize(nVar_, std::vector<double>(kMax_));
        for (unsigned i = 0; i < (unsigned)nVar_; ++i)
            for (unsigned k = 0; k < (unsigned)kOld; ++k)
                ySave_.at(i).at(k) = tempMat.at(i).at(k);
    }

    template <class Stepper>
    void save_dense(Stepper& s, const double xout, const double h) {
        
        if (count_ == kMax_) resize();
        for (unsigned i = 0; i < nVar_;i++)
            ySave_.at(i).at(count_) = s.dense_out(i,xout,h);
        xSave_.at(count_++) = xout;
    }
    
    
    void save(double x, const std::vector<double> &y) {

        if (kMax_ <= 0) return;
        if (count_ == kMax_) resize();
        for (unsigned i = 0; i < (unsigned)nVar_; ++i)
            ySave_.at(i).at(count_) = y.at(i);
        xSave_.at(count_++) = x;
    }
    
    
    template <class Stepper>
    void out(const int nStp, const double x, const std::vector<double>& y, Stepper& s, const double h) {
   
        if (!dense_)
            throw("dense output not set in Output!");
        if (nStp == -1) {
            save(x,y);
            xOut_ += dxOut_;
        } else {
            while ((x - xOut_)*(x2_ - x1_) > 0.0) {
            save_dense(s, xOut_,h);
            xOut_ += dxOut_;
            }       
        }
    }

};


#endif