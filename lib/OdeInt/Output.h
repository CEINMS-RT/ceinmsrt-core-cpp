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