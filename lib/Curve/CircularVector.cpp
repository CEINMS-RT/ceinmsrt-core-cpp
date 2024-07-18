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
#include <vector>
using std::vector;
#include <iterator>
#include <iostream>
#include <stdlib.h>


const unsigned DEFAULT_MAX_SIZE = 15;

template<typename T>
CircularVector<T>::CircularVector()
:size_(DEFAULT_MAX_SIZE), count_(0), beg_(0) { 
 
    v_.resize(size_);
}


template<typename T>
T CircularVector<T>::at(unsigned i) const { 

    if(i >= count_) {
        std::cout << "CircularVector out of range\n";
        exit(EXIT_FAILURE);
    }
    
    return v_.at((beg_+i)%count_);
    
}


template<typename T>
T CircularVector<T>::operator[](unsigned i) const {
  
    return v_[(beg_+i)%count_];
}

template<typename T>
CircularVector<T>& CircularVector<T>::operator=(const CircularVector<T>& rhs) {
    
    v_ = rhs.v_;
    size_ = rhs.size_;
    count_ = rhs.count_;
    beg_ = rhs.beg_;
    
    return *this;
}

template<typename T>
CircularVector<T>& CircularVector<T>::operator=(const std::vector<T>& rhs) {

    v_ = rhs;
    size_ = ((unsigned)rhs.size() > DEFAULT_MAX_SIZE) ? (unsigned)rhs.size() : DEFAULT_MAX_SIZE;
    count_ = (unsigned) rhs.size();
    beg_ = 0;
    
    return *this;
}


template<typename T>
CircularVector<T>::CircularVector(const CircularVector< T >& rhs) {
    
    v_ = rhs.v_;
    size_ = rhs.size_;
    count_ = rhs.count_;
    beg_ = rhs.beg_;
    
}


template<typename T>
void CircularVector<T>::resizeMax(unsigned i) {
    
    size_ = i;
    v_.resize(size_);
    beg_ = 0;
    count_ = 0;
    
}


template<typename T>
void CircularVector<T>::clear() {

    v_.clear();
    v_.resize(DEFAULT_MAX_SIZE);
    beg_ = 0;
    count_ = 0;
}


template<typename T>
void CircularVector<T>::push_back(const T& e) {

    unsigned end = (beg_ + count_) % size_;
    v_.at(end) = e;
    if (count_ == size_) 
        beg_ = (beg_ + 1) % size_;
    else
        ++count_;
}


template<typename T>
void CircularVector<T>::pop_back() {

    if(count_ != 0)
        --count_;
}


template<typename T>
unsigned CircularVector<T>::size() const {
 
    return count_;
}


template<typename T>
bool CircularVector<T>::isFull() {

    return count_ == size_;
    
}


template<typename T>
bool CircularVector<T>::isEmpty() {

    return (count_ == 0);
}
