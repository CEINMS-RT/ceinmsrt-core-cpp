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
