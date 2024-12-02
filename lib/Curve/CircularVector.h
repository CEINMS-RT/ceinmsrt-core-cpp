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

#ifndef CircularVector_h
#define CircularVector_h
#include <vector>

template<typename T>
class CircularVector {
    
public:
    CircularVector();
    CircularVector(const CircularVector<T>& rhs); 
    CircularVector<T>& operator=(const CircularVector<T>& rhs);
    CircularVector<T>& operator=(const std::vector<T>& rhs);
    T operator[](unsigned i) const;
    T at(unsigned i) const;
    T back() { return at((count_-1)); }
    void resizeMax(unsigned i);
    void clear();
    void push_back(const T& e);
    void pop_back();
    unsigned getMaxsize() { return size_; }
    unsigned size() const;
private:
    void rotateR();
    void rotateL();
    bool isFull();
    bool isEmpty();
    std::vector<T> v_;
    unsigned beg_, count_, size_;
    
};

#include "CircularVector.cpp"

#endif 
