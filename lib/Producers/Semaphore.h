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

#ifndef Semaphore_h
#define Semaphore_h

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

//since boost libraries don't provide for semahpores, a simple class, based on mutexes and condition variables, has been made to let the code be clearer
  
class Semaphore
{
public:
    Semaphore();
    Semaphore(unsigned int count);
    ~Semaphore();    
    void notify();
    void wait();
	void waitOnce();

private:
    boost::mutex mutex_;
    boost::condition_variable condition_;
    unsigned int count_;
};
#endif

