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

#include "Semaphore.h"
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

Semaphore::Semaphore()
          : count_(0) { }
          
Semaphore::Semaphore(unsigned int count)
          : count_(count) { }

void Semaphore::notify()
{
  boost::mutex::scoped_lock lock(mutex_);
  ++count_;
  condition_.notify_all();
}

void Semaphore::wait()
{
  boost::mutex::scoped_lock lock(mutex_);
  while(!count_)
    condition_.wait(lock);
  --count_;
}

void Semaphore::waitOnce()
{
  boost::mutex::scoped_lock lock(mutex_);
  //if(count_ <= 0)
	while(!count_)
		condition_.wait(lock);
}

Semaphore::~Semaphore() { }
