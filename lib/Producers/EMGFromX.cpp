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

#include "EMGFromX.h"
#include "SyncTools.h"

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <cstdlib>


void EMGFromX::pushEmgBack(const vector<double>& newEmgToPush)
{ 
//   SyncTools::Shared::queueEmgSemEmpty.wait();  //waits if the buffer is full. if so it waits until the consumer removes at least one item from queueEmg
//   SyncTools::Shared::queueEmgMutex.lock();
//   SyncTools::Shared::queueEmg.push_back(newEmgToPush);
//   SyncTools::Shared::queueEmgMutex.unlock();
//   SyncTools::Shared::queueEmgSemFull.notify(); //notify that an item has been pushed in the queue
	InterThread::setEMG(newEmgToPush);
}    

void EMGFromX::updateEmg(const vector<double>& currentEmgData, double currentTime)
{
  vector<double> emgDataToPush = currentEmgData;
  emgDataToPush.push_back(currentTime); //appends currentTime at the end
  pushEmgBack(emgDataToPush);
}

void EMGFromX::getMusclesNames(std::vector< std::string >& muscleNamesFromModel)
{
	muscleNamesFromModel = InterThread::getMusclesNames();
}

void EMGFromX::getMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel)
{
	musclesNamesOnChannel = InterThread::getMusclesNamesOnChannel();
}

void EMGFromX::getSyncGui(bool& gui)
{
	gui = InterThread::getSyncGui();
}

void EMGFromX::getSyncVerbose(int& verbose)
{
	verbose = InterThread::getSyncVerbose();
}

void EMGFromX::setMusclesNames ( const std::vector< std::string >& musclesNames )
{
	InterThread::setMusclesNames(musclesNames);
}


EMGFromX::~EMGFromX() { }



