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



