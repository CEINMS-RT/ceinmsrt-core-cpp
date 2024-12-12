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

#include "EMGFromFile.h"
#include "SyncTools.h"
#include "EMGDataFromFile.h"
#include "EMGgeneratorFrom16To34.h"
#include "EMGgeneratorFrom6To24.h"

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <iostream>
using std::cout;
using std::endl;

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include <cstdlib>

#define LOG

EMGFromFile::EMGFromFile(const string& dataDirectory)
                 :dataDirectory_(dataDirectory){ }
                 
                 
// void EMGFromFile::setEMGMusclesNames(const vector<string>& emgMusclesNames)
// {
//   SyncTools::Shared::musclesNamesMutex.lock();
//   if (SyncTools::Shared::musclesNames.empty())         //musclesNames is a global variable
//     SyncTools::Shared::musclesNames = emgMusclesNames; 
//   else
//     if (emgMusclesNames != SyncTools::Shared::musclesNames)
//     {
//       cout << "ERROR: muscles names among emg and lmt files are different" << endl;
//       exit(EXIT_FAILURE);          
//     }
//   SyncTools::Shared::musclesNamesMutex.unlock();
// }

void EMGFromFile::operator()()
{
#ifdef LOG
  cout << "emg produce" << endl;
#endif
  double myTime;
  string emgDataFilename = dataDirectory_ + "emg.txt";
#ifdef LOG  
  cout << "\nReading..." << emgDataFilename << endl;
#endif
  EMGDataFromFile<EMGgeneratorFrom16To34> myEmgData(emgDataFilename);
  vector<string> emgMusclesNames;
  myEmgData.getMusclesNames(emgMusclesNames);
  setMusclesNames(emgMusclesNames);

  // all initialization stuff MUST be placed before this line
  // be sure to call setEmgMusclesNames(...) above if you want to
  InterThread::readyToStart->wait();
  
  while (myEmgData.areStillData())
  { 
    myEmgData.readNextEmgData();
    myTime=myEmgData.getCurrentTime();
    updateEmg(myEmgData.getCurrentData(), myTime);
  } 
  vector<double> endOfEmg;
  updateEmg(endOfEmg, 0); 
  
  InterThread::emgProducingDone.notify();
  
#ifdef LOG  
  cout << "\nEMG DONE\n";
#endif
 
}

EMGFromFile::~EMGFromFile(){}


