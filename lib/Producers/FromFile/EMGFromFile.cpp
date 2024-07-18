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


