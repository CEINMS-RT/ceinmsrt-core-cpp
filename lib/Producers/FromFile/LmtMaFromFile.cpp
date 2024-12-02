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

#include "LmtMaFromFile.h"
#include "DataFromFile.h"
#include "SyncTools.h"


#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <cstdlib>

#define LOG

LmtMaFromFile::LmtMaFromFile(const string& dataDirectory)
                   :dataDirectory_(dataDirectory)
{ }


// void LmtMaFromFile::setLmtMusclesNames(const vector<string>& lmtMusclesNames)
// { 
//   SyncTools::Shared::musclesNamesMutex.lock();
//   if (SyncTools::Shared::musclesNames.empty())         //musclesNames is a global variable, if it isn't empty means that emg set it
//     SyncTools::Shared::musclesNames = lmtMusclesNames;
//   else
//     if (lmtMusclesNames != SyncTools::Shared::musclesNames)
//     {
//       cout << "muscles names are not the same among emg and lmt input data" << endl;
//       exit(EXIT_FAILURE);      
//     }
//   SyncTools::Shared::musclesNamesMutex.unlock();
// }

// void LmtMaFromFile::setMomentArmsMusclesNames(const vector< vector<string> >& musclesNamesFromMomentArmsFiles)
// {
//   SyncTools::Shared::musclesNamesOnDofMutex.lock();
//   SyncTools::Shared::musclesNamesOnDof = musclesNamesFromMomentArmsFiles;
//   SyncTools::Shared::musclesNamesOnDofMutex.unlock();
// }



void LmtMaFromFile::operator()()
{
  double myTime;  
  vector<double> endOfData;

#ifdef LOG
  cout << "starting lmtMaProduce, reading from lmt and ma data files" << endl;
#endif

  vector<string> dofNamesFromModel;
  getDofNames(dofNamesFromModel);
  int noDof = dofNamesFromModel.size();

//LMT INIT
//open lmt file and read muscles names
  string lmtDataFilename = dataDirectory_ + "lmt.txt";  
#ifdef LOG  
  cout << "\nReading..." << lmtDataFilename << endl;
#endif
  DataFromFile myLmtData(lmtDataFilename);
  vector<string> lmtMusclesNames;
  vector<string>::const_iterator it = myLmtData.getColumnNames().begin();
  it++;
  lmtMusclesNames = vector<string>(it, myLmtData.getColumnNames().end() );
//set lmt muscles name read from file to global variable
//   setLmtMusclesNames(lmtMusclesNames);
//END OF LMT INIT
#ifdef LOG  
  cout << "\nend of lmt INIT\n\n";
#endif

//MOMENT ARMS INIT
//open moment arms files and read muscles names
  vector<DataFromFile*> myMomentArmsData;
  for (vector<string>::iterator dofIt = dofNamesFromModel.begin(); dofIt < dofNamesFromModel.end(); ++dofIt) 
  {
    *dofIt = dataDirectory_ + *dofIt + "Ma.txt";
#ifdef LOG  
    cout << "\nReading..." << *dofIt << endl;
#endif
    DataFromFile* newDoFDataFromFilePointer = new DataFromFile(*dofIt);
    myMomentArmsData.push_back(newDoFDataFromFilePointer);  
  }  
  vector< vector <string> > musclesNamesFromMomentArmsFiles;
  for (unsigned int i = 1; i < noDof; ++i) // first row is time
    musclesNamesFromMomentArmsFiles.push_back(myMomentArmsData.at(i)->getColumnNames()); 
//set ma muscles names read from files to global variable,  
  setMusclesNamesOnDof(musclesNamesFromMomentArmsFiles);
//END OF MOMENT ARMS INIT
#ifdef LOG  
  cout << "\nend of ma INIT\n";
#endif
  
  // all initialization stuff MUST be placed before this line
  // be sure to call setLmtMusclesNames(...) and setMomentArmsMusclesNames(...) above
  InterThread::readyToStart->wait();

  while (myLmtData.areStillData())
  {    
    myLmtData.readNextData();
    myTime=myLmtData.getCurrentTime(); //lmt and momentArms times are equal, ALWAYS!    
    updateLmt(myLmtData.getCurrentData(), myTime);
    for (unsigned int i = 0; i < noDof; ++i)
    {
      myMomentArmsData.at(i)->readNextData();
      updateMomentArms(myMomentArmsData.at(i)->getCurrentData(), myTime, i);

    }
  }

  updateLmt(endOfData, 0);
  for (unsigned int i = 0; i < (unsigned int) noDof; ++i)
    updateMomentArms(endOfData, 0, i); 

  InterThread::lmtProducingDone.notify(); //used for validate curve only

#ifdef LOG  
  cout << "\nlmtMa DONE\n\n";
#endif

}

