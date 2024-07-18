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
#include "ExternalTorqueFromFile.h"
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

ExternalTorqueFromFile::ExternalTorqueFromFile(const string& dataDirectory)
                   :dataDirectory_(dataDirectory)
{ }

bool ExternalTorqueFromFile::externalTorqueFileExist(const string& fileName)
{
  std::ifstream dataFile(fileName.c_str());
  bool exist = dataFile.is_open();
  dataFile.close();
  return exist;
}
  
  

void ExternalTorqueFromFile::operator()()
{
  double myTime;  
  vector<double> endOfData;

#ifdef LOG
  cout << "\nstarting externalTorqueProduce, reading from external torque data files" << endl;
#endif

  vector<string> dofNamesFromModel;
  vector<string> externalTorqueDofNames;
  getDofNames(dofNamesFromModel);
  int noDofWithExtTorque = 0;

//TORQUE INIT
  vector<DataFromFile*> myExternalTorqueData;
  for (vector<string>::iterator dofIt = dofNamesFromModel.begin(); dofIt < dofNamesFromModel.end(); ++dofIt) 
  { 
    if(externalTorqueFileExist(dataDirectory_ + *dofIt + "Torque.txt"))
    {
      externalTorqueDofNames.push_back(*dofIt);
      *dofIt = dataDirectory_ + *dofIt + "Torque.txt";
#ifdef LOG  
      cout << "\nReading..." << *dofIt << endl;
#endif
      DataFromFile* newDoFDataFromFilePointer = new DataFromFile(*dofIt);
      myExternalTorqueData.push_back(newDoFDataFromFilePointer);  
      ++noDofWithExtTorque;    
    }  
  }  
//set dof names with and externat torque associated on them to a global variable
  setExternalTorqueDofNames(externalTorqueDofNames); 


//END OF EXTERNAL TORQUE INIT
#ifdef LOG  

  cout << "\nnoDofWithExtTorque " << noDofWithExtTorque << endl;
  cout << "\nend of external torque INIT";
#endif
  
  // all initialization stuff MUST be placed before this line
  // be sure to call setLmtMusclesNames(...) and setMomentArmsMusclesNames(...) above
  InterThread::readyToStart->wait();

  bool stopCondition = false;
  while (!stopCondition)
  {    
    stopCondition = true;
    for (unsigned int i = 0; i < noDofWithExtTorque; ++i)
    {
//poiché le torque esterne possono essere calcolate con differenti frequenze di campionamento
//e quindi un file può contenere più dati di un'altro, il controllo della 
//presenza di dati viene effettuato su tutti i file
      if(myExternalTorqueData.at(i)->areStillData())
      {
        myExternalTorqueData.at(i)->readNextData();
        myTime = myExternalTorqueData.at(i)->getCurrentTime();
        updateExternalTorque(myExternalTorqueData.at(i)->getCurrentData(), myTime, i);
      }
    stopCondition = stopCondition && !myExternalTorqueData.at(i)->areStillData();
    }
  }
  for (unsigned int i = 0; i < noDofWithExtTorque; ++i)
    updateExternalTorque(endOfData, 0, i); 

#ifdef LOG  
  cout << "\nexternalTorque DONE\n";
#endif

}

