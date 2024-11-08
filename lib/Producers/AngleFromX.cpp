/*
 * AngleFromX.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#include "AngleFromX.h"
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

void AngleFromX::pushAngleBack(const vector<double>& newAngleToPush)
{
//   SyncTools::Shared::queueAngleSemEmpty.wait();  //waits if the buffer is full. if so it waits until the consumer removes at least one item from queueEmg
//   SyncTools::Shared::queueAngleMutex.lock();
//   SyncTools::Shared::queueAngle.push_back(newAngleToPush);
//   SyncTools::Shared::queueAngleMutex.unlock();
//   SyncTools::Shared::queueAngleSemFull.notify(); //notify that an item has been pushed in the queue
	InterThread::setAngle(newAngleToPush);
}

void AngleFromX::updateAngle(const vector<double>& currentAngleData, double currentTime)
{
  vector<double> AngleDataToPush = currentAngleData;
  AngleDataToPush.push_back(currentTime); //appends currentTime at the end
  pushAngleBack(AngleDataToPush);
}

void AngleFromX::getDofNames(vector<string>& dofNamesFromModel)
{
//   SyncTools::Shared::dofNamesSem.wait();
//   dofNamesFromModel = SyncTools::Shared::dofNames; //gets dof names from XML model, passed from global variable dofNames
//   SyncTools::Shared::dofNamesSem.notify();
	dofNamesFromModel = InterThread::getDofNames();
}

void AngleFromX::getSyncGui(bool& gui)
{
	gui = InterThread::getSyncGui();
}

void AngleFromX::getSyncVerbose(int& verbose)
{
	verbose = InterThread::getSyncVerbose();
}

