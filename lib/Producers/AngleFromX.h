/*
 * AngleFromX.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#ifndef ANGLEFROMX_H_
#define ANGLEFROMX_H_

#include <vector>
#include <string>
#include "SyncTools.h"

using namespace std;

class AngleFromX {
public:
	virtual ~AngleFromX() {};
	virtual void operator()() {};
	void pushAngleBack(const vector<double>& newAngleToPush);
	void updateAngle(const vector<double>& currentAngleData, double currentTime);
    void getDofNames(std::vector< std::string >& dofNamesFromModel);
	void getSyncGui(bool& gui);
	void getSyncVerbose(int& verbose);
};

#endif /* ANGLEFROMX_H_ */
