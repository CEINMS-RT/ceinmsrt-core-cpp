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

#include "SplineBasisFunction.h" 
#include <cmath>

double SplineBasisFunction::getValue(double x, int k, double a, double h)
{
	double t = fabs(((x - a) / h) - (k - 1));

	if ((t <= 2) && (t >= 1))
		return pow((2 - t), 3);
	else if (t < 1)
		return (4 - 6 * t * t + 3 * pow(t, 3));
	else
		return 0;
}

double SplineBasisFunction::getFirstDerivative(double x, int k, double a,
		double h)
{
	double t = (((x - a) / h) - (k - 1));

	if ((t <= 2) && (t >= 1))
		return -3 * (2 - t) * (2 - t) / h;
	else if ((t < 1) && (t >= 0))
		return (-12 * t + 9 * t * t) / h;
	else if ((t < 0) && (t >= -1))
		return (-12 * t - 9 * t * t) / h;
	else if ((t < -1) && (t >= -2))
		return 3 * (2 + t) * (2 + t) / h;
	else
		return 0;
}

