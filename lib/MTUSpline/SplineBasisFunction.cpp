// Copyright (c) 2011, Massimo Sartori and Monica Reggiani
// All rights reserved.

// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, 
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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

