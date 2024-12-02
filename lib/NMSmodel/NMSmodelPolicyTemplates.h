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

#ifndef NMSmodelPolicyTemplates_h
#define NMSmodelPolicyTemplates_h

#include <iostream>
#include "ExponentialActivation.h"
#include "ExponentialActivationRT.h"
#include "PiecewiseActivation.h"
#include "StiffTendon.h"
#include "ElasticTendon.h"
#include "ElasticTendon_BiSec.h"
#include "Curve.h"

// template class NMSmodel<ExponentialActivation, StiffTendon, CurveMode::Offline>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivation, StiffTendon, CurveMode::Offline>& b);
// 
// template class NMSmodel<ExponentialActivation, StiffTendon, CurveMode::Online>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivation, StiffTendon, CurveMode::Online>& b);
// 
// template class NMSmodel<PiecewiseActivation, StiffTendon, CurveMode::Offline>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<PiecewiseActivation, StiffTendon, CurveMode::Offline>& b);
// 
// template class NMSmodel<PiecewiseActivation, StiffTendon, CurveMode::Online>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<PiecewiseActivation, StiffTendon, CurveMode::Online>& b);

template class NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> > , CurveMode::Online>& b);

template class NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> > , CurveMode::Offline>& b);

template class NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, CurveMode::Offline>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> > , CurveMode::Offline>& b);

template class NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> > , CurveMode::Online>& b);


template class NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Offline> >, CurveMode::Offline>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Offline> >, CurveMode::Offline>& b);

template class NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online>;
template
std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online>& b);


// template class NMSmodel<ExponentialActivation, ElasticTendon_BiSec, CurveMode::Offline>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivation, ElasticTendon_BiSec, CurveMode::Offline>& b);
// 
// template class NMSmodel<ExponentialActivation, ElasticTendon_BiSec, CurveMode::Online>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<ExponentialActivation, ElasticTendon_BiSec, CurveMode::Online>& b);
// 
// template class NMSmodel<PiecewiseActivation, ElasticTendon_BiSec, CurveMode::Offline>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<PiecewiseActivation, ElasticTendon_BiSec, CurveMode::Offline>& b);
// 
// template class NMSmodel<PiecewiseActivation, ElasticTendon_BiSec, CurveMode::Online>;
// template
// std::ostream& operator<< (std::ostream& output, const NMSmodel<PiecewiseActivation, ElasticTendon_BiSec, CurveMode::Online>& b);


#endif
