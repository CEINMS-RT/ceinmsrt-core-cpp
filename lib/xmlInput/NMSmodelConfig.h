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

#ifndef NMSModelCfg_h
#define NMSModelCfg_h

namespace NMSModelCfg {
    
    const unsigned OpenLoop = 1000;
    const unsigned Hybrid   = 2000;
    const unsigned RealTime   = 10000;
                 
    const unsigned ExponentialActivation = 100;
    const unsigned PiecewiseActivation   = 200;
                 
    const unsigned StiffTendon        = 10;
    const unsigned ElasticTendon      = 20;
    const unsigned ElasticTendonBiSec = 30;  
                 
    const unsigned Online  = 1;
    const unsigned Offline = 2;
                 
    const unsigned Nop = 0;
                
    
    enum RunMode{
                 
                 OpenLoopExponentialActivationStiffTendonOnline          = 1111, 
                 OpenLoopExponentialActivationStiffTendonOffline         = 1112,
                 OpenLoopExponentialActivationElasticTendonOnline        = 1121, 
                 OpenLoopExponentialActivationElasticTendonOffline       = 1122, 
                 OpenLoopExponentialActivationElasticTendonBiSecOnline   = 1131, 
                 OpenLoopExponentialActivationElasticTendonBiSecOffline  = 1132,
                 
                 OpenLoopPiecewiseActivationStiffTendonOnline            = 1211, 
                 OpenLoopPiecewiseActivationStiffTendonOffline           = 1212,
                 OpenLoopPiecewiseActivationElasticTendonOnline          = 1221, 
                 OpenLoopPiecewiseActivationElasticTendonOffline         = 1222, 
                 OpenLoopPiecewiseActivationElasticTendonBiSecOnline     = 1231, 
                 OpenLoopPiecewiseActivationElasticTendonBiSecOffline    = 1232,
                 
                 HybridExponentialActivationStiffTendonOnline            = 2111, 
                 HybridExponentialActivationStiffTendonOffline           = 2112, 
                 HybridExponentialActivationElasticTendonOnline          = 2121, 
                 HybridExponentialActivationElasticTendonOffline         = 2122, 
                 HybridExponentialActivationElasticTendonBiSecOnline     = 2131, 
                 HybridExponentialActivationElasticTendonBiSecOffline    = 2132,
                 
                 HybridPiecewiseActivationStiffTendonOnline              = 2211, 
                 HybridPiecewiseActivationStiffTendonOffline             = 2212,
                 HybridPiecewiseActivationElasticTendonOnline            = 2221, 
                 HybridPiecewiseActivationElasticTendonOffline           = 2222, 
                 HybridPiecewiseActivationElasticTendonBiSecOnline       = 2231, 
                 HybridPiecewiseActivationElasticTendonBiSecOffline      = 2232,

                 RealTimeOpenLoopExponentialActivationStiffTendonOnline	 		= 11111,
                 RealTimeOpenLoopExponentialActivationStiffTendonOffline        = 11112,
                 RealTimeOpenLoopExponentialActivationElasticTendonOnline       = 11121,
                 RealTimeOpenLoopExponentialActivationElasticTendonOffline      = 11122,
                 RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline  = 11131,
                 RealTimeOpenLoopExponentialActivationElasticTendonBiSecOffline = 11132,

                 RealTimeOpenLoopPiecewiseActivationStiffTendonOnline           = 11211,
                 RealTimeOpenLoopPiecewiseActivationStiffTendonOffline          = 11212,
                 RealTimeOpenLoopPiecewiseActivationElasticTendonOnline         = 11221,
                 RealTimeOpenLoopPiecewiseActivationElasticTendonOffline        = 11222,
                 RealTimeOpenLoopPiecewiseActivationElasticTendonBiSecOnline    = 11231,
                 RealTimeOpenLoopPiecewiseActivationElasticTendonBiSecOffline   = 11232,

                 RealTimeHybridExponentialActivationStiffTendonOnline           = 12111,
                 RealTimeHybridExponentialActivationStiffTendonOffline          = 12112,
                 RealTimeHybridExponentialActivationElasticTendonOnline         = 12121,
                 RealTimeHybridExponentialActivationElasticTendonOffline        = 12122,
                 RealTimeHybridExponentialActivationElasticTendonBiSecOnline    = 12131,
                 RealTimeHybridExponentialActivationElasticTendonBiSecOffline   = 12132,

                 RealTimeHybridPiecewiseActivationStiffTendonOnline             = 12211,
                 RealTimeHybridPiecewiseActivationStiffTendonOffline            = 12212,
                 RealTimeHybridPiecewiseActivationElasticTendonOnline           = 12221,
                 RealTimeHybridPiecewiseActivationElasticTendonOffline          = 12222,
                 RealTimeHybridPiecewiseActivationElasticTendonBiSecOnline      = 12231,
                 RealTimeHybridPiecewiseActivationElasticTendonBiSecOffline     = 12232,
                 };
};

#endif
