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
