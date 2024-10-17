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

#include <iostream>
using std::cout;
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <stdlib.h>

#include "EMGgeneratorFrom10To13.h"

EMGgeneratorFrom10To13::EMGgeneratorFrom10To13(){

  fromMusclesNames_.clear();
  fromMusclesNames_.push_back("semimem_r");
  fromMusclesNames_.push_back("bifemlh_r");
  fromMusclesNames_.push_back("sar_r");
  fromMusclesNames_.push_back("rect_fem_r");
  fromMusclesNames_.push_back("tfl_r");
  fromMusclesNames_.push_back("grac_r");
  fromMusclesNames_.push_back("vas_med_r");
  fromMusclesNames_.push_back("vas_lat_r");
  fromMusclesNames_.push_back("med_gas_r");
  fromMusclesNames_.push_back("lat_gas_r");
                                                                         
  if ( fromMusclesNames_.size() != noFromMuscles_ ) {  
    cout << "It's quite impossible that you are looking at this message!\n";
    exit (EXIT_FAILURE);
  }
  
  toMusclesNames_.clear();
  toMusclesNames_.push_back("semimem_r");
  toMusclesNames_.push_back("semiten_r");
  toMusclesNames_.push_back("bifemlh_r");
  toMusclesNames_.push_back("bifemsh_r");
  toMusclesNames_.push_back("sar_r");
  toMusclesNames_.push_back("rect_fem_r");
  toMusclesNames_.push_back("tfl_r");
  toMusclesNames_.push_back("grac_r");
  toMusclesNames_.push_back("vas_med_r");
  toMusclesNames_.push_back("vas_int_r");
  toMusclesNames_.push_back("vas_lat_r");
  toMusclesNames_.push_back("med_gas_r");
  toMusclesNames_.push_back("lat_gas_r");

  if ( toMusclesNames_.size() != noToMuscles_ ) {  
    cout << "It's quite impossible that you are looking at this message!\n";
    exit (EXIT_FAILURE);
  }
  
}

void EMGgeneratorFrom10To13::convert(const vector<double>& fromEMG, vector<double>& toEMG) const {
    if (fromEMG.size() != noFromMuscles_) {
       cout << "I need " << noFromMuscles_ << " muscles, but I got " << fromEMG.size() << " muscles!\n";
       exit(EXIT_FAILURE); 
    }
    
    if (toEMG.size() != noToMuscles_) {
       cout << "I need " << noToMuscles_ << " muscles, but I got " << toEMG.size() << " muscles!\n";
       exit(EXIT_FAILURE); 
    }
     
    toEMG.clear();
    toEMG.resize(noToMuscles_);
    
    toEMG[0] = fromEMG[0]; // semimem_r
    toEMG[1] = fromEMG[0]; // semiten_r = semimem_r
    toEMG[2] = fromEMG[1]; // bifemlh_r 
    toEMG[3] = fromEMG[1]; // bifemsh_r = bifemlh_r 
    toEMG[4] = fromEMG[2]; // sar_r 
    toEMG[5] = fromEMG[3]; // rect_fem_r
    toEMG[6] = fromEMG[4]; // tfl_r 
    toEMG[7] = fromEMG[5]; // grac_r 
    toEMG[8] = fromEMG[6]; // vas_med_r 
    toEMG[9] = ( fromEMG[6] + fromEMG[7] ) / 2; // vas_int_r = avg of vas_med_r and vas_lat_r 
    toEMG[10] = fromEMG[7]; // vas_lat_r 
    toEMG[11] = fromEMG[8]; // med_gas_r 
    toEMG[12] = fromEMG[9]; // lat_gas_r 
  
  }
