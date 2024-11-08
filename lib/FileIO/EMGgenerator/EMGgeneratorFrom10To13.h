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

#ifndef EMGgeneratorFrom10To13_h
#define EMGgeneratorFrom10To13_h

#include <string>
#include <vector>

/**
 * Use this class when you are dealing
 * with 10 EMG input data and you need 13 muscles in your model
 *
 * It accept data only when the input EMG sequence is:
 * <ul>
 *  <li> semimem_r
 *  <li> bifemlh_r
 *  <li> sar_r
 *  <li> rect_fem_r
 *  <li> tfl_r
 *  <li> grac_r
 *  <li> vas_med_r
 *  <li> vas_lat_r
 *  <li> med_gas_r
 *  <li> lat_gas_r
 * </ul>
 *
 * and the output EMG sequence is:
 * <ul>
 *  <li> semimem_r = semimem_r
 *  <li> semiten_r = semimem_r
 *  <li> bifemlh_r = bifemlh_r 
 *  <li> bifemsh_r = bifemlh_r 
 *  <li> sar_r = sar_r
 *  <li> rect_fem_r = rect_fem_r
 *  <li> tfl_r = tfl_r
 *  <li> grac_r = grac_r
 *  <li> vas_med_r = vas_med_r
 *  <li> vas_int_r = avg of vas_med_r and vas_lat_r 
 *  <li> vas_lat_r = vas_lat_r
 *  <li> med_gas_r = med_gas_r 
 *  <li> lat_gas_r = lat_gas_r 
 * </ul>
 *
 * @author Monica Reggiani
 */
class EMGgeneratorFrom10To13 {
public:
  /**
   * Constructor. 
   * Setup names for the 10 input muscles and 13 NMS model muscles
   */
  EMGgeneratorFrom10To13(); 
  
  /**
   * Check that the sequence of muscles in fromMusclesNames matches
   * with the 16 muscles the class is able to deal with
   */
  inline bool checkFromMusclesNames(const std::vector<std::string>& fromMusclesNames) const  {
    return ( (fromMusclesNames == fromMusclesNames_) );
  }
  
  /**
   * \briefTakes 10 emg values and convert them in 13 emg values (two are sets two zero) 
   * 
   * @param[in] fromEMG The sequence of 10 EMG values read from file
   * @param[out] toEMG  The sequence of 13 EMG values converted from fromEMG values
   */
  void convert(const std::vector<double>& fromEMG, std::vector<double>& toEMG) const;
  
  /**
   * \brief return the pointer to the vector of 13 muscles' names
   *
   * @return A const reference to the vector of 13 muscles' names
   * @bug We should change this getMusclesNames and copy instead of returning
   *      a reference
   */
  const std::vector<std::string>&  getMusclesNames() const {return toMusclesNames_;};
  /**
   * \brief It returns 10 
   *
   */
  int getNoFromMuscles() const { return noFromMuscles_;}
  
  /**
   * \brief It returns 13
   *
   */
  int getNoToMuscles() const {return noToMuscles_;}
  
 private:
  static const unsigned int noFromMuscles_ = 10;
  static const unsigned int noToMuscles_ = 13;
  std::vector<std::string> fromMusclesNames_;
  std::vector<std::string> toMusclesNames_;
  
};

#endif
