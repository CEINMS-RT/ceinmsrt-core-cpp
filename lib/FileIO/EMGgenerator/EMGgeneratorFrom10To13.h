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
