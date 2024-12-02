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

#ifndef EMGgeneratorFrom16To34_h
#define EMGgeneratorFrom16To34_h

#include <string>
#include <vector>

/**
 * Use this class when you are dealing
 * with 16 EMG input data and you need 34 muscles in your model
 *
 * It accept data only when the input EMG sequence is:
 * <ul>
 *  <li> glut_med_r
 *  <li> glut_max_r
 *  <li> tfl_r
 *  <li> sar_r
 *  <li> add_mag_r
 *  <li> grac_r
 *  <li> rect_fem_r
 *  <li> vas_med_r
 *  <li> vas_lat_r
 *  <li> bifemlh_r
 *  <li> semimem_r
 *  <li> lat_gas_r
 *  <li> med_gas_r
 *  <li> soleus_r
 *  <li> tib_ant_r
 *  <li> per_long_r 
 * </ul>
 *
 * and the output EMG sequence is:
 * <ul>
 *  <li> glut_max1_r = glut_max_r
 *  <li> glut_max2_r = glut_max_r  
 *  <li> glut_max3_r = glut_max_r
 *  <li> glut_med1_r = glut_med_r
 *  <li> glut_med2_r = glut_med_r
 *  <li> glut_med3_r = glut_med_r
 *  <li> glut_min1_r = (glut_max_r+glut_med_r)/2
 *  <li> glut_min2_r = (glut_max_r+glut_med_r)/2
 *  <li> glut_min3_r = (glut_max_r+glut_med_r)/2
 *  <li> tfl_r = tfl_r
 *  <li> sar_r = sar_r
 *  <li> *  <li> add_mag1_r = add_mag_r
 *  <li> add_mag2_r = add_mag_r
 *  <li> add_mag3_r = add_mag_r
 *  <li> add_long_r = add_mag_r
 *  <li> add_brev_r = add_mag_r
 *  <li> grac_r = grac_r
 *  <li> rect_fem_r = rect_fem_r
 *  <li> vas_med_r = vas_med_r
 *  <li> vas_int_r = (vas_med_r+vas_lat_r)/2
 *  <li> vas_lat_r = vas_lat_r
 *  <li> bifemlh_r = bifemlh_r
 *  <li> bifemsh_r = bifemlh_r
 *  <li> semimem_r = semimem_r
 *  <li> semiten_r = semimem_r
 *  <li> lat_gas_r = lat_gas_r
 *  <li> med_gas_r = med_gas_r
 *  <li> soleus_r = soleus_r
 *  <li> tib_ant_r = tib_ant_r
 *  <li> per_long_r = per_long_r
 *  <li> per_brev_r = per_long_r
 *  <li> iliacus_r = 0
 *  <li> psoas_r =0
 * </ul>
 *
 * @author Monica Reggiani
 */

class EMGgeneratorFrom16To34 {
public:
  /**
   * Constructor. 
   * Setup names for the 16 input muscles and 34 NMS model muscles
   */
  EMGgeneratorFrom16To34(); 
    
  /**
   * Check that the sequence of muscles in fromMusclesNames matches
   * with the 16 muscles the class is able to deal with
   */
  inline bool checkFromMusclesNames(const std::vector<std::string>& fromMusclesNames) const {
    return ( (fromMusclesNames == fromMusclesNames_) );
  }
  
  /**
   * \briefTakes 16 emg values and convert them in 34 emg values (two are sets two zero) 
   * 
   * @param[in] fromEMG The sequence of 16 EMG values read from file
   * @param[out] toEMG  The sequence of 34 EMG values converted from fromEMG values
   */
  void convert(const std::vector<double>& fromEMG, std::vector<double>& toEMG) const;
  
  /**
   * \brief return the pointer to the vector of 34 muscles' names
   *
   * @return A const reference to the vector of 34 muscles' names
   * @bug We should change this getMusclesNames and copy instead of returning
   *      a reference
   */
  const std::vector<std::string>&  getMusclesNames() const {return toMusclesNames_;};
  
  /**
   * \brief It returns 16 
   *
   */
  int getNoFromMuscles() const { return noFromMuscles_;}
  
  /**
   * \brief It returns34
   *
   */
  int getNoToMuscles() const {return noToMuscles_;}
private:
  static const unsigned int noFromMuscles_ = 16;
  static const unsigned int noToMuscles_ = 34;
  std::vector<std::string> fromMusclesNames_;
  std::vector<std::string> toMusclesNames_;
  
};

#endif

