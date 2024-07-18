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
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <algorithm>
using std::copy;
#include <iostream>
using std::cout;
using std::endl;
#include <stdlib.h>

//#include "DoF.h"
#include "MTU.h"
#include "Curve.h"

template<typename Activation, typename Tendon, CurveMode::Mode mode>
DoF<Activation, Tendon, mode>::DoF()
	: id_ ( "" ), muscles_ ( 0 ), momentArms_ ( 0 ), torque_ ( 0 ) {  }

template<typename Activation, typename Tendon, CurveMode::Mode mode>
DoF<Activation, Tendon, mode>::DoF ( const string& id )
	: id_ ( id ), muscles_ ( 0 ), momentArms_ ( 0 ), torque_ ( 0 ) {  }

template<typename Activation, typename Tendon, CurveMode::Mode mode>
DoF<Activation, Tendon, mode>::DoF ( const DoF& orig )
	: id_ ( orig.id_ ), muscles_ ( orig.muscles_ ),
		momentArms_ ( orig.momentArms_ ), torque_ ( orig.torque_ ) {  }


template<typename Activation, typename Tendon, CurveMode::Mode mode>
DoF<Activation, Tendon, mode>::~DoF()
{
}

/*!
  Add a new muscle to the list of muscles connected to the current dof
*/
template<typename Activation, typename Tendon, CurveMode::Mode mode>
void DoF<Activation, Tendon, mode>::addNewMuscle ( const vectorMTUitr newMuscle )
{

	muscles_.push_back ( newMuscle );
}


// :TODO: probabilmente qui, invece di passargli una copia del vettore,
//        è meglio passargli il vettore originale con i limiti che deve
//        copiare


/*!
  .

*/
template<typename Activation, typename Tendon, CurveMode::Mode mode>
void DoF<Activation, Tendon, mode>::setMomentArms ( const vector<double>& momentArms )
{

	if ( momentArms.size() != muscles_.size() )
	{
		std::cout << "We have " << momentArms.size() << " ma data for "
				<< muscles_.size() << "muscles.\n";
		std::cout << "Something went wrong, gotta exit!\n";
		exit ( EXIT_FAILURE );
	}

	momentArms_.clear();
	momentArms_.resize ( muscles_.size() );
	copy ( momentArms.begin(), momentArms.end(), momentArms_.begin() );
}


/*!
  Compute the torque <a href="./biblio.html">[2]</a>:
  \f[
  M^{j}(\theta,t)=\sum_{i=1}^m (r_i(\theta) \cdot F_i^{mt}(\theta,t))
  \f]

  where \f$M^j\f$ is the torque of the j-th degree of freedom, \f$r_i(\theta)\f$ and
  \f$F_i^{mt}(\theta,t)\f$ are the moment arm and the musculotendon force of the i-th muscle.

  In the following code:

  \f$\mathtt{torque\_} = M^{j}(\theta,t)\f$,

  \f$\mathtt{muscles\_.at(i)->getMuscleTendonForce()} =F_i^{mt}(\theta,t)\f$,

  \f$\mathtt{*currentMomentArm} = r_i(\theta)\f$

*/

template<typename Activation, typename Tendon, CurveMode::Mode mode>
void DoF<Activation, Tendon, mode>::updateTorque()
{

	torque_ = 0;
	vector<double>::const_iterator currentMomentArm;
	currentMomentArm = momentArms_.begin();
	
// 	COUT;

	for ( unsigned int i = 0; i < muscles_.size(); ++i )
	{/*
		std::cout << "*currentMomentArm: " << *currentMomentArm << std::endl;
		std::cout << "muscles_.at ( i )->getMuscleForce(): " << muscles_.at ( i )->getMuscleForce() << std::endl;
		std::cout << "torque_: " << torque_ << std::endl;*/
		torque_ += muscles_.at ( i )->getMuscleForce() * ( *currentMomentArm );
		currentMomentArm++;
	}
}


template<typename Activation, typename Tendon, CurveMode::Mode mode>
bool DoF<Activation, Tendon, mode>::compareMusclesNames ( const vector<string>& muscleNames ) const
{

	vector<string>::const_iterator m1 = muscleNames.begin();

	for ( typename vector<vectorMTUitr>::const_iterator m2 = muscles_.begin();
			m2 != muscles_.end(); )
	{
		if ( ! ( *m2 )->compareMusclesId ( *m1 ) )
			return false;

		m1++;
		m2++;
	}

	return true;
}


template<typename Activation, typename Tendon, CurveMode::Mode mode>
void DoF<Activation, Tendon, mode>::getMusclesNamesOnDof ( std::vector<std::string>& musclesNames ) const
{
	musclesNames.clear();

	for ( unsigned int i = 0; i < muscles_.size(); ++i )
		musclesNames.push_back ( muscles_.at ( i )->getMuscleName() );
}



template<typename Activation, typename Tendon, CurveMode::Mode mode>
std::ostream& operator<< ( std::ostream& output, const DoF<Activation, Tendon, mode>& dof )
{
	output << "Name: " << dof.id_ << std::endl;
	output << "Muscles: ";

	for ( unsigned int i = 0 ; i < dof.muscles_.size(); ++i )
	{
		dof.muscles_[i]->printName ( output );
		output << " ";
	}

	return output;
}
