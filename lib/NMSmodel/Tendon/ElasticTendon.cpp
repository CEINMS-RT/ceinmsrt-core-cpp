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

 #include "ElasticTendon.h"
 


#ifndef RADTODEG
#define RADTODEG
inline double radians (double d) {
return d * 3.1415926535897932384626433832795 / 180;
}

inline double degrees (double r) {
return r * 180/ 3.1415926535897932384626433832795;
}

#endif

#ifndef PENANGLE
#define PENANGLE
class PennationAngle {

public:
    static double compute(double fiberLength, double optimalFiberLength, double pennationAngle) {
    double value = optimalFiberLength*sin(radians(pennationAngle) )/fiberLength;
    if (value <= 0.0)
        return (0.0);
    else if (value >= 1.0)
        return (90.0);
    return (degrees(asin(value)));
    }
};
#endif

#ifndef TENDSPLINEPOINTS
#define TENDSPLINEPOINTS
struct TendonSplinePoints {

    static void getX(vector<double>& x) {
    
        x.clear();
        x.push_back(-10);
        x.push_back(-0.002);
        x.push_back(-0.001);
        x.push_back(0);
        x.push_back(0.00131);
        x.push_back(0.00281);
        x.push_back(0.00431);
        x.push_back(0.00581);
        x.push_back(0.00731);
        x.push_back(0.00881);
        x.push_back(0.0103);
        x.push_back(0.0118);
        x.push_back(0.0123);
        x.push_back(9.2);
        x.push_back(9.201);
        x.push_back(9.202);
        x.push_back(20);
    }

    static void getY(vector<double>& y) {
     
        y.clear();
        y.push_back(0);
        y.push_back(0);
        y.push_back(0);
        y.push_back(0);
        y.push_back(0.0108);
        y.push_back(0.0257);
        y.push_back(0.0435);
        y.push_back(0.0652);
        y.push_back(0.0915);
        y.push_back(0.123);
        y.push_back(0.161);
        y.push_back(0.208);
        y.push_back(0.227);
        y.push_back(345);
        y.push_back(345);
        y.push_back(345);
        y.push_back(345);
    }
    
};
#endif

template <typename CurveM>
LDFM<CurveM>::LDFM(double optimalFiberLength,
           double pennationAngle,
           double tendonSlackLength,
           double percentageChange,
           double damping,
           double maxIsometricForce,
           double strengthCoefficient, 
           const CurveM& activeForceLengthCurveM, 
           const CurveM& passiveForceLengthCurveM,
           const CurveM& forceVelocityCurveM):

optimalFiberLength_(optimalFiberLength),
pennationAngle_(pennationAngle),
tendonSlackLength_(tendonSlackLength),
percentageChange_(percentageChange),
damping_(damping),
maxIsometricForce_(maxIsometricForce),
strengthCoefficient_(strengthCoefficient),
activeForceLengthCurveM_(activeForceLengthCurveM),
passiveForceLengthCurveM_(passiveForceLengthCurveM),
forceVelocityCurveM_(forceVelocityCurveM),
activation_(.0),
muscleTendonLength_(.0),
muscleTendonVelocity_(.0),
muscleTendonForce_(.0),
fiberLength_(.0),
fiberVelocity_(.0), 
fiberStiffness_(.0),
tendonLength_(.0),
tendonVelocity_(.0), 
tendonStiffness_(.0),
optimalFiberLengthAtT_(.0)

{
    vector<double> x,y;
    TendonSplinePoints::getX(x);
    TendonSplinePoints::getY(y);

    tendonForceStrainCurveM_.resetPointsWith(x, y);
}

template <typename CurveM>
double LDFM<CurveM>::computePennationAngle() const {
    
    return PennationAngle::compute(fiberLength_, optimalFiberLengthAtT_, pennationAngle_);
}

template <typename CurveM>
void LDFM<CurveM>::setInitConditions(double muscleTendonLength, double muscleTendonVelocity, double activation) {

    activation_ = activation;
    muscleTendonLength_   = muscleTendonLength;
    muscleTendonVelocity_ = muscleTendonVelocity;
 //   muscleTendonVelocity_ = .0;
    fiberVelocity_  = muscleTendonVelocity; //e' nulla se si usa il CurveMllo online, poiche' mancano punti per la spline
    tendonVelocity_ = 0.0;
                               
    /* Make first guess of muscle and tendon lengths. Make muscle length
    * lmo so that the LDFEM iterative method just moves down one side of
    * the force length curve (i.e. the part of the curve you use is monotonic).
    * And if you are including passive muscle force, you start in the middle
    * of the combined force-length curve.
    */                
    double muscleThick = optimalFiberLength_  * sin(radians(pennationAngle_));
    optimalFiberLengthAtT_ = optimalFiberLength_ * (percentageChange_ * (1.0 - activation) + 1 );
    
    if (muscleTendonLength_ < tendonSlackLength_) {
        fiberLength_  = muscleThick;
        tendonLength_ = muscleTendonLength_;
    } 
    else {
        fiberLength_  = optimalFiberLength_;
        double cf = cos(radians(computePennationAngle()));
        tendonLength_ = muscleTendonLength_ - fiberLength_*cf;

        /* Check to make sure tendon is not shorter than its slack length. If it is,
        * set the length to its slack length and re-compute muscle length.
        */
                        
        if (tendonLength_ < tendonSlackLength_) { 
            
            tendonLength_ = tendonSlackLength_;
            double cf = cos(atan(muscleThick/muscleTendonLength - tendonLength_));
            fiberLength_ = (muscleTendonLength_ - tendonLength_)/cf;
            if (fiberLength_ < muscleThick)
                fiberLength_ = muscleThick;
        }
    }
    
}

template <typename CurveM>
double LDFM<CurveM>::estimateFiberLength() {
    
    double errorForce = DBL_MAX;
    double errorForceOld = DBL_MAX;
    bool runCondition = true;
    const double relTolerance = 1e-5;
    const unsigned maxCount = 1000;
    unsigned count = 0;
    double fiberLengthTmp = .0, fiberLengthOld = .0;
    
    do {
            
        /* force = maxisoforce * (flcurve+pflcurve) * velocity * pennation 
        * calculate active force (af) then passive force (pf) */
            
        computeMuscleTendonForce();
        double tendonStrain = (tendonLength_/ tendonSlackLength_ ) - 1.0;
        double tendonForce = tendonStrain <= 0.0 ? 0.0 : strengthCoefficient_*maxIsometricForce_*tendonForceStrainCurveM_.getValue(tendonStrain);
        
        errorForce = tendonForce - muscleTendonForce_;
        
        if ( (errorForce * errorForceOld) < 0.0 ) {  // if the signs are different
            double percent = fabs(errorForce) / (fabs(errorForce) + fabs(errorForceOld));
            fiberLengthTmp = fiberLength_;
            fiberLength_ += percent * (fiberLengthOld - fiberLength_);
            fiberLengthOld = fiberLengthTmp;
        } 
        else { /* estimate stiffness of the tendon and muscle */
                                        
            tendonStiffness_ = strengthCoefficient_*maxIsometricForce_  / tendonSlackLength_ * tendonForceStrainCurveM_.getFirstDerivative(tendonStrain); 
    
            /* If tendon stiffness is too low, then the next length guess will
            * overshoot the equilibrium point. So we artificially raise it
            * using the normalized muscle force. (af+pf) is the normalized
            * force for the current muscle length, and we assume that the
            * equilibrium length is close to this current length. So we want
            * to get force = (af+pf) from the tendon as well. We hope this
            * will happen by setting the tendon stiffness to (af+pf) times
            * its maximum stiffness. TODO: replace the numbers 1200 and 32.
            */
                                
            //non l' capita del tutto eh.. anzi, per nulla
            //    double minTendonStiffness = (fa+fp)*1200.0*(maxIsometricForce_*strengthCoefficient_*(TENDON_STIFF_SCALE))/
            //                                (32.0*tendonSlackLength_);
            
            double normFiberLength   = fiberLength_ / optimalFiberLengthAtT_;
            double fa = activeForceLengthCurveM_.getValue(normFiberLength);
            double fp = passiveForceLengthCurveM_.getValue(normFiberLength);      
            double minTendonStiffness = (fa+fp)*1200.0*(maxIsometricForce_*strengthCoefficient_)/
                                        (32.0*tendonSlackLength_);
                                    
            if (tendonStiffness_ < minTendonStiffness)
                tendonStiffness_ = minTendonStiffness;
                
            // used activation in the calculation of fiber stiffness
            fiberStiffness_ = (maxIsometricForce_ * strengthCoefficient_*activation_) / optimalFiberLength_ *
                              (activeForceLengthCurveM_.getFirstDerivative(normFiberLength) +
                               passiveForceLengthCurveM_.getFirstDerivative(normFiberLength));
                                  
            /* determine how much the muscle and tendon lengths have to
             * change to make the errorforce zero.   */
            
            double lengthChange = fabs( errorForce/(fiberStiffness_/cos(radians(computePennationAngle())) + tendonStiffness_) );
            
            if (fabs(lengthChange/ optimalFiberLength_) > 0.5)
                lengthChange = 0.5 * optimalFiberLength_ ;
            
                /* now change the muscle length depending on the sign of the error
                * and the sign of the muscle stiffness (which side of the force
                * length curve you're on).
                */
                                        
            fiberLengthOld = fiberLength_;
                
            if (errorForce > 0.0)
                    fiberLength_ += lengthChange;
            else
                    fiberLength_ -= lengthChange;
        }
            
//        optimalFiberLengthAtT_ = optimalFiberLength_ * (percentageChange_ * (1.0 - activation_) + 1 );
//        double pennationAngleAtT = computePennationAngle();
        tendonLength_ = muscleTendonLength_ - fiberLength_ * cos(radians(computePennationAngle() )); 
        errorForceOld = errorForce;
        runCondition = fabs(errorForce) > (relTolerance*(maxIsometricForce_*strengthCoefficient_));
                            
        /* Check to make sure tendon is not shorter than its slack length. If it is,
        * set the length to its slack length and re-compute muscle length.
        */
        
        if (tendonLength_ < tendonSlackLength_) {
            fiberLength_ = (muscleTendonLength_ - tendonLength_)/cos(radians(computePennationAngle()));
            tendonLength_ = tendonSlackLength_;
        }
        
        if (count >= maxCount) {
            muscleTendonForce_ = 0.0;
 //           cout << "LDFM: Could not find force for muscle: " <<  endl;
            runCondition = false;
        }
            
        /* If the error force is less than or equal to a certain percentage
        * of the muscle's max isometric force, then you're done.
        */
        updateInitConditions();
        ++count;             
          
    } while(runCondition);
    return fiberLength_;
}

template <typename CurveM>
void LDFM<CurveM>::updateInitConditions() {
 
    double beta = 1;
    if( (fiberStiffness_ + tendonStiffness_)!= 0 )
        beta = fiberStiffness_ / (fiberStiffness_ + tendonStiffness_);
    tendonVelocity_ = muscleTendonVelocity_* beta;
    double cf = cos(radians(computePennationAngle()));
    fiberVelocity_ = (1.0 - beta) * cf * muscleTendonVelocity_;
}

template <typename CurveM>
void LDFM<CurveM>::computeMuscleTendonForce() {

    double normFiberLength   = fiberLength_ / optimalFiberLengthAtT_;

    //:TODO: THIS IS WRONG! timeScale_?  0.1 should be timeScale_
    // double normFiberVelocity = timescale_ *fiberVelocity_ / optimalFiberLengthAtT;
    double normFiberVelocity = 0.02 * fiberVelocity_ * 0.1 / optimalFiberLengthAtT_;
    
    double fa = activeForceLengthCurveM_.getValue(normFiberLength);
    double fp = passiveForceLengthCurveM_.getValue(normFiberLength);                        
    double fv = forceVelocityCurveM_.getValue(normFiberVelocity);
    
    double pennationAngleAtT = computePennationAngle();
    muscleTendonForce_ = maxIsometricForce_ * strengthCoefficient_ *
                                (fa * fv * activation_ + fp + damping_ * normFiberVelocity) * cos(radians(pennationAngleAtT));
   
}

template <typename CurveM>
double LDFM<CurveM>::oldEstimation(double muscleTendonLength, double muscleTendonVelocity, double activation) {
    
        const double relTolerance = 1e-3;
    int i, j, count;
    double fiber_stiffness, mthick, tmpfiber_length, min_tendon_stiffness;
    double af, pf, fv, cf, tendon_stiffness, percent, tendonstrain, oldfiber_length;
    double lengthchange, errorforce, olderrorforce, tendonforce, force, beta;
    
    muscleTendonLength_ = muscleTendonLength;
    muscleTendonVelocity_ = muscleTendonVelocity;
    activation_ = activation;
    
    if (muscleTendonLength_ <= tendonSlackLength_)
    {
//        ds->force[i][0] = 0.0;
        fiberLength_ = 0.0;
        tendonLength_ = tendonSlackLength_/ optimalFiberLength_ ; 
    }
      
        
        /* now calculate musculotendon velocity; initial guess of velocity distribution 

    * puts all velocity in muscle */ 

    fiberVelocity_ = muscleTendonVelocity_;
    tendonVelocity_ = 0.0;
                                

                /* Make first guess of muscle and tendon lengths. Make muscle length

                * lmo so that the LDFEM iterative method just moves down one side of

                * the force length curve (i.e. the part of the curve you use is monotonic).

                * And if you are including passive muscle force, you start in the middle

                * of the combined force-length curve.

                */

                

    mthick = optimalFiberLength_ * sin(radians(pennationAngle_));
                
    if (muscleTendonLength_ < tendonSlackLength_)
    {
        fiberLength_ = mthick;
        tendonLength_ = muscleTendonLength_;
    }
    else
    {
        fiberLength_ = optimalFiberLength_;
        cf = cos(radians(computePennationAngle()));  
        tendonLength_=muscleTendonLength_ - fiberLength_*cf;


        /* Check to make sure tendon is not shorter than its slack length. If it is,

        * set the length to its slack length and re-compute muscle length.

        */

            
        if (tendonLength_ < tendonSlackLength_)
        {
                            
            tendonLength_ = tendonSlackLength_;
            cf = cos(atan(mthick/muscleTendonLength_ - tendonLength_));
            fiberLength_ = (muscleTendonLength_ - tendonLength_)/cf;
            if (fiberLength_ < mthick)
                fiberLength_ = mthick;
        }
    }
        
        /* the LDFEM (Loan Dynamic Force Estimation Method) loop is done twice. The
    * first time through, the tendon velocity is assumed to be zero, so all of
    * the velocity is `in' the muscle. The muscle force is then found iteratively
    * for this situation. This is done by guessing the length of the muscle and
    * of the tendon, and calculating their respective forces. If the forces
    * match (are within MAXIMUMERROR of each other), stop; else change the length
    * guesses based on the error and try again. Once the muscle force is know
    * a better estimate of its velocity can be made. The slope of the force-length
    * curve at this length is compared to the slope of the tendon force-leng
    * curve, and the velocities are guessed again.
    */

    mthick = optimalFiberLength_  * sin(radians(pennationAngle_));


    for (j=0; j<2; j++)
    {
        count = 0;
        while (1)
        {

            count++;
                
            /* force = maxisoforce * (flcurve+pflcurve) * velocity * pennation 
            * calculate active force (af) then passive force (pf) */
                
            af = activeForceLengthCurveM_.getValue(fiberLength_)*activation_;
            if (af < 0.0)
                    af = 0.0;
                
            pf = passiveForceLengthCurveM_.getValue(fiberLength_);
            if (pf < 0.0)
                    pf = 0.0;
                
            fv = forceVelocityCurveM_.getValue(fiberVelocity_);
                
            force = (af*fv + pf) * (maxIsometricForce_*strengthCoefficient_) * cf;
                
            tendonstrain = (tendonLength_/tendonSlackLength_ - 1.0);
            if (tendonstrain <= 0.0)
                    tendonforce = 0.0;
            else

                    tendonforce = maxIsometricForce_*tendonForceStrainCurveM_.getValue(tendonstrain);
                
                                errorforce = tendonforce -force;
                                if (count==1)
                                        olderrorforce = errorforce;
                                /* If the error force is less than or equal to a certain percentage
                                * of the muscle's max isometric force, then you're done.
                                */
                                if (count > 1)
                                {
                                        if (fabs(errorforce) <= relTolerance*(maxIsometricForce_*strengthCoefficient_))
                                                break;

                                }
                                
                                if ((errorforce * olderrorforce) < 0)
                                {
                                        percent = fabs(errorforce) / (fabs(errorforce) + fabs(olderrorforce));
                                        tmpfiber_length = oldfiber_length;
                                        oldfiber_length = fiberLength_;
                                        fiberLength_ += percent * (tmpfiber_length - fiberLength_);
                                } 
                                else /* estimate stiffness of the tendon and muscle */
                                {
                                        
                                        tendon_stiffness = (maxIsometricForce_)/ (tendonSlackLength_)*tendonForceStrainCurveM_.getFirstDerivative(tendonstrain);
                                        
                                                /* If tendon stiffness is too low, then the next length guess will
                                                * overshoot the equilibrium point. So we artificially raise it
                                                * using the normalized muscle force. (af+pf) is the normalized
                                                * force for the current muscle length, and we assume that the
                                                * equilibrium length is close to this current length. So we want
                                                * to get force = (af+pf) from the tendon as well. We hope this
                                                * will happen by setting the tendon stiffness to (af+pf) times
                                                * its maximum stiffness. TODO: replace the numbers 1200 and 32.
                                        */
                                        double tendonStiffScale = 1;
                                        min_tendon_stiffness = (af+pf)*1200.0*((maxIsometricForce_)*(strengthCoefficient_)*(tendonStiffScale))/
                                                (32.0*(tendonSlackLength_));
                                        
                                        if (tendon_stiffness < min_tendon_stiffness)
                                                tendon_stiffness = min_tendon_stiffness;
                                        
                                        // used activation in the calculation of fiber stiffness
                                        
                                        double normFiberLength = fiberLength_ / optimalFiberLength_;
                                         fiber_stiffness = (maxIsometricForce_ * strengthCoefficient_*activation_) / optimalFiberLength_ *
                                                            (activeForceLengthCurveM_.getFirstDerivative(normFiberLength) +
                                                            passiveForceLengthCurveM_.getFirstDerivative(normFiberLength));
                                           /* determine how much the muscle and tendon lengths have to
                                                * change to make the errorforce zero.   */
                                        
                                        lengthchange = fabs( errorforce/(fiber_stiffness/cf + tendon_stiffness) );
                                        
                                        if (fabs(lengthchange/ optimalFiberLength_) > 0.5)
                                                lengthchange = 0.5*optimalFiberLength_ ;
                                        
                                                /* now change the muscle length depending on the sign of the error
                                                * and the sign of the muscle stiffness (which side of the force
                                                * length curve you're on).
                                        */
                                        
                                        oldfiber_length = fiberLength_;
                                       
                                        if (errorforce > 0.0)
                                            fiberLength_ += lengthchange;
                                        else
                                            fiberLength_ -= lengthchange;
                                }
                                
                                cf = cos(radians(computePennationAngle()));  
                                tendonLength_ = muscleTendonLength_ - fiberLength_*cf;
                                olderrorforce = errorforce;
                                
                                /* Check to make sure tendon is not shorter than its slack length. If it is,
                                * set the length to its slack length and re-compute muscle length.
                                */
                                
                                if (tendonLength_ < tendonSlackLength_)
                                {
                                
                                    tendonLength_ = tendonSlackLength_;
                                    cf = cos(atan(mthick/(muscleTendonLength_ - tendonLength_)));
                                    fiberLength_ = (muscleTendonLength_- tendonLength_)/cf;
                                }
                                if (count >= 100)
                                {
                                    force = 0.0;
                                    break;
                                }
      }
      /* if the first iteration, adjust the guess of muscle and tendon
          * velocity now that you know the (quasi) real lengths.
          * The total musculotendon velocity is distributed between the fibers
          * and tendon based on their relative stiffnesses (the stiffer componen
          * gets less velocity). The velocity equation for a pennate muscles is:
          *    Vmt = Vtend + Vfibers / cos(pennation)
          * Using the fiber and tendon stiffnesses to distribute velocity:
          *    Vtend = Vmt * (fiber_stiffness / (fiber_stiffness + tendon_stiffness))
          * Then plugging the second equation into the first:
          *    Vfibers = (1-B) * cos(pennation) * Vmt,
          * where B = (fiber_stiffness / (fiber_stiffness + tendon_stiffness)
          */
          
      if (j == 1)
                  break;
          
        beta = fiber_stiffness / (fiber_stiffness + tendon_stiffness);
        tendonVelocity_ = muscleTendonVelocity_ * beta;
        fiberVelocity_ = (1.0 - beta) * cf * muscleTendonVelocity_;
          
           
     }
     return fiberLength_;
     
} 
//*************//


template <typename CurveM>
ElasticTendon<CurveM>::ElasticTendon():
optimalFiberLength_(.0),
pennationAngle_(.0),
tendonSlackLength_(.0),
percentageChange_(.0),
damping_(.0),
maxIsometricForce_(.0), 
strengthCoefficient_(.0),
muscleTendonLength_(0.0),
muscleTendonLengthT1_(0.0),
muscleTendonVelocity_(0.0),
muscleTendonVelocityT1_(0.0),
fiberLength_(0.0),
fiberLengthT1_(0.0),
fiberVelocity_(0.0),
muscleTendonForce_(0.0),
activation_(0.0),
activationT1_(0.0),
timescale_(0.004),
nLmt_(0),
lmtTime_(.0),
lmtTimeT1_(.0),
id_("")
{ 
    
    vector<double> x,y;
    TendonSplinePoints::getX(x);
    TendonSplinePoints::getY(y);

    tendonForceStrainCurveM_.resetPointsWith(x, y);
}


template <typename CurveM>
ElasticTendon<CurveM>::ElasticTendon(std::string id):
optimalFiberLength_(.0),
pennationAngle_(.0),
tendonSlackLength_(.0),
percentageChange_(.0),
damping_(.0),
maxIsometricForce_(.0), 
strengthCoefficient_(.0),
muscleTendonLength_(0.0),
muscleTendonLengthT1_(0.0),
muscleTendonVelocity_(0.0),
muscleTendonVelocityT1_(0.0),
fiberLength_(0.0),
fiberLengthT1_(0.0),
fiberVelocity_(0.0),
muscleTendonForce_(0.0),
activation_(0.0),
activationT1_(0.0),
timescale_(0.004),
nLmt_(0),
lmtTime_(.0),
lmtTimeT1_(.0),
id_(id)
{ 
    vector<double> x,y;
    TendonSplinePoints::getX(x);
    TendonSplinePoints::getY(y);

    tendonForceStrainCurveM_.resetPointsWith(x, y);
}


template <typename CurveM>
ElasticTendon<CurveM>::ElasticTendon (double optimalFiberLength, 
                              double pennationAngle, 
                              double tendonSlackLength, 
                              double percentageChange, 
                              double damping, 
                              double maxIsometricForce, 
                              double strengthCoefficient, 
                              const CurveM& activeForceLengthCurveM, 
                              const CurveM& passiveForceLengthCurveM, 
                              const CurveM& forceVelocityCurveM):

optimalFiberLength_(optimalFiberLength),
pennationAngle_(pennationAngle),
tendonSlackLength_(tendonSlackLength),
percentageChange_(percentageChange),
damping_(damping),
maxIsometricForce_(maxIsometricForce), 
strengthCoefficient_(strengthCoefficient),
activeForceLengthCurveM_(activeForceLengthCurveM),
passiveForceLengthCurveM_(passiveForceLengthCurveM),
forceVelocityCurveM_(forceVelocityCurveM),
muscleTendonLength_(0.0),
muscleTendonLengthT1_(0.0),
muscleTendonVelocity_(0.0),
muscleTendonVelocityT1_(0.0),
fiberLength_(0.0),
fiberLengthT1_(0.0),
fiberVelocity_(0.0),
muscleTendonForce_(0.0),
activation_(0.0),
activationT1_(0.0),
timescale_(0.004),
nLmt_(0),
lmtTime_(.0),
lmtTimeT1_(.0),
id_("")
{   

    vector<double> x,y;
    TendonSplinePoints::getX(x);
    TendonSplinePoints::getY(y);

    tendonForceStrainCurveM_.resetPointsWith(x, y);
}




template <typename CurveM>
ElasticTendon<CurveM>::ElasticTendon ( const ElasticTendon<CurveM>& orig )
{

    cout << "ElasticTendon copy constructor. EXIT\n";
    exit(EXIT_FAILURE);
}


template <typename CurveM>
ElasticTendon<CurveM>& ElasticTendon<CurveM>::operator= ( const ElasticTendon<CurveM>& orig )
{

    optimalFiberLength_      = orig.optimalFiberLength_;
    pennationAngle_          = orig.pennationAngle_;
    tendonSlackLength_       = orig.tendonSlackLength_;
    percentageChange_        = orig.percentageChange_;
    damping_                 = orig.damping_;
    maxIsometricForce_       = orig.maxIsometricForce_;
    strengthCoefficient_     = orig.strengthCoefficient_;
    activeForceLengthCurveM_  = orig.activeForceLengthCurveM_;
    passiveForceLengthCurveM_ = orig.passiveForceLengthCurveM_;
    forceVelocityCurveM_      = orig.forceVelocityCurveM_;
    tendonForceStrainCurveM_  = orig.tendonForceStrainCurveM_;
    
    muscleTendonLength_      = orig.muscleTendonLength_;
    muscleTendonLengthT1_    = orig.muscleTendonLengthT1_;
    muscleTendonVelocity_    = orig.muscleTendonVelocity_;   
    muscleTendonVelocityT1_  = orig.muscleTendonVelocityT1_;
    fiberLength_             = orig.fiberLength_;
    fiberLengthT1_           = orig.fiberLengthT1_;          
    fiberVelocity_           = orig.fiberVelocity_;
    muscleTendonForce_       = orig.muscleTendonForce_;
    activation_              = orig.activation_;             
    activationT1_            = orig.activationT1_;
    timescale_               = orig.timescale_;
    lmtTime_                 = orig.lmtTime_;
    lmtTimeT1_               = orig.lmtTimeT1_;
    nLmt_                    = orig.nLmt_;
    id_                      = orig.id_;
    return *this;
}


template <typename CurveM>
void ElasticTendon<CurveM>::setParametersToComputeForces(double optimalFiberLength,
                                                 double pennationAngle,
                                                 double tendonSlackLength,
                                                 double percentageChange,
                                                 double damping, 
                                                 double maxIsometricForce, 
                                                 double strengthCoefficient) {
 
    optimalFiberLength_  = optimalFiberLength;
    pennationAngle_      = pennationAngle;
    tendonSlackLength_   = tendonSlackLength;
    percentageChange_    = percentageChange;
    damping_             = damping;
    maxIsometricForce_   = maxIsometricForce;
    strengthCoefficient_ = strengthCoefficient;
}


template <typename CurveM>
void ElasticTendon<CurveM>::setCurves(const CurveM& activeForceLengthCurveM, 
                                    const CurveM& passiveForceLengthCurveM, 
                                    const CurveM& forceVelocityCurveM) { 
                                  
    
    activeForceLengthCurveM_  = activeForceLengthCurveM;
    passiveForceLengthCurveM_ = passiveForceLengthCurveM;
    forceVelocityCurveM_      = forceVelocityCurveM;
};

template <typename CurveM>
void ElasticTendon<CurveM>::setStrengthCoefficient (double strengthCoefficient) {
    
    strengthCoefficient_ = strengthCoefficient;
    resetState();

}


template <typename CurveM>
void ElasticTendon<CurveM>::setTendonSlackLength (double tendonSlackLength) {
    
    tendonSlackLength_ = tendonSlackLength;
    resetState();

}


template <typename CurveM>
void ElasticTendon<CurveM>::resetState() {

    muscleTendonLength_ = 0.0;
    muscleTendonLengthT1_ = 0.0;
    muscleTendonVelocity_ = 0.0;
    muscleTendonVelocityT1_ = 0.0;
    fiberLength_ = 0.0;
    fiberLengthT1_ = 0.0;
    fiberVelocity_ = 0.0;
    muscleTendonForce_ = 0.0;
    activation_ = 0.0;
    activationT1_ = 0.0;
    nLmt_ = 0;
    lmtTime_ = .0;
    lmtTimeT1_ = .0;
    muscleTendonLengthTrace_.reset();
}


template <typename CurveM>
double ElasticTendon<CurveM>::getInitialFiberLength(double time) {
   
    
 //   muscleTendonVelocity_ = (muscleTendonLength_ - muscleTendonLengthT1_)/timescale_;

    LDFM<CurveM> ldfm(optimalFiberLength_,
              pennationAngle_,
              tendonSlackLength_,
              percentageChange_,
              damping_,
              maxIsometricForce_,
              strengthCoefficient_, 
              activeForceLengthCurveM_, 
              passiveForceLengthCurveM_,
              forceVelocityCurveM_);

    
    
    ldfm.setInitConditions(muscleTendonLength_, muscleTendonVelocity_, activation_);
        
    /* the LDFEM (Loan Dynamic Force Estimation Method) loop is done twice. The
    * first time through, the tendon velocity is assumed to be zero, so all of
    * the velocity is `in' the muscle. The muscle force is then found iteratively
    * for this situation. This is done by guessing the length of the muscle and
    * of the tendon, and calculating their respective forces. If the forces
    * match (are within MAXIMUMERROR of each other), stop; else change the length
    * guesses based on the error and try again. Once the muscle force is known,
    * a better estimate of its velocity can be made. The slope of the force-length
    * curve at this length is compared to the slope of the tendon force-length
    * curve, and the velocities are guessed again.
    */

    
      
    /* if the first iteration, adjust the guess of muscle and tendon
    * velocity now that you know the (quasi) real lengths.
    * The total musculotendon velocity is distributed between the fibers
    * and tendon based on their relative stiffnesses (the stiffer component
    * gets less velocity). The velocity equation for a pennate muscles is:
    *    Vmt = Vtend + Vfibers / cos(pennation)
    * Using the fiber and tendon stiffnesses to distribute velocity:
    *    Vtend = Vmt * (fiber_stiffness / (fiber_stiffness + tendon_stiffness))
    * Then plugging the second equation into the first:
    *    Vfibers = (1-B) * cos(pennation) * Vmt,
    * where B = (fiber_stiffness / (fiber_stiffness + tendon_stiffness)
    */
        
    fiberLength_ = ldfm.estimateFiberLength();
    
    return fiberLength_;
}


template <typename CurveM>
void ElasticTendon<CurveM>::setMuscleTendonLength(double muscleTendonLength, double activation, double time) {

    
    
    lmtTimeT1_ = lmtTime_;
    lmtTime_ = time;
    timescale_ = lmtTime_ - lmtTimeT1_;
    activationT1_  = activation_;
    activation_    = activation;
   
    
    muscleTendonLengthT1_ = muscleTendonLength_;
    muscleTendonLength_   = muscleTendonLength;
    muscleTendonLengthTrace_.addPoint(time, muscleTendonLength_);
    
    muscleTendonVelocityT1_ = muscleTendonVelocity_;
    muscleTendonVelocity_ = muscleTendonLengthTrace_.getFirstDerivative(time);
    //muscleTendonVelocity_ = (muscleTendonLength_ - muscleTendonLengthT1_)/timescale_;
    
    fiberLengthT1_ = fiberLength_;
            
    
    //we need the first two values of lmt for getting the initial muscleTendonVelocity value
    //TODO if nLmt_ == 0 do nothing or something like that, verify which one is better
    if(nLmt_ < 3)
        fiberLength_ = getInitialFiberLength(time);
    else
        fiberLength_ = getFiberLengthRKF();
 
    ++nLmt_;
}

template <typename CurveM>
void ElasticTendon<CurveM>::setMuscleTendonLength(double muscleTendonLength) {
    
    muscleTendonLengthT1_ = muscleTendonLength_;
    muscleTendonLength_   = muscleTendonLength;
    muscleTendonLengthTrace_.addPoint(time_, muscleTendonLength_);
    
    muscleTendonVelocityT1_ = muscleTendonVelocity_;
    muscleTendonVelocity_ = muscleTendonLengthTrace_.getFirstDerivative(time_);
    //muscleTendonVelocity_ = (muscleTendonLength_ - muscleTendonLengthT1_)/timescale_;
}


template <typename CurveM>
void ElasticTendon<CurveM>::updateFiberLengthUsingNewActivation(double activation, double time) {

    activation_    = activation;    
    
    //we need the first two values of lmt for getting the initial muscleTendonVelocity value
    //TODO if nLmt_ == 0 do nothing or something like that, verify which one is better
 
    if(nLmt_ < 3)
        fiberLength_ = getInitialFiberLength(time);
    else
        fiberLength_ = getFiberLengthRKF(); 
    
    ++nLmt_;
    
}


template <typename CurveM>
double ElasticTendon<CurveM>::getFiberLengthRKF() {


    const int nvar = 1;
    double atol = 1.0e-3, rtol = atol, h1 = 0.01, hmin = 0.0, x1 = lmtTimeT1_, x2 = lmtTime_;
    vector<double> ystart(nvar);
    Output out(1);
    
    ystart.at(0) = fiberLengthT1_;

    Odeint< StepperDopr5<ElasticTendon> > ode(ystart,x1,x2,atol,rtol,h1,hmin,out, *this);

    ode.integrate();
    
//    cout << "rk " << out.ySave_.at(0).at(1);
    
    return out.ySave_.at(0).at(1);
    
}


template <typename CurveM>
void ElasticTendon<CurveM>::operator() (double x, const std::vector<double>& y, std::vector<double>& dydx) {

    testTime_ = x;
    double time = x;
    double delta = time - lmtTimeT1_;
    //double lmtTimescale = lmtTime_ - lmtTimeT1_; //should be equal to timescale_ 
    
    //**the following can be done with a spline
    double muscleTendonLengthAtT = (delta *(muscleTendonLength_ - muscleTendonLengthT1_)/timescale_) + muscleTendonLengthT1_;
    double activationAtT = (delta *(activation_- activationT1_ )/timescale_) + activationT1_;
    double muscleTendonVelocityAtT = (delta *(muscleTendonVelocity_ - muscleTendonVelocityT1_)/timescale_) + muscleTendonVelocityT1_;
    
    dydx.at(0) = computeFiberVelocityAtT(muscleTendonLengthAtT, activationAtT, muscleTendonVelocityAtT, y.at(0), timescale_);
        
}



//this is the function to integrate
template <typename CurveM>
double ElasticTendon<CurveM>::computeFiberVelocityAtT(double muscleTendonLengthAtT, double activationAtT, double muscleTendonVelocityAtT, double fiberLength, double lmtTimescale) {
    
    
    double optimalFiberLengthAtT = optimalFiberLength_ * (percentageChange_ * (1.0 - activationAtT) + 1.0);
    double normFiberLength = fiberLength / optimalFiberLengthAtT;
    double pennationAngleAtT = PennationAngle::compute(fiberLength, optimalFiberLength_, pennationAngle_);
    double ca = cos(radians(pennationAngleAtT));
    tendonLength_ = muscleTendonLengthAtT - fiberLength*ca;
    double tendonStrain = (tendonLength_ - tendonSlackLength_)/tendonSlackLength_;
    double normTendonForce = tendonForceStrainCurveM_.getValue(tendonStrain);
    double fa = activeForceLengthCurveM_.getValue(normFiberLength);
    
    double normFiberVelocity = 0.0;
    double velocityDependentForce = 0.0;
        
    /* If pennation equals 90 degrees, fiber length equals muscle width and fiber
    * velocity goes to zero.  Pennation will stay at 90 until tendon starts to
    * pull, then "stiff tendon" approximation is used to calculate approximate
    * fiber length.
    */
    
   if ((tendonLength_ < tendonSlackLength_) && (ca> 0)) {
 //      cout << id_ << " tendonLength < tendonSlackLength " << endl << tendonLength << " " << tendonSlackLength_ << endl;
       double first = optimalFiberLengthAtT * sin( radians(pennationAngleAtT));
            double second = muscleTendonLength_ - tendonSlackLength_;
            double fiberLength = sqrt(first * first + second * second); 
            pennationAngleAtT = PennationAngle::compute(fiberLength, optimalFiberLengthAtT, pennationAngle_);
            ca = cos(radians(pennationAngleAtT));
            normFiberVelocity = lmtTimescale*(muscleTendonVelocityAtT/optimalFiberLengthAtT) * ca;
#ifdef DEBUG
   //            if(id_ == "gaslat_r")
               {
                    cout << id_ << " computeFiberVelocityAtT@Z1 \n";
    //                cout << "tendonlength " << tendonLength << endl;
               }
#endif
      
//        normFiberVelocity = 100;
    }
    
    
    else if (fabs(ca) < 0.1) {
        if ( fabs(normTendonForce) < 0.1 )  // 0.01 means close to zero, could be changed
        {
 #ifdef DEBUG
  //              if(id_ == "gaslat_r")
                    cout << id_ << " computeFiberVelocityAtT@Z2 \n";
#endif   
            normFiberVelocity = 0.0;
        }
        else {
            double first = optimalFiberLengthAtT * sin( radians(pennationAngleAtT));
            double second = muscleTendonLength_ - tendonSlackLength_;
            double fiberLength = sqrt(first * first + second * second); 
            pennationAngleAtT = PennationAngle::compute(fiberLength, optimalFiberLengthAtT, pennationAngle_);
            ca = cos(radians(pennationAngleAtT));
            normFiberVelocity = lmtTimescale*(muscleTendonVelocityAtT/optimalFiberLengthAtT) * ca;    
            #ifdef DEBUG
 //              if(id_ == "gaslat_r")
                    cout << id_ << " computeFiberVelocityAtT@Z3 \n";
#endif
        }
    }
    else {
        double fp = passiveForceLengthCurveM_.getValue(normFiberLength);
        velocityDependentForce = normTendonForce/ca - fp;
        normFiberVelocity = computeNormFiberVelocity(activationAtT, fa, velocityDependentForce);
#ifdef DEBUG
 //              if(id_ == "gaslat_r")
                    cout << id_ << " computeFiberVelocityAtT@Z4 \n";
//                cout << "activationAtT, fa, velocityDependentForce, pennationAngleAtT\n";
//                cout << activationAtT << " " << fa << " " << velocityDependentForce << " " << pennationAngleAtT <<  endl;
#endif
    }
        /* Un-normalize the muscle state derivatives */
#ifdef DEBUG
/*          if(id_ == "bicfemsh_r") {
    cout << testTime_ << "\nnormFiberVelocity optimalFiberLengthAtT lmtTimescale\n";
    cout << normFiberVelocity << " " << optimalFiberLengthAtT  << " " << lmtTimescale << endl;
    cout << (normFiberVelocity * optimalFiberLengthAtT / lmtTimescale) << endl;}*/
#endif
    return (normFiberVelocity * optimalFiberLengthAtT / lmtTimescale);
    
}


template <typename CurveM>
double ElasticTendon<CurveM>::computeNormFiberVelocity(double activation, double activeForce, double velocityDependentForce) {
    
    double b, c, fiberVelocity;

    double kv = 0.25, slope_k = 0.13, fmax = 1.8;

    //this equation are in Schutte phd thesis. 

//    if (velocityDependentForce < -damping_)
//        fiberVelocity = velocityDependentForce / damping_;
    if (velocityDependentForce < activation*activeForce) {
        c = kv * (velocityDependentForce - activation*activeForce) / damping_;
        b = -kv * (velocityDependentForce/kv + activation*activeForce + damping_) / damping_;
  //      cout << "a. " << b*b-4.0*c << endl;
        fiberVelocity = (-b-sqrt(b*b-4.0*c)) / 2.0;

    }
    else {
        c = -(slope_k*kv/((damping_*(kv+1))))* (velocityDependentForce - activation*activeForce);
        b = -(velocityDependentForce/damping_ - fmax*activation*activeForce/damping_-slope_k*kv/(kv+1));

   //     cout << "b. " << b*b-4.0*c << endl;
        // fiber_velocity = (-b+sqrt(b*b-4.0*c)) / 2.0;

        fiberVelocity = -2*c/(b+sqrt(b*b-4.0*c));

    }

   // cout << "activation " << activation << " activeForce " << activeForce << " velocityDependentForce " << velocityDependentForce << endl;
    return fiberVelocity;
}

template <typename CurveM>
ElasticTendon<CurveM>::~ElasticTendon() { }

// template class ElasticTendon<Curve<CurveMode::Online> >;
// template class ElasticTendon<Curve<CurveMode::Offline> >;
