#ifndef RANDOM_H

#define RANDOM_H
#include "parameters.h"
 
 //ranmar routines
 void rmarin(int ij, int kl);
 double ranmar( void );
 
 struct rand_wrapper
 {
 private:
 //      double last_random;
 public:
         rand_wrapper(int seed=0)
         {
                 int ij = 1802+7*seed, kl = 9373-seed*3;  
                 rmarin(ij,kl);
                 double r1=ranmar();
                 double r2=ranmar();
                 assert(r1!=r2);
         //      std::cout<<"\nRandom number generator with \nthe seeds and first two numbers (ij,kl,rand1,rand2)\n"
         //              <<ij<<' '<<kl<<' '<<r1<<' '<<r2<<std::endl;
         }
         inline double random_number() { return ranmar(); }
         inline void get_random_numbers(int n, std::vector<double> &vrand) 
         {
                 int i;
                 vrand.resize(n);
                 for(i=0;i<n;i++)        
                 {
                         vrand[i]=ranmar();
         //              std::cout<<vrand[i]<<' ';
                 }
         }
         //tested
         inline void get_random_vector(double &x,double &y,double &z,double radius,bool variable_r)//uniform x,y,z distribution on and inside of sphere
         {
                 double rho;
                 double r=1;
                 if(variable_r) 
                 {       rho=random_number();
                         r=pow(rho,1/3.);
                 }
                 double cost=2*random_number()-1;
                 double fi=table::TWOPI*random_number();
                 
                 r*=radius;
                 double rsint=r*sqrt(1-SQR(cost));
 
                 x=rsint*cos(fi);
                 y=rsint*sin(fi);
                 z=r*cost;
         }
 };
 
 #endif