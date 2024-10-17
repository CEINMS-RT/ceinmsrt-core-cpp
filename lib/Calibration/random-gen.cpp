//this generator use as: 
// int ij = 1802, kl = 9373;  rmarin(ij,kl); (double)ranmar();
 #include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <float.h>
 #include <assert.h>
 //#include <ieeefp.h>
#include <errno.h>
 #include "random-gen.h"

 
#define RANMAR_TRUE -1
#define RANMAR_FALSE 0
 #define boolean int


 static double u[98], c, cd, cm;
 static long int i97, j97;
 static boolean test = RANMAR_FALSE;
 
 void rmarin(int ij,int kl)
 {
 /*
00062 C This is the initialization routine for the random number generator RANMAR()
00063 C NOTE: The seed variables can have values between:    0 <= IJ <= 31328
00064 C                                                      0 <= KL <= 30081
00065 C The random number sequences created by these two seeds are of sufficient 
00066 C length to complete an entire calculation with. For example, if sveral 
00067 C different groups are working on different parts of the same calculation,
00068 C each group could be assigned its own IJ seed. This would leave each group
00069 C with 30000 choices for the second seed. That is to say, this random 
00070 C number generator can create 900 million different subsequences -- with 
00071 C each subsequence having a length of approximately 10^30.
00072 C 
00073 C Use IJ = 1802 & KL = 9373 to test the random number generator. The
00074 C subroutine RANMAR should be used to generate 20000 random numbers.
00075 C Then display the next six random numbers generated multiplied by 4096*4096 
00076 C If the random number generator is working properly, the random numbers
00077 C should be:
00078 C           6533892.0  14220222.0  7275067.0
00079 C           6172232.0  8354498.0   10633180.0
00080 */
 
     long int i, j, k, l, ii, jj, m;
         double s, t;
         if (ij<0 || ij>31328 || kl<0 || kl>30081) 
         {
                 puts("The first random number seed must have a value between 0 and 31328.");
                 puts("The second seed must have a value between 0 and 30081.");
                 exit(1);
         }
         i = (ij/177)%177 + 2;
         j = ij%177 + 2;
         k = (kl/169)%178 + 1;
         l = kl%169;
 
         for (ii=1; ii<=97; ii++) 
         {
                 s = 0.0;
                 t = 0.5;
                 for (jj=1; jj<=24; jj++) 
                 {
                         m = (((i*j)%179)*k) % 179;
                         i = j;
                         j = k;
                         k = m;
                         l = (53*l + 1) % 169;
                         if ((l*m)%64 >= 32) s += t;
                         t *= 0.5;
                 }
                 u[ii] = s;
         }
         c = 362436.0 / 16777216.0;
         cd = 7654321.0 / 16777216.0;
         cm = 16777213.0 / 16777216.0;
         i97 = 97;
         j97 = 33;
         test = RANMAR_TRUE;
 }
 
 static long int nranmar = 0;                      /* number of calls to ranmar */
 
 /********************/
 double ranmar( void )
 /********************/
 /*
 C This is the random number generator proposed by George Marsaglia in 
  Florida State University Report: FSU-SCRI-87-50
 C It was slightly modified by F. James to produce an array of pseudorandom
 C numbers.
 */
 {
         double uni;
         if (test==RANMAR_FALSE) 
         {
                 puts("Call the init routine rmarin() before calling ranmar().");
exit(2);
         }
         nranmar++;
         uni = u[i97] - u[j97];
         if (uni < 0.0) uni += 1.0;
         u[i97] = uni;
         i97--;
         if (i97==0) i97 = 97;
         j97--;
         if (j97==0) j97 = 97;
         c -= cd;
         if (c<0.0) c += cm;
         uni -= c;
         if (uni<0.0) uni += 1.0;
         return uni;
 }
 
 /* 
00153 C This random number generator originally appeared in "Toward a Universal 
00154 C Random Number Generator" by George Marsaglia and Arif Zaman. 
00155 C Florida State University Report: FSU-SCRI-87-50 (1987)
00156 C 
00157 C It was later modified by F. James and published in "A Review of Pseudo-
00158 C random Number Generators" 
00159 C 
00160 C THIS IS THE BEST KNOWN RANDOM NUMBER GENERATOR AVAILABLE.
00161 C       (However, a newly discovered technique can yield 
00162 C         a period of 10^600. But that is still in the development stage.)
00163 C
00164 C It passes ALL of the tests for random number generators and has a period 
00165 C   of 2^144, is completely portable (gives bit identical results on all 
00166 C   machines with at least 24-bit mantissas in the floating point 
00167 C   representation). 
00168 C 
00169 C The algorithm is a combination of a Fibonacci sequence (with lags of 97
00170 C   and 33, and operation "subtraction plus one, modulo one") and an 
00171 C   "arithmetic sequence" (using subtraction).
00172 C======================================================================== 
00173 This C language version was written by Jim Butler, and was based on a
00174 FORRAN program posted by David LaSalle of Florida State University.
00175 */
