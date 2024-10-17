#include <correlation.hpp>
#include <stdexcept>
#include <math.h>
	
double calcCorrelation(const std::vector<double>& dataA, const std::vector<double>& dataB){

    if(dataA.size() != dataB.size()){
        throw std::runtime_error("Sizes must be the same for calculation of Correlation. Resample data first.");
    }
    double n = (double) dataA.size();
    double sumOfXY = 0;
    double sumOfX = 0;
    double sumOfY = 0;
    double sumOfX2 = 0;
    double sumOfY2 = 0;


    for(unsigned int idx = 0; idx < dataA.size(); idx++){
        sumOfXY += dataA[idx] * dataB[idx];
        sumOfX += dataA[idx];
        sumOfY += dataB[idx];
        sumOfX2 += dataA[idx] * dataA[idx];
        sumOfY2 += dataB[idx] * dataB[idx];
    }


    double correlationCoeff = (n * sumOfXY - sumOfX * sumOfY) / sqrt( ( n * sumOfX2 - sumOfX * sumOfX ) * (n * sumOfY2 - sumOfY * sumOfY) );

    return correlationCoeff;
}