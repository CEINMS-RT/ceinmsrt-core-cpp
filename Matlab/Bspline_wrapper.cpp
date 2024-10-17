

/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <MTUSplineInterface.h>

MTUSplineInterface* MTUInter;
std::vector<std::string> DOFName;
std::vector<std::string> MuscleName;
std::vector<std::vector<std::string> > musclesNamesOnDof;

/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 32
#define y_width 1
/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
 void start_wrapper(const int8_T  *XMLName, const int_T  p_width0,
			const int8_T  *UserName, const int_T  p_width1)
{
	char* subjectSpecificXmlChar = (char*)XMLName;
	char* subjectNameChar = (char*)UserName;
	std::string subjectSpecificXml = "cfg/TestData/gait2392Left.xml";
	std::string subjectName = "TestData";
	
	//std::string subjectSpecificXml(subjectSpecificXmlChar);
	//std::string subjectName(subjectNameChar);
	
	FILE *fp = NULL;
	fp = fopen(subjectSpecificXmlChar, "r");
	
    MTUInter = new MTUSplineInterface(subjectSpecificXml, subjectName);
	MTUInter->initialisationFromXML();
	DOFName = MTUInter->getDOFName();
	MuscleName = MTUInter->getMuscleName();
	musclesNamesOnDof = MTUInter->getMusclesNamesOnDof();

}

void end_wrapper()
{
    delete MTUInter;
}
 
 
void Bspline_Outputs_wrapper(const real_T *Angles,
			real_T *LMT,
			real_T *MA,
			const int8_T  *XMLName, const int_T  p_width0,
			const int8_T  *UserName, const int_T  p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
std::vector<double> position;
std::vector<std::vector<double> > ma;
std::vector<double> lmt;
for(int i = 0; i < DOFName.size(); i++)
{
    position.push_back(Angles[i]);
}
/*
for(int i = 0; i < 32; i++)
{
    LMT[i] = out;
    for(int w = 0; w < 32; w++)
    {
        MA[i*32+w] = out;
    }
}

*/
MTUInter->setPosition(position);
lmt = MTUInter->getLMT();
ma = MTUInter->getMA();

//if(lmt.size() > 32)
//	mexErrMsgIdAndTxt( "MATLAB:Bspline_wrapper:101", "Error lmt size bigger than output vector LMT (too many muscles in model > 32).");
for(std::vector<double>::const_iterator it = lmt.begin(); it != lmt.end(); it++)
{
    LMT[std::distance<std::vector<double>::const_iterator >(lmt.begin(), it)] = *it;
}

//if(ma.size() > 32)
//	mexErrMsgIdAndTxt( "MATLAB:Bspline_wrapper:108", "Error ma size bigger than output vector MA (too many DOF in model > 32).");
for(std::vector<std::vector<double> >::const_iterator it = ma.begin(); it != ma.end(); it++)
{
    //if(it->size() > 32)
	//	mexErrMsgIdAndTxt( "MATLAB:Bspline_wrapper:112", "Error ma DOF size bigger than output vector MA for this DOF(too many muscle in DOF in model > 32).");
    for(std::vector<double>::const_iterator it2 = it->begin(); it2 != it->end(); it2++)
    {
        int itDis = std::distance<std::vector<std::vector<double> >::const_iterator >(ma.begin(), it);
        int it2Dis = std::distance<std::vector<double>::const_iterator >(it->begin(), it2);
        MA[itDis*32+it2Dis] = *it2;
    }
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}
