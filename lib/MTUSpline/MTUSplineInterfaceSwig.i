%module MTUSplineInterfaceSwig
%{
#include "MTUSplineInterface.h"
%}

%include "std_vector.i"
%include "std_string.i"

using namespace std;

class MTUSplineInterface
{
public:
	MTUSplineInterface();
	MTUSplineInterface(const std::string& subjectSpecificXml, const std::string& subjectName );
	~MTUSplineInterface();

	vector< vector<double> > getMA();
	vector<double> getLMT();
	void setPosition(vector<double> position);
	void initialisation();
	void initialisationFromXML();
	void setDOFName(vector<string> DofName);
	void setMusclesNamesOnDof(std::vector<std::vector<std::string> > 	musclesNamesOnDof);
	void setMuscleName(std::vector<std::string> muscleNames);
	std::vector<std::string> getMuscleName();
	std::vector<std::vector<std::string> > getMusclesNamesOnDof();
	vector<string> getDOFName();
};


namespace std {
   %template(vectors) vector<string>;
   %template(vectord) vector<double>;
   %template(vectordd) vector<vector<double> >;
   %template(vectorss) vector<vector<string> >;
};