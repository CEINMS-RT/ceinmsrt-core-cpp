#ifndef MTUSPLINEINTERFACE_H_
#define MTUSPLINEINTERFACE_H_

#ifdef WIN32
#include <windows.h>
#endif

#include <string>
#include <vector>
#include "MTUSplineDataRead.h"
#include "ExecutionXmlReader.h"
#include "executionIK_ID.hxx"
//#include <boost/shared_ptr.hpp>
#include <memory>
#include "NMSmodel.h"

using namespace std;

class MTUSplineInterface
{
public:
	MTUSplineInterface();
	MTUSplineInterface(const std::string& subjectSpecificXml, /*!< CEINMS specific XML */const std::string& subjectName /*!< Name of the subject */);
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


protected:

	// Template function for Spline computation with order > 1 
	template<class T>
	void computeLmtMafromSplines(
		T& splines, int dim,
		const std::vector<double>& angles, std::vector<double>& lmt,
		std::vector<std::vector<double> >& ma
	);

	// Template function for Spline conputation with order 1
	void computeLmtMafromSplines(
		std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
		const std::vector<double>& angles, std::vector<double>& lmt,
		std::vector<std::vector<double> >& ma
	);

	std::vector<std::vector<std::string> > 	musclesNamesOnDof_; /*!< Name of the muscle on the different DOF from the model */
	std::vector<std::string> 				dofNames_; 			/*!< Dof name from the model */
	std::vector<std::string>				muscleNames_;
	std::string 							subjectSpecificXml_;/*!< CEINMS subject XML */
	std::string 							subjectName_;		/*!< name of the subject */
	std::string 							executionName_;		/*!< name of the execution XML */
	std::vector<std::vector<double> >		ma_;
	std::vector<double>						lmt_;
	std::vector<MTUSplineDataRead::Task>	taskMTU_;
	int										noMuscles_;
	MTUSplineDataRead*						splineData_;
};

#endif