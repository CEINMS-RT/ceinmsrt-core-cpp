//#define BOOST_TEST_DYN_LINK
#ifdef UNIX
	#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE BsplineTest
#include <boost/test/unit_test.hpp>
#include "MTUSplineInterface.h"
#include "DataFromFile.h"

#ifdef WINDOWS
#include <direct.h>
#else
#include <unistd.h>
#endif

BOOST_AUTO_TEST_CASE(Bspline)
{

#ifdef UNIX
	if (access("cfg/TestData/data/ik.sto", F_OK) == -1)
		chdir("..");
#endif

#ifdef WIN32
	if (GetFileAttributes("cfg/TestData/data/ik.sto") == 0xFFFFFFFF)
		SetCurrentDirectory("..");
#endif

	std::string subjectSpecificXml = "cfg/TestData/gait2392Left.xml";
	std::string subjectName = "TestData";
	MTUSplineInterface test(subjectSpecificXml, subjectName);
	test.initialisationFromXML();
	std::vector<std::vector<double> > position, OpenSimMA, OpenSimLmt, ma;
	std::vector<double> lmt;
	std::string ikName = "cfg/TestData/data/ik.sto";

	// Read IK, LMT and ma from filesmake test




	DataFromFile ikData("cfg/TestData/Data2/angles_0.sto");
	DataFromFile maData("cfg/TestData/3DGaitModel2392-scaled_MuscleAnalysis_MomentArm_ankle_angle_l.sto");
	DataFromFile lmtData("cfg/TestData/3DGaitModel2392-scaled_MuscleAnalysis_Length.sto");



	std::vector<std::string> musclesNameMAOpensim = maData.getColumnNames();
	std::vector<std::string> musclesNameLMTOpenSim = lmtData.getColumnNames();
	std::vector<std::string> dofNameIK = ikData.getColumnNames();

	std::vector<std::string> DOFName = test.getDOFName();
	std::vector<std::string> MuscleName = test.getMuscleName();
	std::vector<std::vector<std::string> > musclesNamesOnDof = test.getMusclesNamesOnDof();



	std::vector<double> itReorganizeLMT;
	for (std::vector<std::string>::iterator it = MuscleName.begin(); it != MuscleName.end(); it++)
	{
		std::vector<std::string>::iterator temp = std::find(musclesNameLMTOpenSim.begin(), musclesNameLMTOpenSim.end(), *it);
		itReorganizeLMT.push_back(std::distance(musclesNameLMTOpenSim.begin(), temp));
	}



	std::vector<std::string>::iterator itMA = std::find(DOFName.begin(), DOFName.end(), "ankle_angle_l");
	if (itMA == DOFName.end())
	{
		std::cout << "Error ankle_angle_l not find in MA from BSpline" << std::endl;
		exit(0);
	}



	int itMaDist = std::distance(DOFName.begin(), itMA);

	std::vector<double> itReorganizeMA;
	for (std::vector<std::string>::iterator it = musclesNamesOnDof.at(itMaDist).begin(); it != musclesNamesOnDof.at(itMaDist).end(); it++)
	{
		std::vector<std::string>::iterator temp = std::find(musclesNameMAOpensim.begin(), musclesNameMAOpensim.end(), *it);
		itReorganizeMA.push_back(std::distance(musclesNameMAOpensim.begin(), temp));
	}



	while (ikData.areStillData())
	{
		std::vector<double> temp;
		ikData.readNextData();
		temp = ikData.getCurrentData();
		position.push_back(temp);
	}


	while (maData.areStillData())
	{
		// Get the Data
		std::vector<double> temp, maStateReorg;
		maData.readNextData();
		temp = maData.getCurrentData();
		// The Data order and number of muscle is not the same
		// so it need to be reorganize
		for (std::vector<double>::const_iterator it = itReorganizeMA.begin(); it != itReorganizeMA.end(); it++)
			maStateReorg.push_back(temp.at(*it));
		OpenSimMA.push_back(maStateReorg);
	}

	while (lmtData.areStillData())
	{
		// Get the Data
		std::vector<double> temp, lmtStateReorg;
		lmtData.readNextData();
		temp = lmtData.getCurrentData();
		// The Data order and number of muscle is not the same
		// so it need to be reorganize
		for (std::vector<double>::const_iterator it = itReorganizeLMT.begin(); it != itReorganizeLMT.end(); it++)
			lmtStateReorg.push_back(temp.at(*it));
		OpenSimLmt.push_back(lmtStateReorg);
	}


	for (std::vector<std::vector<double> >::const_iterator it1 = position.begin(); it1 != position.end(); it1++)
	{
		//std::cout << it1->at(0) << std::endl;
		//std::cout << it1->at(1) << std::endl;
		//std::cout << it1->at(2) << std::endl;
		test.setPosition(*it1);
		lmt = test.getLMT();
		ma = test.getMA();
		int cpt = std::distance<std::vector<std::vector<double> >::const_iterator >(position.begin(), it1);

		//std::cout << lmt.size() << std::endl;

		BOOST_REQUIRE_EQUAL(lmt.size(), OpenSimLmt.at(cpt).size());
		BOOST_REQUIRE_EQUAL(ma.at(itMaDist).size(), OpenSimMA.at(cpt).size());
		for (std::vector<double>::const_iterator it2 = lmt.begin(); it2 != lmt.end(); it2++)
		{
			//std::cout << MuscleName.at(std::distance<std::vector<double>::const_iterator>(lmt.begin(), it2)) << ":" << *it2 << "\t";
			BOOST_CHECK(abs(*it2 - OpenSimLmt.at(cpt).at(std::distance<std::vector<double>::const_iterator>(lmt.begin(), it2))) < 0.001);
			//break;
		}

		for (std::vector<double>::const_iterator it2 = ma.at(itMaDist).begin(); it2 != ma[itMaDist].end(); it2++)
		{
			//std::cout << musclesNamesOnDof.at(itMaDist).at(std::distance<std::vector<double>::const_iterator>(ma.at(itMaDist).begin(), it2)) << ":" << *it2 << "\t";
			BOOST_CHECK(abs(*it2 - OpenSimMA.at(cpt).at(std::distance<std::vector<double>::const_iterator>(ma.at(itMaDist).begin(), it2))) < 0.005);
		}
		//std::cout << std::endl;
	}
}