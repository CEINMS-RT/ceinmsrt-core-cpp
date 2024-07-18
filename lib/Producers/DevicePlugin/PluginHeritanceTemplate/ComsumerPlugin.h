#ifndef COMSUMERPLUGIN_H_
#define COMSUMERPLUGIN_H_

#include <string>
#include <map>
#include <vector>
#include <set>
#include <string>

class ComsumerPlugin
{
	public:
		ComsumerPlugin() {};
		~ComsumerPlugin() {};
		virtual void init ( int portno ) = 0;
		virtual void setMuscleName ( const std::vector<std::string>& muscleName ) = 0;
		virtual void setMuscleForce ( const std::vector<double>& muscleForce ) = 0;
		virtual void setDofName ( const std::vector<std::string>& dofName ) = 0;
		virtual void setDofTorque ( const std::vector<double>& dofTorque ) = 0;
		virtual void setTimeStamp ( const double& timeStamp ) = 0;
		virtual void spin() = 0;
		virtual void stop() = 0;
		virtual void setDirectory ( std::string outDirectory, std::string inDirectory = std::string() ) = 0;
		virtual void setVerbose ( int verbose ) = 0;
		virtual void setRecord ( bool record ) = 0;
};

typedef ComsumerPlugin* create_p();
typedef void destroy_p ( ComsumerPlugin* );

#endif /* COMSUMERPLUGIN_H_ */
