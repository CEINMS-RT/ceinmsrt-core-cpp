#include <OpenSim/OpenSim.h>
#include <cmath>
using namespace OpenSim;

int main()
{
	try
	{
		// Create an OpenSim model and set its name
		Model osimModel;
		osimModel.setName ( "tugOfWar" );

		double mass = 2.0, radius = 0.0765, heigth = 0.01;
		SimTK::Vec3 massCenter ( 0, 0, 0 );
		SimTK::Inertia inertia = mass * SimTK::Inertia::cylinderAlongX ( radius, heigth );
		// Create a new block body with specified properties
		OpenSim::Body* cylinder = new OpenSim::Body ( "cylinder", mass, massCenter, inertia );
		// Add display geometry to the block to visualize in the GUI
		cylinder->addDisplayGeometry ( "cylinder.vtp" );
		cylinder->updDisplayer()->setScaleFactors(SimTK::Vec3(radius, heigth, radius));
		SimTK::Rotation rotation(M_PI/2, SimTK::CoordinateAxis::ZCoordinateAxis());
		SimTK::Transform transform(rotation);
		SimTK::Vec3 locationInParent(0, 0, -0.01), orientationInParent(0), locationInBody(0), orientationInBody(0);
		OpenSim::Body& ground = osimModel.getGroundBody(); 
		WeldJoint *cylinderToGround = new WeldJoint("cylinderToGround", ground, locationInParent, orientationInParent, *cylinder, locationInBody, orientationInBody);
		CoordinateSet& jointCoordinateSet = cylinderToGround->upd_CoordinateSet();
		//cylinder->updDisplayer()->setTransform(transform);
		osimModel.addBody(cylinder);
		osimModel.print("tugOfWar_model.osim");
	}
	catch ( OpenSim::Exception ex )
	{
		std::cout << ex.getMessage() << std::endl;
		return 1;
	}

	std::cout << "OpenSim example completed successfully.\n";
	return 0;
}
