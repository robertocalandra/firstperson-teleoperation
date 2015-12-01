#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class ICubController {

	typedef struct moveData {
		int ID;
		double angle;
	};


public: 

	static const int MOVE_LEFT_ARM = 0;
	static const int MOVE_RIGHT_ARM = 1;
	static const int MOVE_HEAD = 2;
	static const int MOVE_TORSO = 3;

	ICubController();

	int close();
	int move(int bodyPart, double* refs);
	int move(int bodyPart, int jointId, double angle);
private:
	void initialize();
	void initializeDevice(int bodyPart, string remotePorts, string localPorts, string device);
	void initializeVelocity(int bodyPart);
};