#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/PortWriter.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <YarpAdapter.h>
#include <Windows.h>
#define NO_STDIO_REDIRECT

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


const double pi = 3.141592654;
const double g = 9.81;


Network yarpNet;

Property params;
std::string robotName;

IControlMode *controlMode[4];
ITorqueControl *torque[4];
IPositionDirect *positionControl[4];
IEncoders *encoders[4];
IPidControl *pidControl[4];

BufferedPort<ImageOf<PixelRgb>> imagePort[2];
BufferedPort<Bottle>outputPort;

ImageOf<PixelRgb> *image;

PolyDriver robotDevice[4];


typedef struct moveData {
	int ID;
	double angle;
} Single;

/// <summary>
/// Constructor
/// </summary>
YarpAdapter::YarpAdapter() {
	initialize();
}

YarpAdapter::~YarpAdapter() {
	outputPort.close();
}

// register YARP grabber 
void YarpAdapter::initializeDevice(int bodyPart, string remotePorts, string localPorts, string device)
{
	Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPorts.c_str());   //local port names
	options.put("remote", remotePorts.c_str());         //where we connect to

	// create a device
	robotDevice[bodyPart].open(options);
	if (!robotDevice[bodyPart].isValid()) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		string devices = Drivers::factory().toString().c_str();
		return;
	}

	bool ok;
	ok = robotDevice[bodyPart].view(controlMode[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(torque[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(positionControl[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(encoders[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(pidControl[bodyPart]);

	if (!ok) {
		printf("Problems acquiring interfaces\n");
		return;
	}
}

void YarpAdapter::initializePorts() {
	imagePort[0].open("/oculusController/image_left_in/in"); 
	imagePort[1].open("/oculusController/image_right_in/in");  
	outputPort.open("/oculusController/oculus_raw/out");
	Network::connect("/icub/camcalib/left/out", "/oculusController/image_left_in/in", "udp", true);
	Network::connect("/icub/camcalib/right/out", "/oculusController/image_right_in/in", "udp", true);
}

void YarpAdapter::initialize()
{
	// important!!! has to be set for gazebo 
	robotName = "/icubGazeboSim";
	//robotName = "/icub";

	string remoteLeftArm = robotName + "/left_arm";
	string remoteRightArm = robotName + "/right_arm";
	string remoteTorso = robotName + "/torso";
	string remoteHead = robotName + "/head";

	string localPortsLeftArm = "/positionModule/left_arm";
	string localPortsRightArm = "/positionModule/right_arm";
	string localPortsTorso = "/positionModule/torso";
	string localPortsHead = "/positionModule/head";

	//initializeDevice(MOVE_LEFT_ARM, remoteLeftArm, localPortsLeftArm, "left_arm_controller");
	//initializeDevice(MOVE_RIGHT_ARM, remoteRightArm, localPortsRightArm, "right_arm_controller");
	//initializeDevice(MOVE_TORSO, remoteTorso, localPortsTorso, "torso_controller");
	initializeDevice(MOVE_HEAD, remoteHead, localPortsHead, "head_controller");

	//initializeVelocity(MOVE_LEFT_ARM);
	//initializeVelocity(MOVE_RIGHT_ARM);
	//initializeVelocity(MOVE_TORSO);
	//initializeVelocity(MOVE_HEAD);

	initializePorts();

	return;
}

unsigned char * YarpAdapter::getImage(int camera) {
	image = imagePort[camera].read();  // read an image
	if (image != NULL) { // check we actually got something
		printf("We got an image of size %dx%d\n", image->width(), image->height());
		int width = image->width(); 
		int height = image->height();
	}
	return image->getRawImage();
}

double* YarpAdapter::getJoints(int bodyPart) {
	double* values = new double[6];
	encoders[bodyPart]->getEncoders(values);
	return values;
}

int YarpAdapter::close()
{
	//robotDevice[0].close();
	//robotDevice[1].close();
	robotDevice[2].close();
	//robotDevice[3].close();

	return 0;
}

void YarpAdapter::writeToOutput(int size, double* angles) {
	Bottle& out = outputPort.prepare();
	out.clear();
	for (int i = 0; i < size; i++) {
		out.addDouble(angles[i]);
	}
	printf("X: %d \n", angles[0]);
	int isize = out.size();
	outputPort.write();
}

