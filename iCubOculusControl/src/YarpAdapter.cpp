/*
*   This file is part of firstperson-telecontrol.
*
*    firstperson-telecontrol is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    firstperson-telecontrol is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with firstperson-telecontrol.  If not, see <http://www.gnu.org/licenses/>.
*
*	 Authors: Lars Fritsche, Felix Unverzagt, Roberto Calandra
*	 Created: July, 2015
*/

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
	imagePort[0].open("/oculusController/image_right_in/in");  // give the port a name
	imagePort[1].open("/oculusController/image_right_in/in");  // give the port a name
	outputPort.open("/oculusController/oculus_raw/out");
	Network::connect("/icub/camcalib/left/out", "/icub/image_left/in", "udp", true);
	Network::connect("/icub/camcalib/right/out", "/icub/image_right/in", "udp", true);
}

void YarpAdapter::initializeVelocity(int bodyPart)
{
	/*switch (bodyPart) {
	case MOVE_LEFT_ARM:
	case MOVE_RIGHT_ARM:
		positionControl[bodyPart]->setRefSpeed(0, 80);
		positionControl[bodyPart]->setRefSpeed(1, 80);
		positionControl[bodyPart]->setRefSpeed(2, 80);
		positionControl[bodyPart]->setRefSpeed(3, 80);
		return;
	case MOVE_TORSO:
		positionControl[bodyPart]->setRefSpeed(0, 50);
		positionControl[bodyPart]->setRefSpeed(1, 50);
		positionControl[bodyPart]->setRefSpeed(2, 50);
		return;
	case MOVE_HEAD:
		positionControl[bodyPart]->setRefSpeed(0, 50);
		positionControl[bodyPart]->setRefSpeed(1, 50);
		positionControl[bodyPart]->setRefSpeed(2, 50);
		positionControl[bodyPart]->setRefSpeed(3, 20);
		positionControl[bodyPart]->setRefSpeed(4, 20);
		positionControl[bodyPart]->setRefSpeed(5, 20);
		return;
	}*/
}

void YarpAdapter::initializeGains(int bodyPart) {
	Pid* pids = readGains(bodyPart);
	double gainDiscount = 0.8;

	switch (bodyPart) {
	case MOVE_LEFT_ARM:
	case MOVE_RIGHT_ARM:
		pidControl[bodyPart]->setPid(0, changePid(pids[0], gainDiscount));
		pidControl[bodyPart]->setPid(1, changePid(pids[1], gainDiscount));
		pidControl[bodyPart]->setPid(2, changePid(pids[2], gainDiscount));
		pidControl[bodyPart]->setPid(3, changePid(pids[3], gainDiscount));
		pidControl[bodyPart]->setPid(4, changePid(pids[4], gainDiscount));
		return;
	case MOVE_TORSO:
		pidControl[bodyPart]->setPid(0, changePid(pids[0], gainDiscount));
		pidControl[bodyPart]->setPid(1, changePid(pids[1], gainDiscount));
		pidControl[bodyPart]->setPid(2, changePid(pids[2], gainDiscount));
		return;
	case MOVE_HEAD:
		pidControl[bodyPart]->setPid(0, changePid(pids[0], gainDiscount));
		pidControl[bodyPart]->setPid(1, changePid(pids[1], gainDiscount));
		pidControl[bodyPart]->setPid(2, changePid(pids[2], gainDiscount));
		pidControl[bodyPart]->setPid(3, changePid(pids[3], gainDiscount));
		pidControl[bodyPart]->setPid(4, changePid(pids[4], gainDiscount));
		pidControl[bodyPart]->setPid(5, changePid(pids[5], gainDiscount));
		return;
	}
}

Pid YarpAdapter::changePid(Pid pid, double discount) {
	ofstream myfile;
	myfile.open("right_arm_pid.txt", ios::app);
	myfile << "new " << pid.kp << " " << pid.kd << pid.ki << " ";
	myfile.close();

	pid.kd = pid.kd;
	pid.kp = pid.kp * discount;

	myfile.open("right_arm_pid.txt", ios::app);
	myfile << "old " << pid.kp << " " << pid.kd << pid.ki << "\n";
	myfile.close();

	return pid;
}

Pid* YarpAdapter::readGains(int bodyPart) {
	Pid pidsLArm[16];
	Pid pidsRArm[16];

	Pid pidsTorso[3];
	Pid pidsHead[6];
	switch (bodyPart) {
	case MOVE_LEFT_ARM:
		pidControl[MOVE_LEFT_ARM]->getPids(pidsLArm);
		return pidsLArm;
	case MOVE_RIGHT_ARM:
		pidControl[MOVE_RIGHT_ARM]->getPids(pidsRArm);
		return pidsRArm;
	case MOVE_HEAD:
		pidControl[MOVE_HEAD]->getPids(pidsHead);
		return pidsHead;
	case MOVE_TORSO:
		pidControl[MOVE_TORSO]->getPids(pidsTorso);
		return pidsTorso;
	}
		
	return NULL;
	
}

void YarpAdapter::initialize()
{
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

int YarpAdapter::move(int bodyPart, int jointId, double angle)
{
//		positionControl[bodyPart]->positionMove(jointId, angle);
	return 0;
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

int YarpAdapter::move(int bodyPart, double *refs)
{
	bool success = positionControl[bodyPart]->setPositions(refs);

	return 0;
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

