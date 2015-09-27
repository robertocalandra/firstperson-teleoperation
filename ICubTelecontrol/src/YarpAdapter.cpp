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
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <YarpAdapter.h>
#include <vector>
#include <Vector.h>
#include <iostream>
#include <GloveAdapter.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

const double pi = 3.141592654;
const double g = 9.81;

Network yarpNet;

Property params;
std::string robotName;

IControlMode *controlMode[4];
ITorqueControl *torque[4];
IPositionDirect *positionDirect[4];
IPositionControl *positionControl[4];
IEncoders *encoders[4];

BufferedPort<Bottle> inPort[3];
BufferedPort<Bottle> outPort;

bool kinectInitialized = false;
bool oculusInitialized = false;
bool gloveInitialized = false;

double* oldOculusData;
vector<Vector> oldKinectData;
double* oldGloveData = new double[5];

int bodyCount = 0;
bool leftHandClosed = false;
bool rightHandClosed = false;

//int activatedJoints[] = { 0, 1, 2, 3 };
vector<vector<bool>> activatedJoints(YarpAdapter::NUM_OF_MOVE);
bool activated[] = { true, true, true, true, true, true};
bool useDirect[] = { true, true, true, true, true, true};
int iCubMapping[] = {0, 1, 2, 3, 0, 1};

PolyDriver robotDevice[4];

int dof[] = { 16, 16, 6, 3 , 16, 16};
double iCubBuffer[16];

/// <summary>
/// Constructor
/// </summary>
YarpAdapter::YarpAdapter(bool kinectActivated,bool oculusActivated,bool gloveActivated) {
	initialize(kinectActivated,oculusActivated,gloveActivated);
	setMode(false);
}
YarpAdapter::~YarpAdapter() {
	setMode(true);
}

void YarpAdapter::setMode(bool finished) {
	for (int i = 0; i < 6; i++) {
		if (!activated[i])
			continue;

		if (useDirect[i] && !finished)
			positionDirect[iCubMapping[i]]->setPositionDirectMode();
		else
			positionControl[i]->setPositionMode();
	}
}

void YarpAdapter::initializeDevice(int bodyPart, string remotePorts,
		string localPorts, string device) {
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
	ok = ok && robotDevice[bodyPart].view(positionDirect[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(positionControl[bodyPart]);
	ok = ok && robotDevice[bodyPart].view(encoders[bodyPart]);

	if (!ok) {
		printf("Problems acquiring interfaces\n");
		return;
	}
}

void YarpAdapter::initializePorts(bool kinectActivated,bool oculusActivated,bool gloveActivated) {
	inPort[OCULUS_IN].open("/telecontroller/oculus/in");
	inPort[KINECT_IN].open("/telecontroller/kinect/in");
	inPort[GLOVE_IN].open("/telecontroller/glove/in");
	outPort.open("/telecontroller/glove/vibration/out");
	if(oculusActivated) {
		Network::connect("/oculusController/oculus_raw/out",
			"/telecontroller/oculus/in", "udp", true);
	}
	if(kinectActivated) {
		Network::connect("/kinectController/kinect_raw/out",
			"/telecontroller/kinect/in", "udp", true);
	}
	if(gloveActivated) {
		Network::connect("/hapticGloveController/glove_raw/out",
			"/telecontroller/glove/in", "udp", true);
		Network::connect("/telecontroller/glove/vibration/out",
			"/hapticGloveController/vibration/in", "udp", true);
	}
}

void YarpAdapter::setSingleVelocityNormal(int bodyPart) {
	if (useDirect[bodyPart] || !activated[bodyPart])
		return;

	switch (bodyPart) {
	case MOVE_LEFT_ARM:
	case MOVE_RIGHT_ARM:
		positionControl[bodyPart]->setRefSpeed(0, 5);
		positionControl[bodyPart]->setRefSpeed(1, 5);
		positionControl[bodyPart]->setRefSpeed(2, 5);
		positionControl[bodyPart]->setRefSpeed(3, 5);
		positionControl[bodyPart]->setRefSpeed(10, 0);
		positionControl[bodyPart]->setRefSpeed(11, 0);
		positionControl[bodyPart]->setRefSpeed(12, 0);
		positionControl[bodyPart]->setRefSpeed(13, 0);
		positionControl[bodyPart]->setRefSpeed(14, 0);
		positionControl[bodyPart]->setRefSpeed(15, 0);
		return;
	case MOVE_TORSO:
		positionControl[bodyPart]->setRefSpeed(0, 10);
		positionControl[bodyPart]->setRefSpeed(1, 10);
		positionControl[bodyPart]->setRefSpeed(2, 10);
		return;
	case MOVE_HEAD:
		positionControl[bodyPart]->setRefSpeed(0, 5);
		positionControl[bodyPart]->setRefSpeed(1, 5);
		positionControl[bodyPart]->setRefSpeed(2, 5);
		return;
	}
}

void YarpAdapter::setSingleVelocityLow(int bodyPart) {
	if (useDirect[bodyPart] || !activated[bodyPart])
		return;

	switch (bodyPart) {
	case MOVE_LEFT_ARM:
	case MOVE_RIGHT_ARM:
		positionControl[bodyPart]->setRefSpeed(0, 3);
		positionControl[bodyPart]->setRefSpeed(1, 3);
		positionControl[bodyPart]->setRefSpeed(2, 3);
		positionControl[bodyPart]->setRefSpeed(3, 3);

		positionControl[bodyPart]->setRefSpeed(10, 00);
		positionControl[bodyPart]->setRefSpeed(11, 00);
		positionControl[bodyPart]->setRefSpeed(12, 00);
		positionControl[bodyPart]->setRefSpeed(13, 00);
		positionControl[bodyPart]->setRefSpeed(14, 00);
		positionControl[bodyPart]->setRefSpeed(15, 00);
		return;
	case MOVE_TORSO:
		positionControl[bodyPart]->setRefSpeed(0, 3);
		positionControl[bodyPart]->setRefSpeed(1, 3);
		positionControl[bodyPart]->setRefSpeed(2, 3);
		return;
	case MOVE_HEAD:
		positionControl[bodyPart]->setRefSpeed(0, 1);
		positionControl[bodyPart]->setRefSpeed(1, 1);
		positionControl[bodyPart]->setRefSpeed(2, 1);
		return;
	}
}

void YarpAdapter::initialize(bool kinectActivated,bool oculusActivated,bool gloveActivated) {
	//robotName = "/icubGazeboSim";
	robotName = "/icub";

	/*string ip_leftArm = "/10.0.0.2:10024/";
	 string ip_rightArm = "/10.0.0.2:10021/";
	 string ip_torso = "/10.0.0.2:10015/";
	 string ip_head = "/10.0.0.2/";*/

	//robotName = "/icub";
	/*string ip_leftArm = "/10.0.0.2:10026/";
	 string ip_rightArm = "/10.0.0.2:10029/";
	 string ip_torso = "/10.0.0.2:10023/";
	 string ip_head = "/10.0.0.2:10020/";*/

	string ip_leftArm = "/10.0.0.2/";
	string ip_rightArm = "/10.0.0.2/";
	string ip_torso = "/10.0.0.2/";
	string ip_head = "/10.0.0.2/";

	string remoteLeftArm = robotName + "/left_arm";
	string remoteRightArm = robotName + "/right_arm";
	string remoteTorso = robotName + "/torso";
	string remoteHead = robotName + "/head";

	string localPortsLeftArm = "/positionModule/left_arm";
	string localPortsRightArm = "/positionModule/right_arm";
	string localPortsTorso = "/positionModule/torso";
	string localPortsHead = "/positionModule/head";

	initializeDevice(MOVE_LEFT_ARM, remoteLeftArm, localPortsLeftArm,
			"left_arm_controller");
	initializeDevice(MOVE_RIGHT_ARM, remoteRightArm, localPortsRightArm,
			"right_arm_controller");
	initializeDevice(MOVE_TORSO, remoteTorso, localPortsTorso,
			"torso_controller");
	initializeDevice(MOVE_HEAD, remoteHead, localPortsHead, "head_controller");

	setVelocityNormal();
	initializeActivatedJoints();
	//initializeVelocity(MOVE_HEAD);

	initializePorts(kinectActivated, oculusActivated, gloveActivated);

	return;
}

void YarpAdapter::initializeActivatedJoints() {
	activatedJoints[MOVE_LEFT_ARM] = {
		true, true, true, true, true,
		false, false, false, false, false,
		false, false, false, false, false,
		false
	};

	activatedJoints[MOVE_RIGHT_ARM] = activatedJoints[MOVE_LEFT_ARM];
	activatedJoints[MOVE_TORSO] = {
			true, true, true
	};
	activatedJoints[MOVE_HEAD] = {
			true, true, true, true, true, true
	};
	activatedJoints[MOVE_LEFT_HAND] = {
			/*false, false, false, false, false,
			false, false, false, false, true,
			true, true, true, true, true,
			true*/
			false, false, false, false, false,
			false, false, false, false, true,
			true, true, true, true, true,
			false
	};
	activatedJoints[MOVE_RIGHT_HAND] = activatedJoints[MOVE_LEFT_HAND];
}

void YarpAdapter::setVelocityNormal() {
	setSingleVelocityNormal(MOVE_LEFT_ARM);
	setSingleVelocityNormal(MOVE_RIGHT_ARM);
	setSingleVelocityNormal(MOVE_TORSO);
}

void YarpAdapter::setVelocityLow() {
	setSingleVelocityLow(MOVE_LEFT_ARM);
	setSingleVelocityLow(MOVE_RIGHT_ARM);
	setSingleVelocityLow(MOVE_TORSO);
}

int YarpAdapter::move(int bodyPart, int jointId, double angle) {
	if (!activated[bodyPart])
		return -1;

	if (useDirect[bodyPart])
		positionDirect[bodyPart]->setPosition(jointId, angle);
	else
		positionControl[bodyPart]->positionMove(jointId, angle);

	return 0;
}

int YarpAdapter::move(int bodyPart, double *refs) {
	if (!activated[bodyPart])
		return -1;

	if (useDirect[bodyPart]) {
		int count = 0;
		int foundJoints[dof[bodyPart]];
		for(int i = 0; i < dof[bodyPart]; i++) {
			if(activatedJoints[bodyPart][i]) {
				foundJoints[count] = i;
				iCubBuffer[count] = refs[i];
				count++;
			}
		}

		if(bodyPart == MOVE_HEAD) {
			positionDirect[bodyPart]->setPositions(refs);
		}

		cout << "\n";
		if(bodyPart == YarpAdapter::MOVE_RIGHT_HAND) {
			cout << "Mapping to: " << iCubMapping[bodyPart] << "\n";
			for(int i = 0; i < dof[bodyPart]; i++) {
				cout << "i: " << i << " " << refs[i] << " | ";
			}
			cout << "\n";
		}

		positionDirect[iCubMapping[bodyPart]]->setPositions(count, foundJoints, iCubBuffer);
	} else
		positionControl[bodyPart]->positionMove(refs);

	return 0;
}

double* YarpAdapter::getJoints(int bodyPart) {
	double* values = new double[dof[bodyPart]];
	encoders[bodyPart]->getEncoders(values);
	return values;
}

int YarpAdapter::getDof(int bodyPart) {
	return dof[bodyPart];
}

bool YarpAdapter::isDirect(int bodyPart) {
	return useDirect[bodyPart];
}

bool YarpAdapter::isActivated(int bodyPart) {
	return activated[bodyPart];
}

int YarpAdapter::close() {
	robotDevice[0].close();
	robotDevice[1].close();
	robotDevice[2].close();
	robotDevice[3].close();

	return 0;
}

double* YarpAdapter::readOculusInput() {
	Bottle* in = inPort[OCULUS_IN].read(false);
	if (in == NULL) {
		return oldOculusData;
	}

	double* inVals = new double[in->size()];
	for (int i = 0; i < 6; i++)
		inVals[i] = in->get(i).asDouble();

	oldOculusData = inVals;
	oculusInitialized = true;
	return inVals;
}

/**
 *
 */
vector<Vector> YarpAdapter::readKinectInput() {
	Bottle* in = inPort[KINECT_IN].read(false);
	if (in == NULL) {
		return oldKinectData;
	}

	int isize = in->size();

	// read body count
	bodyCount = in->get(0).asInt();

	// collect joints
	vector<Vector> outVal(isize - 3); //= new vector<Vector>[isize-3];
	int k = 0;
	for (int i = 1; i < isize - 2; i = i + 3) {
		outVal[k] = Vector(in->get(i).asDouble(), in->get(i + 1).asDouble(),
				in->get(i + 2).asDouble());
		k++;
	}

	// get hand state
	leftHandClosed = in->get(isize - 2).asInt() == 1 ? true : false;
	rightHandClosed = in->get(isize - 1).asInt() == 1 ? true : false;

	oldKinectData = outVal;

	kinectInitialized = true;
	return outVal;
}

double* YarpAdapter::readGloveInput() {
	Bottle* in = inPort[GLOVE_IN].read(false);
	if (in == NULL) {
		return oldGloveData;
	}

	for (int i = 0; i < 5; i++)
		oldGloveData[i] = in->get(i).asDouble();

	gloveInitialized = true;
	return oldGloveData;
}

int YarpAdapter::getCurrentBodyCount() {
	return bodyCount;
}

bool YarpAdapter::isLeftHandClosed() {
	return leftHandClosed;
}

bool YarpAdapter::isRightHandClosed() {
	return rightHandClosed;
}

bool YarpAdapter::firstValuesReceivedKinect() {
	return kinectInitialized;
}

bool YarpAdapter::firstValuesReceivedOculus() {
	return oculusInitialized;
}

bool YarpAdapter::firstValuesReceivedGlove() {
	return gloveInitialized;
}

void YarpAdapter::writeVibrationDataToOutput(int* vibData) {
	Bottle& out = outPort.prepare();
	out.clear();

	// add vibration data to output
	for (int i = 0; i < GloveAdapter::NUM_OF_FINGERS; i++) {
		out.addInt(vibData[i]);
	}

	outPort.write();
}

double* YarpAdapter::getFingerPressure(int bodyPart) {
	return new double[GloveAdapter::NUM_OF_FINGERS];
}
