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

#include <iostream>
#include <TelecontrolMain.h>
#include <OculusAdapter.h>
#include <KinectAdapter.h>
#include <GloveAdapter.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <YarpAdapter.h>

#include <fstream>

using namespace std;
clock_t lastClock;
clock_t currentClock;
double lastTime;
double currentTime;

int fps = 200;
double spf = double(1) / double(fps) * 1000000;
int i = 0;

int main(int ac, char** av) {
	bool kinectActivated = true;
	bool oculusActivated = true;
	bool gloveActivated = false;
	
	
	bool running = true;
	bool initialized = false;
	bool performMovement = false;

	KinectAdapter* kinectAdapter = KinectAdapter::getCurrent();
	YarpAdapter* yAdapter = new YarpAdapter(kinectActivated, oculusActivated, gloveActivated);
	kinectAdapter->setController(yAdapter);
	OculusAdapter* oAdapter = new OculusAdapter(yAdapter);
	GloveAdapter* gAdapter = new GloveAdapter();
	TelecontrolMain* tmain = new TelecontrolMain();

	int i = 0;
	bool allowedToMove = true;
	while (running == true) {
		if (allowedToMove) {
			i++;
			lastClock = clock();
			lastTime = ((double) (lastClock));

			// kinect stuff
			if (kinectActivated) {
				tmain->moveRobot(yAdapter);
			}

			// oculus stuff
			if (oculusActivated) {
				double* newVals = yAdapter->readOculusInput();
				if (yAdapter->firstValuesReceivedOculus()) {
					double* angles = oAdapter->transformToAnglesInDegree(
							newVals);
					yAdapter->move(YarpAdapter::MOVE_HEAD, angles);
				}
			}

			// glove stuff
			if (gloveActivated) {
				double* newVals = yAdapter->readGloveInput();
				if (yAdapter->firstValuesReceivedGlove()) {
					double* handValues = gAdapter->calculateFingerAngles(
							newVals);
					yAdapter->move(YarpAdapter::MOVE_RIGHT_HAND, handValues);
					//double* pressureVals = yAdapter->getFingerPressure(YarpAdapter::MOVE_LEFT_HAND);
					//int* vibValues = gAdapter->calculateVibrationIntensities(
					//		pressureVals);
					//yAdapter->writeVibrationDataToOutput(vibValues);
				}
			}
			allowedToMove = false;
		}

		currentClock = clock();
		currentTime = ((double) (currentClock));
		if ((currentTime - lastTime) > spf) {
			allowedToMove = true;
		}
	}

	return 0;
}

TelecontrolMain::TelecontrolMain() {

}

void TelecontrolMain::moveRobot(YarpAdapter* yAdapter) {

	KinectAdapter* kinectAdapter = KinectAdapter::getCurrent();

	vector<Vector> allJointPos = yAdapter->readKinectInput();
	if (!yAdapter->firstValuesReceivedKinect())
		return;

	int bodyCount = yAdapter->getCurrentBodyCount();

	std::vector<double*> angles = kinectAdapter->Update(allJointPos);
	//updateHandPosition(rightHandState == 1, true);
	//updateHandPosition(leftHandState == 1, false);

	if (bodyCount == 1) {
		int i = 0;

		double* leftArmValues = new double[16];
		double* rightArmValues = new double[16];
		double* torsoValues = new double[3];

		for (i = 0; i < 16; i++) {
			if (i > 3) {
				leftArmValues[i] = 0;
			} else
				leftArmValues[i] = angles[0][i];

		}
		for (i = 0; i < 16; i++) {
			if (i > 3) {
				rightArmValues[i] = 0;
			} else
				rightArmValues[i] = angles[1][i];
		}
		for (i = 0; i < 3; i++) {
			torsoValues[i] = angles[2][i];
		}

		double* leftArmEncoder = yAdapter->getJoints(
				YarpAdapter::MOVE_LEFT_ARM);
		double* rightArmEncoder = yAdapter->getJoints(
				YarpAdapter::MOVE_RIGHT_ARM);
		double* torsoEncoder = yAdapter->getJoints(YarpAdapter::MOVE_TORSO);

		yAdapter->move(YarpAdapter::MOVE_LEFT_ARM, leftArmValues);//setHandPosition(false, angles[0]));//setHandPosition(leftHandClosed,angles[0]));
		yAdapter->move(YarpAdapter::MOVE_RIGHT_ARM, rightArmValues); //setHandPosition(false, angles[1]));
		yAdapter->move(YarpAdapter::MOVE_TORSO, torsoValues);

		TelecontrolMain::log(leftArmValues, rightArmValues, torsoValues,
				leftArmEncoder, rightArmEncoder, torsoEncoder);
	}
}

void TelecontrolMain::log(double* leftArmValues, double* rightArmValues,
		double* torsoValues, double* leftArmEncoder, double* rightArmEncoder,
		double* torsoEncoder) {
	ofstream myfile;
	myfile.open("controller.txt", ios::app);
	int z = 0;
	for (z = 0; z < 7; z++) {
		myfile << leftArmValues[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 7; z++) {
		myfile << rightArmValues[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 3; z++) {
		myfile << torsoValues[z] << "  ";
	}
	myfile << "\n";
	myfile.close();

	myfile.open("encoder.txt", ios::app);
	for (z = 0; z < 7; z++) {
		myfile << leftArmEncoder[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 7; z++) {
		myfile << rightArmEncoder[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 3; z++) {
		myfile << torsoEncoder[z] << "  ";
	}
	myfile << "\n";
	myfile.close();

	/*myfile.open("hands.txt", ios::app);
	 myfile << "Left Hand:  " << leftHandState << "  Left Hand Filtered:  " << leftHandClosed << "    Right Hand:  " << rightHandState << "    Right Hand Filtered:  " << rightHandClosed << "\n";
	 myfile.close();*/
}

double* setHandPosition(bool closed, double* oldValues) {
	double armValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	armValues[0] = -30;
	armValues[1] = 30;
	armValues[2] = 0;
	armValues[3] = 45;
	armValues[4] = 0;
	armValues[5] = 0;
	armValues[6] = 0;
	armValues[7] = 15;
	armValues[8] = 45;
	armValues[9] = 0;
	if (closed) {
		armValues[10] = 46;
		armValues[11] = 46;
		armValues[12] = 64;
		armValues[13] = 44;
		armValues[14] = 65;
		armValues[15] = 110;
	} else {
		armValues[10] = 5;
		armValues[11] = 5;
		armValues[12] = 5;
		armValues[13] = 5;
		armValues[14] = 5;
		armValues[15] = 5;
	}
	return armValues;
}