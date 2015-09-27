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
#include <resource.h>
#include <OculusAdapter.h>
//#include <stdlib.h>
//#include <stdio.h>
#include <fstream>
//#include <math.h>
//#include <YarpAdapter.h>

#include <Plain.h>
#include <time.h>
#include <MultidimensionalFilter.h>

using namespace std;

double headAngles[] = { 0, 0, 0, 0, 0, 0};
int limitations[] = { 20, 15, 35, 30, 30, 0 };
//int overControlLimitations[] = {2, 2, 2, 2, 2, 0};
int corresponding[] = { 3, -1, 4 };
MultidimensionalFilter filter2;

OculusAdapter::OculusAdapter(YarpAdapter *yarpAdapter) {
	//clock_t startingtime = clock();
	yAdapter = yarpAdapter;
	numberOfAngles = 5;
	double* encoderValues = yAdapter->getJoints(YarpAdapter::MOVE_HEAD);
	introduceOverControlLimits();
	filter2.initialize(encoderValues,true,3,Filter::KINECT_FILTER);;
	resetLogFiles();
}

void OculusAdapter::introduceOverControlLimits() {
	for(int i = 0; i < numberOfAngles; i++) {
		//limitations[i] = limitations[i] - overControlLimitations[i];
	}
}

void OculusAdapter::resetLogFiles(){
	ofstream myfile;
		myfile.open("head_oculus.txt");
		myfile << "";
		myfile.close();
		myfile.open("head_command.txt");
		myfile << "";
		myfile.close();
		myfile.open("head_robot.txt");
		myfile << "";
		myfile.close();
		myfile.open("head_cropped.txt");
		myfile << "";
		myfile.close();
}

double* OculusAdapter::transformToAnglesInDegree(double* newPos) {
	headAngles[0] = newPos[0] * 180; // pitch
	headAngles[1] = newPos[1] * 180; // roll
	headAngles[2] = newPos[2] * 180; // yaw

	//this->averageAngles();
	vector<double> newAnglesVec(headAngles, headAngles + 6);
	//filter2.filterData(newAnglesVec);
	double* encoderValues = yAdapter->getJoints(YarpAdapter::MOVE_HEAD);
	//double encoderValues[6] = { 0, 0, 0, 0, 0, 0 };

	filter2.filterData(headAngles);
	this->limitAngles(encoderValues);
	double croppedData[] = {headAngles[0],headAngles[1],headAngles[2],headAngles[3],headAngles[4]};

	this->logAngles(encoderValues, newPos, croppedData);

	//std::vector<double*> returnValues = { headAngles };
	return headAngles;
}

void OculusAdapter::limitAnglesAndControlEyes(int i) {

	//for (int i = 0; i < 3; i++) {
			int correspondingVal = corresponding[i];
			if (headAngles[i] < -limitations[i]) {
				if (correspondingVal != -1) {
					headAngles[correspondingVal] = -headAngles[i] - limitations[i];
					if (headAngles[correspondingVal] > limitations[correspondingVal]) {
						headAngles[correspondingVal] = limitations[correspondingVal];
					}
				}
				headAngles[i] = -limitations[i];
			}
			else {
				if (headAngles[i] > limitations[i]) {
					if (correspondingVal != -1) {
						headAngles[correspondingVal] = -headAngles[i] + limitations[i];
						if (headAngles[correspondingVal] < -limitations[correspondingVal]) {
							headAngles[correspondingVal] = -limitations[correspondingVal];
						}
					}
					headAngles[i] = limitations[i];
				}
				else {
					if (correspondingVal != -1) {
						headAngles[correspondingVal] = 0;
					}
				}
			}	

			if (i == 0) {
				headAngles[correspondingVal] = -headAngles[correspondingVal];
			}
	//}
}

void OculusAdapter::limitAngles(double* encoderValues) {
	double stepSize = 5;
	for (int i = 0; i < 6; i++) {
		//double dif = headAngles[i] - encoderValues[i];
		//if (i == 0 || i == 2) {
			if (headAngles[i] > limitations[i] || headAngles[i] < -limitations[i]) {
				this->limitAnglesAndControlEyes(i);
			}
    /*
		if (dif > stepSize)
			headAngles[i] = encoderValues[i] + stepSize;
		else
		if (dif < -stepSize)
			headAngles[i] = encoderValues[i] - stepSize;
	*/
	}
}

void OculusAdapter::logAngles(double* encoderValues, double* newValues, double* croppedData) {
	clock_t actualClock = clock();
	float actualTime = 1000 * ((double)(actualClock)) / CLOCKS_PER_SEC;	

	ofstream myfile;
	myfile.open("head_oculus.txt", ios::app);
	myfile << actualTime << " " << newValues[0] * 180 << " " << newValues[1] * 180 << " " << newValues[2] * 180 << "\n";
	myfile.close();

	myfile.open("head_cropped.txt", ios::app);
	myfile << actualTime << " " << croppedData[0] << " " << croppedData[1] << " " << croppedData[2]
		<< " " << croppedData[3] << " " << croppedData[4] << "\n";
	myfile.close();

	// log
	myfile.open("head_command.txt", ios::app);
	myfile << actualTime << " " << headAngles[0] << " " << headAngles[1] << " " << headAngles[2]
		<< " " << headAngles[3] << " " << headAngles[4] << "\n";
	myfile.close();

	myfile.open("head_robot.txt", ios::app);
	myfile << actualTime << " " << encoderValues[0] << " " << encoderValues[1]
		<< " " << encoderValues[2] << " " << encoderValues[3] << " " << encoderValues[4] << "\n";
	myfile.close();
}

