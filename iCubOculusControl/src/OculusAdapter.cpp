#include <tchar.h>
#include <iostream>
#include <resource.h>
#include <OculusAdapter.h>
#include <YarpAdapter.h>
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include "stdafx.h"
#include <fstream>
#include <math.h>
#include "Plain.h"
#include "time.h"


using namespace std;

double headAngles[] = { 0, 0, 0, 0, 0, 0};
int limitations[] = { 20, 15, 35, 30, 30, 0 };
int corresponding[] = { 3, -1, 4 };
clock_t startClock;
YarpAdapter* yAdapter;
double stepsize = 0.5;
int numberOfAngles;
vector<vector<double>> angles;

OculusAdapter::OculusAdapter(YarpAdapter *yarpAdapter) {
	clock_t startingtime = clock();
	yAdapter = yarpAdapter;
	numberOfAngles = 5;
	angles.resize(numberOfAngles, vector<double>(6, 0));
}

std::vector<double*> OculusAdapter::transformToAnglesInDegree(double* newPos) {
	headAngles[0] = newPos[0] * 180; // pitch
	headAngles[1] = newPos[1] * 180; // roll
	headAngles[2] = newPos[2] * 180; // yaw

	//this->averageAngles();
	double* encoderValues = yAdapter->getJoints(YarpAdapter::MOVE_HEAD);
	//double encoderValues[6] = { 0, 0, 0, 0, 0, 0 };

	this->limitAngles(encoderValues);
	this->logAngles(encoderValues, newPos);

	std::vector<double*> returnValues = { headAngles };
	return returnValues;
}

// limit maximum values that can be set for the icubs neck and use the overflow for controling the eyes in the corresponding direction
void OculusAdapter::limitAnglesAndControlEyes(int i) {
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
}

void OculusAdapter::limitAngles(double* encoderValues) {
	for (int i = 0; i < 6; i++) {
		double dif = headAngles[i] - encoderValues[i];
		if (i == 0 || i == 2) {
			if (headAngles[i] > limitations[i] || headAngles[i] < -limitations[i]) {
				this->limitAnglesAndControlEyes(i);
			}
		}
		if (dif > stepsize)
			headAngles[i] = encoderValues[i] + stepsize;
		else
		if (dif < -stepsize)
			headAngles[i] = encoderValues[i] - stepsize;
	}
}

// logging raw data from HMD, resulting command signal and induced movement of the robot in that order
void OculusAdapter::logAngles(double* encoderValues, double* newValues) {
	clock_t actualClock = clock();
	float actualTime = 1000 * ((double)(actualClock)) / CLOCKS_PER_SEC;	

	ofstream myfile;
	myfile.open("head_oculus.txt", ios::app);
	myfile << actualTime << " " << newValues[0] * 180 << " " << newValues[1] * 180 << " " << newValues[2] * 180 << "\n";
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

void OculusAdapter::averageAngles(){
	vector<double> newAnglesVec(headAngles, headAngles + 6);

	int i = 0;
	int j = 0;
	for (i = 0; i < numberOfAngles - 1; i++) {
		angles[i] = angles[i + 1];
	}
	angles[numberOfAngles - 1] = newAnglesVec;
	//double avgAngles[6] = { 0, 0, 0, 0, 0, 0 };
	for (j = 0; j < 6; j++){
		headAngles[j] = 0;
		for (i = 0; i < numberOfAngles; i++){
			headAngles[j] += angles[i][j];
		}
		headAngles[j] /= numberOfAngles;
	}

	int k = 0;

}