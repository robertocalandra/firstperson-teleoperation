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

#include <GloveAdapter.h>
#include <MultidimensionalFilter.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>

using namespace std;

bool initialized = false;
bool verbose = false;
double minVal[] = { 500, 500, 500, 500, 500 };
double maxVal[] = { 700, 700, 700, 700, 700 };
double transformedVals[GloveAdapter::NUM_OF_FINGERS];
double allJoints = 16;
double outputVals[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
MultidimensionalFilter filter3;

double lowerLimits[] = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 5,
		5, 5, 5, 5, 5, 5 };
double upperLimits[] = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 46,
		46, 64, 44, 65, 65, 5 };
double actualValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

GloveAdapter::GloveAdapter() {

	ofstream myfile;
	myfile.open("gloves_rawData.txt");
	myfile << "";
	myfile.close();

	myfile.open("gloves_filteredData.txt");
	myfile << "";
	myfile.close();

	myfile.open("gloves_linearMapped.txt");
	myfile << "";
	myfile.close();

	myfile.open("gloves_control.txt");
	myfile << "";
	myfile.close();
}

GloveAdapter::~GloveAdapter() {

}

double* GloveAdapter::calculateFingerAngles(double* gloveValues) {
	for (int i = 0; i < NUM_OF_FINGERS; i++) {
		if (gloveValues[i] <= 400 || gloveValues[i] >= 750) {
			continue;
		}

		transformedVals[i] = gloveValues[i];
		//	cout << " i: " << i << " " << transformedVals[i] << " | ";
	}
	//cout << "\n";

	if (!initialized) {
		initialized = true;
		for (int i = 0; i < NUM_OF_FINGERS; i++) {
			maxVal[i] = transformedVals[i];
			minVal[i] = maxVal[i] - 400;
		}
		filter3.initialize(maxVal, true, 5, Filter::KINECT_FILTER);
	}

	logAngles("gloves_rawData.txt", transformedVals, 5);
	filter3.filterData(transformedVals);
	logAngles("gloves_filteredData.txt", transformedVals, 5);

	for (int i = 0; i < NUM_OF_FINGERS; i++) {
		transformedVals[i] = -(transformedVals[i] - minVal[i])
				/ (maxVal[i] - minVal[i]);
	}
	logAngles("gloves_linearMapped.txt", transformedVals, 5);

	if (verbose) {
		for (int i = 0; i < NUM_OF_FINGERS; i++) {
			cout << " i: " << i << " " << transformedVals[i] << " | ";
		}
		cout << "\n";
	}

	// mapping to iCub
	for (int i = 0; i < NUM_OF_FINGERS; i++) {
		//actualValues[i] = gloveValues[i];
		switch (i) {
		case 0:
			outputVals[9] = transformedVals[4];
			outputVals[10] = transformedVals[4];
			break;
		case 1:
			outputVals[11] = transformedVals[3];
			outputVals[12] = transformedVals[3];
			break;
		case 2:
			outputVals[13] = transformedVals[2];
			outputVals[14] = transformedVals[2];
			break;
		case 3:
			outputVals[15] = transformedVals[1];
			break;
		case 4:
			// NOT NEEDED FOR ICUB
			break;
		}
	}

	for (int i = 0; i < allJoints; i++) {
		outputVals[i] = outputVals[i]
				* (upperLimits[i] - lowerLimits[i]) + upperLimits[i];

		if (outputVals[i] >= upperLimits[i]) {
			outputVals[i] = upperLimits[i];
			continue;
		}
		if (outputVals[i] <= lowerLimits[i]) {
			outputVals[i] = lowerLimits[i];
			continue;
		}
	}

	logAngles("gloves_control.txt", outputVals, allJoints);
	return outputVals;
}

int* GloveAdapter::calculateVibrationIntensities(double* pressureValues) {
	return new int[NUM_OF_FINGERS];
}
void GloveAdapter::logAngles(char* filename, double* values, int size) {
	ofstream myfile;
	clock_t actualClock = clock();
	actualTime = 1000 * ((double) (actualClock)) / CLOCKS_PER_SEC;
	myfile.open(filename, ios::app);
	myfile << actualTime << "\t";
	for (int i = 0; i < size; i++) {
		myfile << values[i] << "\t";
	}
	myfile << "\n";
	myfile.close();
}
