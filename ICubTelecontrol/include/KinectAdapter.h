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

#ifndef KINECTADAPTER
#define KINECTADAPTER

#include "resource.h"
#include <vector>
#include "Vector.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <YarpAdapter.h>

class KinectAdapter
{
	static const int NECK_PITCH = 0;
	static const int NECK_ROLL = 1;
	static const int NECK_YAW = 2;
	static const int LEFT_SHOULDER_PITCH = 3;
	static const int LEFT_SHOULDER_ROLL = 4;
	static const int LEFT_SHOULDER_YAW = 5;
	static const int LEFT_ELBOW = 6;
	static const int LEFT_WRIST_PROSUP = 7;
	static const int LEFT_WRIST_PITCH = 8;
	static const int LEFT_WRIST_YAW = 9;
	static const int RIGHT_SHOULDER_PITCH = 10;
	static const int RIGHT_SHOULDER_ROLL = 11;
	static const int RIGHT_SHOULDER_YAW = 12;
	static const int RIGHT_ELBOW = 13;
	static const int RIGHT_WRIST_PITCH = 14;
	static const int RIGHT_WRIST_YAW = 15;
	static const int RIGHT_WRIST_PROSUP = 16;
	static const int TORSO_YAW = 17;
	static const int TORSO_ROLL = 18;
	static const int TORSO_PITCH = 19;
	


public:
	/// <summary>
	/// Constructor
	/// </summary>
	KinectAdapter();

	/// <summary>
	/// Destructor
	/// </summary>
	~KinectAdapter();
	static KinectAdapter* getCurrent();
	static void setCurrent(KinectAdapter* tm);
	static bool hasCurrent();
	std::vector<double*> Update(std::vector<Vector> newInputValues);
	std::vector<double*> iCubSorting();
	void setController(YarpAdapter* c);
	double* setHandPosition(bool closed, double* oldValues);
	void limitStepSize(int bodyPart, double* commandValues, double* encoderValues);

private:
	void logging();
	void log(char* filename);
	void logKinect(std::vector<Vector> newPos);
	void calculateAngles();
	//void averageAngles();
	double getAngle3D(Vector vec1, Vector vec2);
};

#endif
