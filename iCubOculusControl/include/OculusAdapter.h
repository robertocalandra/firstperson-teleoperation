#ifndef OAdapter

#define OAdapter

#include "resource.h"
#include <vector>
#include "Vector.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <YarpAdapter.h>

class OculusAdapter
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
	static const int RIGHT_SHOULDER_YAW = 11;
	static const int RIGHT_SHOULDER_ROLL = 12;
	static const int RIGHT_WRIST_PROSUP = 13;
	static const int RIGHT_WRIST_PITCH = 14;
	static const int RIGHT_WRIST_YAW = 15;
	static const int RIGHT_ELBOW = 16;
	static const int TORSO_YAW = 17;
	static const int TORSO_ROLL = 18;
	static const int TORSO_PITCH = 19;



public:
	/// <summary>
	/// Constructor
	/// </summary>
	OculusAdapter(YarpAdapter* yarpAdapter);

	/// <summary>
	/// Destructor
	/// </summary>
	~OculusAdapter();
	std::vector<double*> transformToAnglesInDegree(double* newPos);
	void limitAnglesAndControlEyes(int i);
	void limitAngles(double* encoderValues);
	void logAngles(double* encoderValues, double* newValues);
	void averageAngles();
private:

};

#endif // !OAdapter
