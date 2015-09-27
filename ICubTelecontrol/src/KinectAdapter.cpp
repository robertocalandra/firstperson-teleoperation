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

/// 																			///
/// Class for transforming data from the raw kinect data to data for the robot 	///
///																				///

#define _USE_MATH_DEFINES
#include <KinectAdapter.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <KMatrix.h>
#include <Plain.h>
#include <Kinect.h>
#include <MultidimensionalFilter.h>

using namespace std;

YarpAdapter* yAdapter = NULL;

double leftArmValues[] = { 0, 0, 0, 0, 0, 0, 0 };
double rightArmValues[] = { 0, 0, 0, 0, 0, 0, 0 };
double neckValues[] = { 0, 0, 0 };
double torsoValues[] = { 0, 0, 0 };

double stepSize = 3;


std::vector<std::vector<double>> limiterAngles;
std::vector<double> lastAngles;
std::vector<double> minimumAngles;
std::vector<double> maximumAngles;
std::vector<double> newAngles;
std::vector<Vector> lastPositions;
std::vector<Vector> newPositions;
double numberOfAngles;
clock_t startClock;
clock_t actualClock;
double actualTime;
bool firstTime = true;

static KinectAdapter *current;
MultidimensionalFilter filter;


/// <summary>
/// Constructor
/// </summary>
KinectAdapter::KinectAdapter(){
	limiterAngles.resize(3, vector<double>(32, 0));
	lastAngles = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	newAngles = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	minimumAngles = { 0, 0, 0, -70, -15, 15, 20, 0, 0, 0, -70, -10, 15, 20, 0, 0, 0, -30, -10, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	maximumAngles = { 0, 0, 0, 0, 15, 90, 90, 0, 0, 0, 0, 15, 90, 90, 0, 0, 0, 30, 10, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	startClock = clock();
	resetFiles();
}


/// <summary>
/// Destructor
/// </summary>
KinectAdapter::~KinectAdapter()
{

}
/// <summary>
/// The class KinectAdapter is a singleton. The function getCurrent
/// returns the only instance. If necessary one is created.
/// </summary>
KinectAdapter* KinectAdapter::getCurrent()
{
	if (current == 0)
	{
		current = new KinectAdapter();
	}
	return current;
}


/// <summary>
/// Set the actual instance of KinectAdapter
/// </summary>
void KinectAdapter::setCurrent(KinectAdapter* tm)
{
	current = tm;
}
/// <summary>
/// Resets all the writeout files
/// </summary>
void KinectAdapter::resetFiles()
{
	ofstream myfile;
	myfile.open("humanValues.txt");
	myfile << "";
	myfile.close();
	myfile.open("mistakes.txt");
	myfile << "";
	myfile.close();
	myfile.open("angles.txt");
	myfile << "";
	myfile.close();
	myfile.open("kinectValues.txt");
	myfile << "";
	myfile.close();
	myfile.open("anglesWithNames.txt");
	myfile << "";
	myfile.close();
	myfile.open("newAngles.txt");
	myfile << "";
	myfile.close();
	myfile.open("controller.txt");
	myfile << "";
	myfile.close();
	myfile.open("encoder.txt");
	myfile << "";
	myfile.close();
	myfile.open("raw_data_in_Joint_Space.txt");
	myfile << "";
	myfile.close();
	myfile.open("cropped_raw_data.txt");
	myfile << "";
	myfile.close();
	myfile.open("filtered_data.txt");
	myfile << "";
	myfile.close();
}
/// <summary>
/// Sets the yarpAdapter. Furthermore it initializes the Filters.
/// <param name="yarpAdapter">The YarpAdapter </param>
/// </summary>
void KinectAdapter::setController(YarpAdapter* yarpAdapter)
{

	yAdapter = yarpAdapter;
	int i = 0;
	double* leftArmValuesInitial = yAdapter->getJoints(YarpAdapter::MOVE_LEFT_ARM);
	double* rightArmValuesInitial = yAdapter->getJoints(YarpAdapter::MOVE_RIGHT_ARM);
	double*  torsoValuesInitial = yAdapter->getJoints(YarpAdapter::MOVE_TORSO);

		newAngles[LEFT_SHOULDER_PITCH] =leftArmValuesInitial[0];
		newAngles[LEFT_SHOULDER_YAW] =leftArmValuesInitial[1];
		newAngles[LEFT_SHOULDER_ROLL] =leftArmValuesInitial[2];
		newAngles[LEFT_ELBOW] =leftArmValuesInitial[3];

		newAngles[RIGHT_SHOULDER_PITCH] = rightArmValuesInitial[0];
		newAngles[RIGHT_SHOULDER_YAW] =rightArmValuesInitial[1];
		newAngles[RIGHT_SHOULDER_ROLL] = rightArmValuesInitial[2];
		newAngles[RIGHT_ELBOW] =rightArmValuesInitial[3];

		newAngles[TORSO_YAW] = torsoValuesInitial[0];
		newAngles[TORSO_ROLL] = torsoValuesInitial[1];
		newAngles[TORSO_PITCH]=torsoValuesInitial[2];

		filter.initialize(newAngles,true,32,Filter::KINECT_FILTER);
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 32; j++){
				limiterAngles[i][j] = newAngles[j];
			}
		}
}


/// <summary>
/// Returns if a class of the singleton exists
/// </summary>
bool KinectAdapter::hasCurrent()
{
	return (current != 0);
}

/// <summary>
/// Main processing function
/// <param name="newPos">The new joint positions in 3-d coordinates</param>
/// </summary>
std::vector<double*> KinectAdapter::Update(std::vector<Vector> newPos)
{
	//update the time and positions
	actualClock = clock();
	actualTime = 1000*((double)(actualClock))/CLOCKS_PER_SEC;
	ofstream myfile;
	newPositions = newPos;
	if (firstTime){
		lastPositions = newPositions;
		firstTime = false;
	}
	//logs the new coordinates
	logKinect(newPos);

	//Call to main calculating function
	//The angles for the robot are calculated here
	calculateAngles();


	//checks for Nan and Inf, takes old value if there is one
	//and writes event into a file 
	for (int i = 0; i < 32; i++){
			if (newAngles[i] != newAngles[i]){
				myfile.open("mistakes.txt", ios::app);
				myfile << "My mistake at new Angle " << i << " position \n";
				myfile.close();
				newAngles[i] = lastAngles[i];
			}
	}
	
	//logging and filtering
	log("raw_data_in_Joint_Space.txt");
	newAngles = filter.filterData(newAngles);
	log("filtered_data.txt");
	
	//pruning if values exceed robot limits and logging
	for (int i = 0; i < 32; i++){
		if (newAngles[i] > maximumAngles[i]/*-adjustingAreaSizes[i]*/){
			newAngles[i] = maximumAngles[i];
		}
		if (newAngles[i] < minimumAngles[i]/*+adjustingAreaSizes[i]*/){
			newAngles[i] = minimumAngles[i];
		}
	
	log("cropped_raw_data.txt");



	lastAngles = newAngles;
	logging();
	lastPositions = newPositions;
	//bringing values in the correct form for the robot
	std::vector<double*> returnValues iCubSorting();

	return returnValues;
}


/// <summary>
/// All angles are calculated from 3-d coordinates
/// </summary>
void KinectAdapter::calculateAngles(){


	newAngles = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	//Orientation UpperSide
	ofstream myfile;
	////////////////////////////////////////////////////////////////////////////////////////
	//								Coordinate Systems									  //
	////////////////////////////////////////////////////////////////////////////////////////
	//Coordinate System and help axes are created
	
	//Coordinate System within and for the human(right handed)
	Vector zAxisHelper = Vector(lastPositions[JointType_ShoulderRight], lastPositions[JointType_SpineBase]);
	Vector xAxis = Vector(lastPositions[JointType_ShoulderRight], lastPositions[JointType_ShoulderLeft]);//right to left
	Vector zAxis = zAxisHelper.getNormalVector(xAxis);//out of the human(like an arrow in the back)
	Vector yAxis = zAxis.getNormalVector(xAxis);//like spine, but straight
	//Coordinate System in the room
	Vector gravity = Vector(0, 1, 0);
	Vector groundX = Vector(-1, 0, 0);
	Vector groundZ = Vector(0, 0, -1);
	xAxis.normalize();
	yAxis.normalize();
	zAxis.normalize();
	//Transform everything to coordinates in the human coordinate system
	KMatrix transformationMatrix = KMatrix(xAxis, yAxis, zAxis);
	int i = 0;
	int iteratorEnd = lastPositions.size();
	myfile.open("humanValues.txt", ios::app);
	myfile << "\n";
	for (i = 0; i < iteratorEnd; i++){
		lastPositions[i] = transformationMatrix.multiply(lastPositions[i]);
		myfile << lastPositions[i].x << " " << lastPositions[i].y << " " << lastPositions[i].z << "\t";
	}
	myfile << "\n";
	myfile.close();
	gravity = transformationMatrix.multiply(gravity);
	groundX = transformationMatrix.multiply(groundX);
	groundZ = transformationMatrix.multiply(groundZ);
	xAxis = Vector(1, 0, 0);
	yAxis = Vector(0, 1, 0);
	zAxis = Vector(0, 0, 1);

	//create help planes
	Plain  frontView = Plain(xAxis, yAxis);//Plain for front view: normal is zAxis
	Plain sideView = Plain(yAxis, zAxis);//Plain for side view: normal is xAxis
	Plain topView = Plain(zAxis, xAxis);//Plain for top view: normal is yAxis

	Plain ground = Plain(groundZ, groundX);

	////////////////////////////////////////////////////////////////////////////////////////
	//										Head										  //
	////////////////////////////////////////////////////////////////////////////////////////

	Vector neck = Vector(lastPositions[JointType_Neck], lastPositions[JointType_Head]);
	//Pitch: front back movement
	double neckPitch = 90 - getAngle3D(zAxis, neck);
	if (frontView.isOnFront(neck)){
		neckPitch = -neckPitch;
	}
	//Roll: left right movement
	double neckRoll = 90 - getAngle3D(xAxis, neck);
	if (!sideView.isOnFront(neck)){
		neckRoll = -neckRoll;
	}
	//Yaw: turning

	//newAngles[NECK_ROLL] = neckRoll;
	//newAngles[NECK_PITCH] = neckPitch;


	////////////////////////////////////////////////////////////////////////////////////////
	//										Left Arm									  //
	////////////////////////////////////////////////////////////////////////////////////////
	
	//Recreating arm with upper and under arm
	Vector leftUpperArm = Vector(lastPositions[JointType_ShoulderLeft], lastPositions[JointType_ElbowLeft]);
	Vector leftUnderArm = Vector(lastPositions[JointType_ElbowLeft], lastPositions[JointType_WristLeft]);

	double leftYaw = 0;
	double leftPitch = 0;
	double leftRoll = 0;
	double leftElbow = getAngle3D(leftUpperArm, leftUnderArm);

	double armlengthLeft = leftUpperArm.getLength();
	leftYaw = asin(leftUpperArm.x/armlengthLeft);//Comes from robot structure
	leftPitch = atan(leftUpperArm.z / leftUpperArm.y);//Comes from robot structure
	leftYaw = leftYaw * 180 / M_PI;
	leftPitch = leftPitch * 180 / M_PI;
	
	//Because of the form of the sinus it has to be checked if the angle is bigger than 90°
	if (topView.isOnFront(leftUpperArm)){
		leftYaw = 180 - leftYaw;
	}

	//Recreating under Arm Position with known Angles(without roll)
	KMatrix leftRotationAroundZ = KMatrix::RotationZ(leftYaw*M_PI / 180);
	KMatrix leftRotationAroundX = KMatrix::RotationX(leftPitch*M_PI / 180);
	KMatrix leftElbowRotation = KMatrix::RotationX(-leftElbow*M_PI / 180);
	Vector leftUnderArmInZeroPos = Vector(0, -leftUnderArm.getLength(), 0);
	double underArmlengthLeft = leftUnderArm.getLength();
	Vector leftUnderArmWithoutRoll = leftRotationAroundX.multiply(leftRotationAroundZ.multiply(leftElbowRotation.multiply(leftUnderArmInZeroPos)));

	//calculating the angle betwenn actual under arm position and the one calculated without roll
	leftRoll = getAngle3D(leftUnderArmWithoutRoll, leftUnderArm);
	
	
	//This is a check which sign the angle has as the calculation only produces positive angles
	KMatrix leftRotationAroundArm = KMatrix::RotationY(-leftRoll*M_PI / 180);
	Vector leftShouldBeWristPos = leftRotationAroundX.multiply(leftRotationAroundZ.multiply(leftRotationAroundArm.multiply(leftElbowRotation.multiply(leftUnderArmInZeroPos))));
	double l1 = sqrt((leftUnderArm.x - leftShouldBeWristPos.x)*(leftUnderArm.x - leftShouldBeWristPos.x) + (leftUnderArm.y - leftShouldBeWristPos.y)*(leftUnderArm.y - leftShouldBeWristPos.y) + (leftUnderArm.z - leftShouldBeWristPos.z)*(leftUnderArm.z - leftShouldBeWristPos.z));
	double l1saver = l1;
	leftRotationAroundArm = KMatrix::RotationY(leftRoll*M_PI / 180);
	leftShouldBeWristPos = leftRotationAroundX.multiply(leftRotationAroundZ.multiply(leftRotationAroundArm.multiply(leftElbowRotation.multiply(leftUnderArmInZeroPos))));
	l1 = sqrt((leftUnderArm.x - leftShouldBeWristPos.x)*(leftUnderArm.x - leftShouldBeWristPos.x) + (leftUnderArm.y - leftShouldBeWristPos.y)*(leftUnderArm.y - leftShouldBeWristPos.y) + (leftUnderArm.z - leftShouldBeWristPos.z)*(leftUnderArm.z - leftShouldBeWristPos.z));
	if (l1 < l1saver){
		leftRoll = -leftRoll;
	}
	else{
		l1 = l1saver;
	}
	
	//As there are some singularities or inaccessible areas in the kinematic structure, 
	//this smoothes these areas out or removes them 
	if (topView.isOnFront(leftUpperArm)){
		leftYaw = 180 - leftYaw;
		if ((leftPitch > 15 && leftPitch < 90)|| (leftPitch < -15 && leftPitch > -90) || leftPitch >165 || leftPitch < -165)
			leftPitch = -90;
		else{
			double comparer = abs(leftPitch);
			if (comparer > 165) comparer = -comparer + 180;
			leftPitch = -comparer/15*90;
		}
	}



	newAngles[LEFT_ELBOW] = leftElbow;
	newAngles[LEFT_SHOULDER_YAW] = leftYaw;
	newAngles[LEFT_SHOULDER_PITCH] = leftPitch;
	newAngles[LEFT_SHOULDER_ROLL] = leftRoll;


	////////////////////////////////////////////////////////////////////////////////////////
	//										Right Arm									  //
	////////////////////////////////////////////////////////////////////////////////////////

	//Recreating arm with upper and under arm
	Vector rightUpperArm = Vector(lastPositions[JointType_ShoulderRight], lastPositions[JointType_ElbowRight]);
	Vector rightUnderArm = Vector(lastPositions[JointType_ElbowRight], lastPositions[JointType_WristRight]);

	
	double rightYaw = 0;
	double rightPitch = 0;
	double rightRoll = 0;
	double rightElbow = getAngle3D(rightUpperArm, rightUnderArm);

	double armlengthRight = rightUpperArm.getLength();
	rightYaw = asin(-rightUpperArm.x / armlengthRight);//Comes from robot structure
	rightPitch = atan(rightUpperArm.z / rightUpperArm.y);//Comes from robot structure
	rightYaw = rightYaw * 180 / M_PI;
	rightPitch = rightPitch * 180 / M_PI;
	
	//Because of the form of the sinus it has to be checked if the angle is bigger than 90°
	if (topView.isOnFront(rightUpperArm)){
		rightYaw = 180 - rightYaw;
	}

	//Recreating under Arm Position with known Angles(without roll)
	KMatrix rightRotationAroundZ = KMatrix::RotationZ(-rightYaw*M_PI / 180);
	KMatrix rightRotationAroundX = KMatrix::RotationX(rightPitch*M_PI / 180);
	KMatrix rightElbowRotation = KMatrix::RotationX(-rightElbow*M_PI / 180);
	Vector rightUnderArmInZeroPos = Vector(0, -rightUnderArm.getLength(), 0);
	double underArmlengthRight = rightUnderArm.getLength();
	Vector rightUnderArmWithoutRoll = rightRotationAroundX.multiply(rightRotationAroundZ.multiply(rightElbowRotation.multiply(rightUnderArmInZeroPos)));
	
	//calculating the angle betwenn actual under arm position and the one calculated without roll
	rightRoll = getAngle3D(rightUnderArmWithoutRoll, rightUnderArm);
	
	//This is a check which sign the angle has as the calculation only produces positive angles
	KMatrix rightRotationAroundArm = KMatrix::RotationY(rightRoll*M_PI / 180);
	Vector rightShouldBeWristPos = rightRotationAroundX.multiply(rightRotationAroundZ.multiply(rightRotationAroundArm.multiply(rightElbowRotation.multiply(rightUnderArmInZeroPos))));
	double r1 = sqrt((rightUnderArm.x - rightShouldBeWristPos.x)*(rightUnderArm.x - rightShouldBeWristPos.x) + (rightUnderArm.y - rightShouldBeWristPos.y)*(rightUnderArm.y - rightShouldBeWristPos.y) + (rightUnderArm.z - rightShouldBeWristPos.z)*(rightUnderArm.z - rightShouldBeWristPos.z));
	double r1saver = r1;
	rightRotationAroundArm = KMatrix::RotationY(-rightRoll*M_PI / 180);
	rightShouldBeWristPos = rightRotationAroundX.multiply(rightRotationAroundZ.multiply(rightRotationAroundArm.multiply(rightElbowRotation.multiply(rightUnderArmInZeroPos))));
	r1 = sqrt((rightUnderArm.x - rightShouldBeWristPos.x)*(rightUnderArm.x - rightShouldBeWristPos.x) + (rightUnderArm.y - rightShouldBeWristPos.y)*(rightUnderArm.y - rightShouldBeWristPos.y) + (rightUnderArm.z - rightShouldBeWristPos.z)*(rightUnderArm.z - rightShouldBeWristPos.z));
	if (r1 < r1saver){
		rightRoll = -rightRoll;
	}

	//As there are some singularities or inaccessible areas in the kinematic structure, 
	//this smoothes these areas out or removes them 
	if (topView.isOnFront(rightUpperArm)){
		rightPitch = -90;
		if ((rightPitch > 15 && rightPitch < 90) || (rightPitch < -15 && rightPitch > -90) || rightPitch >165 || rightPitch < -165)
			rightPitch = -90;
		else{
			double comparer = abs(rightPitch);
			if (comparer > 165) comparer = -comparer + 180;
			rightPitch = -comparer / 15 * 90;
		}
	}

	newAngles[RIGHT_ELBOW] = rightElbow;
	newAngles[RIGHT_SHOULDER_YAW] = rightYaw;
	newAngles[RIGHT_SHOULDER_PITCH] = rightPitch;
	newAngles[RIGHT_SHOULDER_ROLL] = rightRoll;



	////////////////////////////////////////////////////////////////////////////////////////
	//										Torso										  //
	////////////////////////////////////////////////////////////////////////////////////////

	double torsoYaw = 0; //rotation around gravitation
	double torsoRoll = 0; //left right
	double torsoPitch = 0; //front back

	//The angles are calculated with the assumption, that the 
	//human upper body is aligned with gravity in zero position
	Vector shoulders = Vector(lastPositions[JointType_ShoulderLeft], lastPositions[JointType_ShoulderRight]);
	Vector spineHelper = Vector(lastPositions[JointType_SpineBase], lastPositions[JointType_SpineShoulder]);
	Vector alignWithGravity = gravity;
	//Vector bendingForwardsGrav = sideView.projectToPlain(alignWithGravity);
	//Vector bendingSidewardsGrav = frontView.projectToPlain(alignWithGravity);
	Vector bendingForwardsGrav = Vector(0, gravity.y, gravity.z);
	Vector bendingSidewardsGrav = Vector(gravity.x,gravity.y, 0);
	Vector hip = Vector(lastPositions[JointType_HipLeft], lastPositions[JointType_HipRight]);


	torsoRoll = getAngle3D(spineHelper, bendingSidewardsGrav);
	torsoPitch = getAngle3D(spineHelper, bendingForwardsGrav);
	torsoYaw = getAngle3D(ground.projectToPlain(shoulders), ground.projectToPlain(hip));

	if (spineHelper.z < gravity.z){
		torsoPitch = -torsoPitch;
	}
	if (shoulders.z < hip.z){
		torsoYaw = -torsoYaw;
	}
	if (lastPositions[JointType_SpineShoulder].x < lastPositions[JointType_SpineBase].x){
		torsoRoll = -torsoRoll;
	}
	newAngles[TORSO_YAW] = torsoYaw;
	newAngles[TORSO_ROLL] = -torsoRoll;
	newAngles[TORSO_PITCH] = torsoPitch;
}

/// <summary>
/// Calculates the angle between two vectors
/// <param name="vec1">One vector</param>
/// <param name="vec2">Another vector</param>
/// </summary>
double KinectAdapter::getAngle3D(Vector vec1, Vector vec2){
	double scalarMul = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
	double normMul = sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z)*sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);
	return acos(scalarMul / normMul) / M_PI * 180;
}

/// <summary>
/// Sorts the angles correct for the iCub to read
/// </summary>
std::vector<double*> KinectAdapter::iCubSorting(){
	
	leftArmValues[0] = newAngles[LEFT_SHOULDER_PITCH];
	leftArmValues[1] = newAngles[LEFT_SHOULDER_YAW];
	leftArmValues[2] = newAngles[LEFT_SHOULDER_ROLL];
	leftArmValues[3] = newAngles[LEFT_ELBOW];

	rightArmValues[0] = newAngles[RIGHT_SHOULDER_PITCH];
	rightArmValues[1] = newAngles[RIGHT_SHOULDER_YAW];
	rightArmValues[2] = newAngles[RIGHT_SHOULDER_ROLL];
	rightArmValues[3] = newAngles[RIGHT_ELBOW];

	neckValues[0] = newAngles[NECK_PITCH];
	neckValues[1] = newAngles[NECK_ROLL];
	neckValues[2] = newAngles[NECK_YAW];

	torsoValues[0] = newAngles[TORSO_YAW];
	torsoValues[1] = newAngles[TORSO_ROLL];
	torsoValues[2] = newAngles[TORSO_PITCH];
	
	//logging from robot view
	int z = 0;
	myfile.open("newAngles.txt", ios::app);
	myfile << actualTime << " ";
	for (z = 0; z < 7; z++){
		myfile << leftArmValues[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 7; z++){
		myfile << rightArmValues[z] << "  ";
	}
	myfile << "     ";
	for (z = 0; z < 3; z++){
		myfile << torsoValues[z] << "  ";
	}
	myfile << "\n";
	myfile.close();
	std::vector<double*> returnValues = { leftArmValues, rightArmValues, torsoValues, neckValues };
	return returnValues;
}
/// <summary>
/// Logs the Kinect data
/// <param name="new Pos">3-d coordinate positions</param>
/// </summary>
void KinectAdapter::logKinect(std::vector<Vector> newPos){
	ofstream myfile;
	myfile.open("kinectValues.txt", ios::app);
	myfile << actualTime << "\t";
	myfile.close();
	int i = 0;
	for (i = 0; i < newPos.size(); i++){
		myfile.open("kinectValues.txt", ios::app);
		myfile << newPos[i].x << " " << newPos[i].y << " " << newPos[i].z << "\t";
		myfile.close();
		if (newPos[i].x != newPos[i].x){
			newPos[i].x = lastPositions[i].x;
			myfile.open("mistakes.txt", ios::app);
			myfile <<"Kinect mistake x position \n";
			myfile.close();
		}
		if (newPos[i].y != newPos[i].y){
			newPos[i].y = lastPositions[i].y;
			myfile.open("mistakes.txt", ios::app);
			myfile << "Kinect mistake y position \n";
			myfile.close();
		}
		if (newPos[i].z != newPos[i].z){
			newPos[i].z = lastPositions[i].z;
			myfile.open("mistakes.txt", ios::app);
			myfile << "Kinect mistake z position \n";
			myfile.close();
		}
	}
	myfile.open("kinectValues.txt", ios::app);
	myfile << "\n";
	myfile.close();
	if (lastPositions[JointType_SpineBase].x == newPositions[JointType_SpineBase].x &&lastPositions[JointType_SpineBase].y == newPositions[JointType_SpineBase].y &&lastPositions[JointType_SpineBase].z == newPositions[JointType_SpineBase].z){
		myfile.open("equality.txt", ios::app);
		myfile << actualTime << " equal \n ";
		myfile.close();
	}
}

/// <summary>
/// Writes out actual angles to a specific filename
/// <param name="filename">Name of the file</param>
/// </summary>
void KinectAdapter::log(char* filename)
{
	ofstream myfile;
	myfile.open(filename,ios::app);
	myfile << newAngles[LEFT_SHOULDER_PITCH] << "  " << newAngles[LEFT_SHOULDER_YAW] << "  " << newAngles[LEFT_SHOULDER_ROLL] << "  " << newAngles[LEFT_ELBOW] << "        ";
	myfile << newAngles[RIGHT_SHOULDER_PITCH] << "  " << newAngles[RIGHT_SHOULDER_YAW] << "  " << newAngles[RIGHT_SHOULDER_ROLL] << "  " << newAngles[RIGHT_ELBOW] << "        ";
	myfile << newAngles[NECK_PITCH] << "  " << newAngles[NECK_ROLL] << " " << newAngles[NECK_YAW] << "        ";
	myfile << newAngles[TORSO_YAW] << "  " << newAngles[TORSO_ROLL] << "  " << newAngles[TORSO_PITCH] << "        \n";
	myfile.close();
}
/// <summary>
/// basic logging of everything
/// </summary>
void KinectAdapter::logging(){
	ofstream myfile;
	int i = 0;
	double* leftArmRobotValues = yAdapter->getJoints(YarpAdapter::MOVE_LEFT_ARM);
	double* rightArmRobotValues = yAdapter->getJoints(YarpAdapter::MOVE_RIGHT_ARM);
	double* torsoRobotValues = yAdapter->getJoints(YarpAdapter::MOVE_TORSO);
	myfile.open("anglesWithNames.txt", ios::app);
	myfile << "Time:   " << actualTime << "   RS_YAW:  " << newAngles[RIGHT_SHOULDER_YAW] << "  RS_PITCH:  " << newAngles[RIGHT_SHOULDER_PITCH] << "  RS_ROLL:  " << newAngles[RIGHT_SHOULDER_ROLL]
		<< "     LS_YAW:  " << newAngles[LEFT_SHOULDER_YAW] << "  LS_PITCH:  " << newAngles[LEFT_SHOULDER_PITCH] << "  LS_ROLL:  " << newAngles[LEFT_SHOULDER_ROLL]
		<< "     T_YAW:  " << newAngles[TORSO_YAW] << "  T_PITCH:  " << newAngles[TORSO_PITCH] << " T_ROLL:  " << newAngles[TORSO_ROLL];
	//	<< "\n";
	myfile << "      RobotValues     RightArm:";
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_RIGHT_ARM); i++){
		myfile << "  " << rightArmRobotValues[i];
	}
	myfile << "     LeftArm:";
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_LEFT_ARM); i++){
		myfile << "  " << leftArmRobotValues[i];
	}
	myfile << "     Torso:";
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_TORSO); i++){
		myfile << "  " << torsoRobotValues[i];
	}
	myfile << "\n";
	myfile.close();
	myfile.open("angles.txt", ios::app);
	myfile << actualTime << "  " << newAngles[RIGHT_SHOULDER_YAW] << "  " << newAngles[RIGHT_SHOULDER_PITCH] << "  " << newAngles[RIGHT_SHOULDER_ROLL]
		<< "  " << newAngles[LEFT_SHOULDER_YAW] << "  " << newAngles[LEFT_SHOULDER_PITCH] << "  " << newAngles[LEFT_SHOULDER_ROLL]
		<< "  " << newAngles[TORSO_YAW] << "  " << newAngles[TORSO_PITCH] << "  " << newAngles[TORSO_ROLL];
	//	<< "\n";
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_LEFT_ARM); i++){
		myfile << "  " << leftArmRobotValues[i];
	}
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_RIGHT_ARM); i++){
		myfile << "  " << rightArmRobotValues[i];
	}
	for (i = 0; i < yAdapter->getDof(YarpAdapter::MOVE_TORSO); i++){
		myfile << "  " << torsoRobotValues[i];
	}
	myfile << "\n";
	myfile.close();
}
