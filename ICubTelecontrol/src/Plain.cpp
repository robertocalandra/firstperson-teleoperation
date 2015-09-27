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

#include "Plain.h"

	Vector vector1;
	Vector vector2;
	Vector normal;

Plain::Plain(Vector vec1, Vector vec2)
{
	vector1 = vec1;
	vector2 = vec2;
	normal = vector1.getNormalVector(vector2);
	normal.normalize();
}

Plain::~Plain(){

}

Vector Plain::projectToPlain(Vector vecToProj){
	Vector projectedVector = Vector(0,0,0);
	float distance = normal.x * vecToProj.x + normal.y * vecToProj.y + normal.z * vecToProj.z;
	projectedVector.x = vecToProj.x - normal.x * distance;
	projectedVector.y = vecToProj.y - normal.y * distance;
	projectedVector.z = vecToProj.z - normal.z * distance;
	return projectedVector;
}

bool Plain::isOnFront(Vector vec){
	return 0 <= (normal.x * vec.x + normal.y * vec.y + normal.z * vec.z);
}
	
