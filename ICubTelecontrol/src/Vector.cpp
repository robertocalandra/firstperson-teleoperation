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

#include <Vector.h>
#include <math.h>
#include <KMatrix.h>

Vector::Vector()
{
	x = 0;
	y = 0;
	z = 0;
	length = 0;
}
Vector::Vector(float xNew, float yNew, float zNew)
{
	x = xNew;
	y = yNew;
	z = zNew;
	calculateLength();
}

Vector::Vector(Vector v1, Vector v2){
	x = v2.x - v1.x;
	y = v2.y - v1.y;
	z = v2.z - v1.z;
	calculateLength();
}

Vector::~Vector()
{
}

void Vector::calculateLength(){
	length = sqrt(x*x + y*y + z*z);
}

float Vector::scalarProdukt(Vector v){
	return v.x * x + v.y*y + v.z*z;
}
Vector Vector::getNormalVector(Vector v2){
	Vector returnVector = Vector();
	returnVector.x = y * v2.z - z * v2.y;
	returnVector.y = z * v2.x - x * v2.z;
	returnVector.z = x * v2.y - y * v2.x;
	return returnVector;
}
float Vector::getLength(){
	calculateLength();
	return length;
}
Vector Vector::rotateAroundVector(Vector v2, float angle){
	normalize();
	float a = x*x + (1- x*x)*cos(angle);
	float b = x*y*(1-cos(angle)) - z*sin(angle);
	float c = x*z*(1-cos(angle)) + y*sin(angle);
	float d = x*y* (1 - cos(angle))+z*sin(angle);
	float e = y*y*(1 - y*y)*cos(angle);
	float f = y*z*(1 - cos(angle)) - x*sin(angle);
	float g = x*z + (1 - cos(angle))-y*sin(angle);
	float h = z*y*(1 - cos(angle)) + x*sin(angle);
	float i = z*z*(1 -z*z)*cos(angle);
	KMatrix temp = KMatrix(a, b, c, d, e, f, g, h, i);
	return temp.multiply(v2);
}



void Vector::normalize(){
	calculateLength();
	x /= length;
	y /= length;
	z /= length;
	length = 1;
}
