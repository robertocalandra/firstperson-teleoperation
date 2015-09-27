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

#include <KMatrix.h>

KMatrix::KMatrix(Vector x, Vector y, Vector z)
{
	X1 = x.x;
	X2 = x.y;
	X3 = x.z;
	Y1 = y.x;
	Y2 = y.y;
	Y3 = y.z;
	Z1 = z.x;
	Z2 = z.y;
	Z3 = z.z;
}
KMatrix::KMatrix(float a, float b, float c, float d,float e, float f, float g, float h, float i)
{
	X1 = a;
	X2 = b;
	X3 = c;
	Y1 = d;
	Y2 = e;
	Y3 = f;
	Z1 = g;
	Z2 = h;
	Z3 = i;
}


KMatrix::~KMatrix()
{
}

Vector  KMatrix::multiply(Vector v){
	Vector returnv = Vector();
	returnv.x = v.x*X1 + v.y*X2 + v.z*X3;
	returnv.y = v.x*Y1 + v.y*Y2 + v.z*Y3;
	returnv.z = v.x*Z1 + v.y*Z2 + v.z*Z3;
	return returnv;
}
KMatrix KMatrix::RotationX(float a){
	KMatrix temp = KMatrix(1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a));
	return temp;
}
KMatrix KMatrix::RotationY(float a){
	KMatrix temp = KMatrix(cos(a), 0, sin(a), 0, 1, 0, -sin(a), 0, cos(a));
	return temp;
}
KMatrix KMatrix::RotationZ(float a){
	KMatrix temp = KMatrix(cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1);
	return temp;
}
KMatrix KMatrix::multiply(KMatrix m){
	float a = X1*m.X1 + X2*m.Y1 + X3*m.Z1;
	float b = X1*m.X2 + X2*m.Y2 + X3*m.Z2;
	float c = X1*m.X3 + X2*m.Y3 + X3*m.Z3;
	float d = Y1*m.X1 + Y2*m.Y1 + Y3*m.Z1;
	float e = Y1*m.X2 + Y2*m.Y2 + Y3*m.Z2;
	float f = Y1*m.X3 + Y2*m.Y3 + Y3*m.Z3;
	float g = Z1*m.X1 + Z2*m.Y1 + Z3*m.Z1;
	float h = Z1*m.X2 + Z2*m.Y2 + Z3*m.Z2;
	float i = Z1*m.X3 + Z2*m.Y3 + Z3*m.Z3;
	KMatrix temp = KMatrix(a,b,c,d,e,f,g,h,i);
	return temp;
}
