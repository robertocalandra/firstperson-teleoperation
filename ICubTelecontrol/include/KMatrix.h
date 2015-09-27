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

#ifndef KMATRIX
#define KMATRIX

#include "Vector.h"
#include <math.h>
class KMatrix
{
public:

	float X1 = 0;
	float X2 = 0;
	float X3 = 0;
	float Y1 = 0;
	float Y2 = 0;
	float Y3 = 0;
	float Z1 = 0;
	float Z2 = 0;
	float Z3 = 0;

	KMatrix(Vector x, Vector y, Vector z);
	KMatrix(float a, float b, float c, float d, float e, float f, float g, float h, float i);
	~KMatrix();
	Vector  multiply(Vector v);
	KMatrix multiply(KMatrix m);
	static KMatrix RotationX(float a);
	static KMatrix RotationY(float a);
	static KMatrix RotationZ(float a);

private:

};

#endif
