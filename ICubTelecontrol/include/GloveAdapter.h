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

#ifndef GLOVEADAPTER_H_
#define GLOVEADAPTER_H_

class GloveAdapter {


public:
	static const int NUM_OF_FINGERS = 5;

	GloveAdapter();
	~GloveAdapter();

	int* calculateVibrationIntensities(double* pressureValues);
	double* calculateFingerAngles(double* gloveValues);

private:

	void logAngles(char* filename, double* values, int size);
	double actualTime = 0;
};

#endif /* GLOVEADAPTER_H_ */
