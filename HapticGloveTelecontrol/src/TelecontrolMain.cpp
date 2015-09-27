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

#include <iostream>
#include <TelecontrolMain.h>
#include <SensorGloveInterface.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <YarpAdapter.h>

#include <fstream>

using namespace std;

clock_t lastClock;
clock_t currentClock;
double lastTime;
double currentTime;

int fps = 200;
double spf = double(1) / double(fps) * 1000000;

int main(int ac, char** av) {
	bool blocking = false;
	YarpAdapter* yAdapter = new YarpAdapter();
	SensorGloveInterface* sInterface = new SensorGloveInterface(yAdapter);
	double vibrationValues[] = {0, 0, 0, 0, 0};
	int i = 0;
	while(true) {
		i++;
		lastClock = clock();
		lastTime = ((double) (lastClock));
		if(!blocking) {
			double* handValues = sInterface->talkToGloves(vibrationValues);
			blocking = true;
		}
		//double* vibrationValues = yAdapter->readGloveVibrationInput();
		//yAdapter->writeToOutput(handValues);
		currentClock = clock();
		currentTime = ((double) (currentClock));
		
		// limit updates per second
		if ((currentTime - lastTime) > spf) {
			blocking = false;
		}
	}
}

