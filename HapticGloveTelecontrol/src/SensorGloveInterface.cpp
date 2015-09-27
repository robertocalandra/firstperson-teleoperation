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
*	 Original-Author: Elmar Rueckert, January 2015
*	 Authors: Lars Fritsche, Felix Unverzagt, Roberto Calandra
*	 Created: July, 2015
*/

#include <SensorGloveInterface.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;

#define SEGMENTLENGTH  17 //2+2*5+5 including start and end ASCII bytes
#define SEGMENT_START_ASCII  114
#define SEGMENT_END_ASCII  120
#define NUM_SENSOR_READINGS  5

double lastData[5] = {0,0,0,0,0};
double* rbStateData = new double[5];
double eta = 50;

bool verbose = true;

YarpAdapter* yAdapter;

SensorGloveInterface::SensorGloveInterface(YarpAdapter* yarpAdapter) {
	yAdapter = yarpAdapter;
}

SensorGloveInterface::~SensorGloveInterface() {

}

// returns glove finger output and wants vibration values
double* SensorGloveInterface::talkToGloves(double* vibrationData) {
	PolyDriver driver;
	ISerialDevice *iSerial;
	Property prop;
	prop.put("device", "serialport");
	prop.put("comport", "/dev/ttyACM0");
	prop.put("baudrate", 115200);
	prop.put("paritymode", "NONE");
	prop.put("databits", 8);
	prop.put("readtimeoutmsec", 1000);
	prop.put("verbose", 1);

	driver.open(prop);
	if (!driver.isValid()) {
		fprintf(stderr, "Error opening PolyDriver check parameters\n");
		return false;
	}

	driver.view(iSerial);
	if (!iSerial) {
		fprintf(stderr, "Error opening serial driver. Device not available\n");
		return false;
	}

	//Parse all incomming bytes
	double flgTerminateMex = 0.0;

	unsigned char segment[SEGMENTLENGTH];
	bool flgReadyToPopulate = false;
	int segmentPtrIndex = 0;
	int parsingMode = 0; // 0...LookingForSegmentStart, 1...fillingUpSegment, 2...CheckCorrectSegmentEnd
	char next_byte_signed;

	double* handValues;
	bool finished = false;
	while (!finished) {
		int success = iSerial->receiveChar(next_byte_signed);
		if (success) {
			unsigned char next_byte = reinterpret_cast<unsigned char&>(next_byte_signed);

			if (segmentPtrIndex == SEGMENTLENGTH - 1) {
				parsingMode = 2;
			}

			//mode 1
			switch (parsingMode) {
			case 0:
				if (next_byte == SEGMENT_START_ASCII) {
					parsingMode = 1;
					segmentPtrIndex = 0;
					segment[segmentPtrIndex++] = next_byte;
				}
				break;

			case 1:
				segment[segmentPtrIndex++] = next_byte;
				break;

			case 2:
				parsingMode = 0;
				segmentPtrIndex = 0;
				if (next_byte == SEGMENT_END_ASCII) {
					segment[segmentPtrIndex] = next_byte;
					flgReadyToPopulate = true;
				}
				break;
			}
		}

		if (flgReadyToPopulate) {
			handValues = PopulateData(segment, vibrationData, iSerial);

			if (verbose) {
				for (int i = 0; i < 5; i++) {
					cout << "i: " << i << " " << handValues[i] << " | ";
				}
				cout << "\n";
			}

			yAdapter->writeToOutput(handValues);

			flgReadyToPopulate = false;
			//finished = true;
		} else {
			usleep(10); //wait for 100 micro seconds, otherwise you have 100% cpu usage
		}
	}

	//close serial connection
	driver.close();
	return 0;
}

double* SensorGloveInterface::PopulateData(unsigned char* pData,
		double* vibrationData, ISerialDevice* serial) {
	//callback arguments pointer

	rbStateData[0] = 256.0 * pData[1] + pData[2];
	rbStateData[1] = 256.0 * pData[4] + pData[5];
	rbStateData[2] = 256.0 * pData[7] + pData[8];
	rbStateData[3] = 256.0 * pData[10] + pData[11];
	rbStateData[4] = 256.0 * pData[13] + pData[14];

	//double *returnValuesPtr = vibrationData;

	char buffer[20];
	sprintf(buffer, "w,%u,%u,%u,%u,%ux", (unsigned int) vibrationData[0],
			(unsigned int) vibrationData[1], (unsigned int) vibrationData[2],
			(unsigned int) vibrationData[3], (unsigned int) vibrationData[4]);
	std::string pwmCmd(buffer);
	//serial->Write(pwmCmd);
	return rbStateData;
}
