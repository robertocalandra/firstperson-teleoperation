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

#ifndef myFilter

#define myFilter

#include <vector>

using namespace std;

class Filter

{
public:

	//creates a normal Filter with call to initialize

	Filter(double initialData, bool useButterworth, int additionalData);

	Filter();
	~Filter();

	//dataSize:			The size of the data (For 30 joints this is 30)(dataSize
	//initData:			An array with the initial Position data
	//additionalData:	AdditionalData for the filter. If average then amount of angles, if Butterworth filter type
	void initialize(double initData, bool useButterworth, int additionalData);

	double filterData(double data);
	void initializeButterworthFilter(double initData);
	void initializeAverageFilter(double initData);


	static const int OCULUS_FILTER = 1; 	//butter(4,0.05)
	static const int GLOVE_FILTER = 2;		//butter(4,0.05)
	static const int KINECT_FILTER = 3;		//butter(8,0.015)

private:
	double averageFilter(double data);
	double butterworthFilter(double data);
	
	vector<double> savedDataAverageFilter;
	vector<double> savedDataButterworthFilter; //= { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	vector<double> lastCalculatedValuesButterworth;// = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

};

#endif

