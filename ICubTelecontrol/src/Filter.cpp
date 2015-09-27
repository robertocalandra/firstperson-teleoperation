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

#include "Filter.h"
#include <vector>
using namespace std;
/// This class filters data
/// It has special butterworth filters for Kinect, Oculus and Glove
/// If those are not wanted a simple average filter can be used



int filterDataSize = 100;
int dataSize;
bool useButterworthFilter = false;
int filterOrder = 8;
vector<double> aValues;
vector<double> bValues;

/// hand designed filters for Kinect Oculus and Glove.
double aValuesKinect[] = { 1, -7.758453298128671, 26.338251329996360, -51.099725312184745, 61.970846140298725, -48.105287706158710, 23.341788845289320, -6.472807463017559, 0.785387463926867 };
double bValuesKinect[] = { 8.431233142203176e-14, 6.744986513762541e-13, 2.360745279816889e-12, 4.721490559633779e-12, 5.901863199542223e-12, 4.721490559633779e-12, 2.360745279816889e-12, 6.744986513762541e-13, 8.431233142203176e-14 };

double aValuesOculus[] = {1,	-3.589733887112176,	4.851275882519418,	-2.92405265616246,	0.663010484385891};
double bValuesOculus[] = {3.123897691706401e-05,	0.000124955907668256,	0.0001874338615023841,	0.000124955907668256,	3.123897691706401e-05};

double aValuesGlove[] = {1,	-3.589733887112176,	4.851275882519418,	-2.92405265616246,	0.663010484385891};
double bValuesGlove[] = {3.123897691706401e-05,	0.000124955907668256,	0.0001874338615023841,	0.000124955907668256,	3.123897691706401e-05};

/// <summary>
/// Constructor
/// </summary>
Filter::Filter()
{
}

/// <summary>
/// Destructor
/// </summary>
Filter::~Filter()
{
}
/// <summary>
/// Constructor with simple initialization
/// <param name="initialData">The initial data with which the filter is initialized</param>
/// <param name="useButterworth">If true butterworth is used, otherwise average filter</param>
/// <param name="dataSize"">amount of dimensions/filters</param>
/// <param name="additionalData"">If useButterworth = true, this says which special filter is used, otherwise its the amount of data to average over</param>
/// </summary>
Filter::Filter( double initialData, bool useButterworth, int additionalData){
	initialize(initialData,useButterworth, additionalData);
}

/// <summary>
/// Constructor with simple initialization
/// <param name="initData">The initial data with which the filter is initialized</param>
/// <param name="useButterworth">If true butterworth is used, otherwise average filter</param>
/// <param name="dataSize"">amount of dimensions/filters</param>
/// <param name="additionalData"">If useButterworth = true, this says which special filter is used, otherwise its the amount of data to average over</param>
/// </summary>
void Filter::initialize(double initData, bool useButterworth, int additionalData){
	useButterworthFilter = useButterworth;
	if (useButterworth){
		if(additionalData == Filter::OCULUS_FILTER || additionalData == Filter::GLOVE_FILTER) filterOrder = 4;
		if(additionalData == Filter::KINECT_FILTER) filterOrder = 8;
		aValues.resize(filterOrder+1);
		bValues.resize(filterOrder+1);
		for(int i = 0; i < filterOrder+1; i++){
			if(additionalData == Filter::OCULUS_FILTER ){
				aValues[i] = aValuesOculus[i];
				bValues[i] = bValuesOculus[i];
			}
			else if (additionalData == Filter::GLOVE_FILTER ){
				aValues[i] = aValuesGlove[i];
				bValues[i] = bValuesGlove[i];
			}
			else if(additionalData == Filter::KINECT_FILTER){
				aValues[i] = aValuesKinect[i];
				bValues[i] = bValuesKinect[i];
			}
			else{
				aValues[i] = 0;
				bValues[i] = 0;
			}
		}

		initializeButterworthFilter(initData);
	}
	else{
		filterDataSize = additionalData;
		initializeAverageFilter( initData);
	}
}

/// <summary>
/// Initializes the butterworth filter(simply writes the data into the memory)
/// <param name="initData">The initial data with which the filter is initialized</param>
/// </summary>
void Filter::initializeButterworthFilter(double initData){

	savedDataButterworthFilter.resize(filterOrder + 1);
	lastCalculatedValuesButterworth.resize(filterOrder + 1);
	for (int i = 0; i < filterOrder+1; i++){
			savedDataButterworthFilter[i] = initData;
			lastCalculatedValuesButterworth[i] = initData;
	}

}
/// <summary>
/// Initializes the average filter(simply writes the data into the memory)
/// <param name="initData">The initial data with which the filter is initialized</param>
/// </summary>
void Filter::initializeAverageFilter(double initData){
	savedDataAverageFilter.resize(filterDataSize,0);
	for (int i = 0; i < filterDataSize; i++){
			savedDataAverageFilter[i] = initData;
	}
}
/// <summary>
/// Filters given data
/// <param name="data">Data to be filtered</param>
/// </summary>
double Filter::filterData(double data){
	if (useButterworthFilter){
		return butterworthFilter(data);
	}
	else{
		return averageFilter(data);
	}	
}

/// <summary>
/// Filters data with butterworth filter
/// <param name="data">Data to be filtered</param>
/// </summary>
double Filter::butterworthFilter(double data){

	for (int i = 0; i < filterOrder; i++){
			savedDataButterworthFilter[i] = savedDataButterworthFilter[i + 1];
			lastCalculatedValuesButterworth[i] = lastCalculatedValuesButterworth[i + 1];
	}
	savedDataButterworthFilter[filterOrder] = data;
	lastCalculatedValuesButterworth[filterOrder] = 0;

	for (int i = 0; i < filterOrder + 1; i++){
			lastCalculatedValuesButterworth[filterOrder] += bValues[i] * savedDataButterworthFilter[filterOrder - i];
			if (i != 0){
				lastCalculatedValuesButterworth[filterOrder] -= aValues[i] * lastCalculatedValuesButterworth[filterOrder - i];
		}
	}
	return lastCalculatedValuesButterworth[filterOrder];
}

/// <summary>
/// Filters data with average filter
/// <param name="data">Data to be filtered</param>
/// </summary>
double Filter::averageFilter(double data){

	for (int i = 0; i < filterDataSize - 1; i++){
		savedDataAverageFilter[i] = savedDataAverageFilter[i + 1];
	}
	savedDataAverageFilter[filterDataSize - 1] = data;

	double output = 0;
	for (int i = 0; i < filterDataSize; i++){
			output += savedDataAverageFilter[i] / filterDataSize;
	}
	return output;
}
