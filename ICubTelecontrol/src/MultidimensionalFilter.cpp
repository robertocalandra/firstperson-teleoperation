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

/// This class is simply a way to have a filter for more than one dimension
/// For further understanding look into the Filter class

#include "MultidimensionalFilter.h"

//#include <vector>

/// <summary>
/// Constructor
/// </summary>
MultidimensionalFilter::MultidimensionalFilter()
{
}
/// <summary>
/// Constuctor with default initialization
/// <param name="initialData">The initial data with which the filter is initialized</param>
/// <param name="useButterworth">If true butterworth is used, otherwise average filter</param>
/// <param name="dataSize"">amount of dimensions/filters</param>
/// </summary>
MultidimensionalFilter::MultidimensionalFilter(float initialData[], bool useButterworth, int dataSize){
	initialize(initialData, useButterworth, dataSize,Filter::KINECT_FILTER);
}
/// <summary>
/// Destructor
/// </summary>
MultidimensionalFilter::~MultidimensionalFilter()
{
}


/// <summary>
/// Initializes the multiple filters with given data and some additional Information
/// </summary>
/// <param name="initData">The initial data with which the filter is initialized</param>
/// <param name="useButterworth">If true butterworth is used, otherwise average filter</param>
/// <param name="dataSize"">amount of dimensions/filters</param>
/// <param name="additionalData"">If useButterworth = true, this says which special filter is used, otherwise its the amount of data to average over</param>
void MultidimensionalFilter::initialize(double* initData, bool useButterworth, int dataSize, int additionalData){
	inputDataSize = dataSize;
	filters.resize(dataSize);
	for (int i = 0; i < dataSize; i++){
		filters[i] = new Filter(initData[i], useButterworth, additionalData);
	}
}
/// <summary>
/// Initializes the multiple filters with given data and some additional Information
/// </summary>
/// <param name="initData">The initial data with which the filter is initialized</param>
/// <param name="useButterworth">If true butterworth is used, otherwise average filter</param>
/// <param name="dataSize"">amount of dimensions/filters</param>
/// <param name="additionalData"">If useButterworth = true, this says which special filter is used, otherwise its the amount of data to average over</param>
void MultidimensionalFilter::initialize(std::vector<double> initData, bool useButterworth, int dataSize, int additionalData){
	inputDataSize = dataSize;
	filters.resize(dataSize);
	for (int i = 0; i < dataSize; i++){
		filters[i] = new Filter(initData[i], useButterworth, additionalData);
	}
}

/// <summary>
/// Filters the given data
/// </summary>
/// <param name="data">To be filtered data</param>
void MultidimensionalFilter::filterData(double* data){
	for (int i = 0; i < inputDataSize; i++){
		double temp = filters[i]->filterData(data[i]);
		double dif = temp - data[i];
		data[i] = temp;
	}
}

/// <summary>
/// Filters the given data
/// </summary>
/// <param name="data">To be filtered data</param>
std::vector<double> MultidimensionalFilter::filterData(std::vector<double> data){
	for (int i = 0; i < inputDataSize; i++){
		double temp = filters[i]->filterData(data[i]);
		double dif = temp - data[i];
		data[i] = temp;
	}
	return data;
}
