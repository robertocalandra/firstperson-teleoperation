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


std::vector<double> savedDataAverageFilter;
std::vector<double> savedDataButterworthFilter; //= { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
std::vector<double> lastCalculatedValuesButterworth;// = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };


const int filterDataSize = 100;
int dataSize;
bool useButterworthFilter = false;
const int filterOrder = 8;
double aValues[] = { 1, -7.51692089667645, 24.7343761161550, -46.5329915081609, 54.7434921806640, -41.2391933030154, 19.4262472271955, -5.23173586044389, 0.616726049200009 };
double bValues[] = { 1.92109136758212e-11, 1.53687309406569e-10, 5.37905582922993e-10, 1.07581116584599e-09, 1.34476395730748e-09, 1.07581116584599e-09, 5.37905582922993e-10, 1.53687309406569e-10, 1.92109136758212e-11 };


Filter::Filter()
{
}


Filter::~Filter()
{
}

Filter::Filter( double initialData, bool useButterworth){
	initialize(initialData,useButterworth);
}

void Filter::initialize(double initData, bool useButterworth){
	useButterworthFilter = useButterworth;
	if (useButterworth) initializeButterworthFilter(initData);
	else initializeAverageFilter( initData);
}
void Filter::initializeButterworthFilter(double initData){

	savedDataButterworthFilter.resize(filterOrder + 1,0);
	lastCalculatedValuesButterworth.resize(filterOrder + 1,0);
	for (int i = 0; i < filterOrder+1; i++){
			savedDataButterworthFilter[i] = initData;
			lastCalculatedValuesButterworth[i] = initData;
	}

}

void Filter::initializeAverageFilter(double initData){
	savedDataAverageFilter.resize(filterDataSize,0);
	for (int i = 0; i < filterDataSize; i++){
			savedDataAverageFilter[i] = initData;
	}
}

double Filter::filterData(double data){


	if (useButterworthFilter){
		return butterworthFilter(data);
	}
	else{
		return averageFilter(data);
	}	
}
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