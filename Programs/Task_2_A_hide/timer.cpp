
// windows time measurement functions

#include <iostream>
#include <cmath>

#include <windows.h>

#include "timer.h"

using namespace std;

double high_resolution_time()
// windows high resolution time function
// typically has 0.1 us resolution (i.e. intervals),
// but not very accurate over long periods
{
	static int init=0;
	static double pow32, count_low0, count_high0, timer_frequency;
	double t, count_low, count_high;
	LARGE_INTEGER count;

	if(init==0) {
		pow32 = pow(2.0,32); // calculate a constant, 2^32
		
		QueryPerformanceCounter(&count); // get initial count
		count_low0  = count.LowPart;
		count_high0 = count.HighPart;
		
		// read the timer frequency
		// assume the frequency is constant for subsequent timer reads
		QueryPerformanceFrequency(&count);
		timer_frequency = count.LowPart;
		
		init=1;
	}

	// read the timer
	QueryPerformanceCounter(&count);
	count_low  = count.LowPart  - count_low0;
	count_high = count.HighPart - count_high0; 

	// calculate the time
	t = (count_low + count_high*pow32) / timer_frequency;

	return t;
}


// returns the integer count of the high resolution clock
// 1 count typically = 0.1 us
unsigned int high_resolution_count()
{
	LARGE_INTEGER count;
	unsigned int ans;
	
	// read the timer
	QueryPerformanceCounter(&count);
	
	// take the low part of the counter
	ans = count.LowPart;	
	
	return ans;
}


