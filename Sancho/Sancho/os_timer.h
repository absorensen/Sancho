// Timer function that is as precise as posible 
// and works for several platforms.
//
// Created by Bent Dalgaard Larsen, 5 April 2002. 
// Modified by Jeppe Revall Frisvad, August 2009.
// Copyright (c) DTU Informatics 2009

#ifndef OSTIMER_H
#define OSTIMER_H

#ifdef WIN32
#include <windows.h>
static LARGE_INTEGER largeInteger;
#else
#include <sys/time.h>
#endif

class Timer
{
public:

	void start()
	{
#ifdef WIN32
		QueryPerformanceFrequency(&largeInteger);
		freq = largeInteger.QuadPart;
		QueryPerformanceCounter(&largeInteger);
		start_count = largeInteger.QuadPart;
#else
		gettimeofday(&start_time, 0);
#endif
	}

	void stop()
	{
#ifdef WIN32
		QueryPerformanceCounter(&largeInteger);
		stop_count = largeInteger.QuadPart;
#else
		gettimeofday(&stop_time, 0);
#endif
	}

	double get_time()
	{
#ifdef WIN32
		return (stop_count - start_count) / freq;
#else
		return (stop_time.tv_sec - start_time.tv_sec)
			+ (stop_time.tv_usec - start_time.tv_usec) / 1.0e6;
#endif
	}

private:
#ifdef WIN32
	double freq;
	double start_count;
	double stop_count;
#else
	timeval start_time;
	timeval stop_time;
#endif
};

#endif // OSTIMER_H