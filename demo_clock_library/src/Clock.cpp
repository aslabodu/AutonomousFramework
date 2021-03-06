#include "Clock.h"
#include <ctime>
#include <time.h>
#include <cstdio>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>


struct timeval requestStart, requestEnd;

void ResetClock()
{
    // gettimeofday(&requestStart, NULL);
    while(gettimeofday(&requestStart, NULL) < 0);
} 

float ElapsedTime()
{
    // gettimeofday(&requestEnd, NULL);
    while(gettimeofday(&requestEnd, NULL) < 0);

    long seconds  = requestEnd.tv_sec  - requestStart.tv_sec;
    long useconds = requestEnd.tv_usec - requestStart.tv_usec;

    float elapsed = static_cast<float>(((seconds) * 1000 + useconds/1000.0) + 0.5)/1000.0f;
  
	return(elapsed);
}

