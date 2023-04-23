//Timers.cpp
//intended to be used with Debian/Ubuntu
//originally compiled on Raspberry Pi 4

#include "Timers.h"
#include <stdio.h>
#include <sys/time.h>


Timers::Timers()
{
	Start();
}

u_int64_t Timers::GetMs()
{
    // get current time//
    struct timeval nowTimeVal;
    gettimeofday(&nowTimeVal, NULL);
    
    //the timeval structure contains the time spread out in multiple variables. Combine them here.
    
    u_int64_t ms = ((u_int64_t)1000 * (u_int64_t)nowTimeVal.tv_sec) + ((u_int64_t)nowTimeVal.tv_usec / (u_int64_t)1000);
    
    return ms;
}

void Timers::Start()
{
	timer_start = GetMs();
}

u_int64_t Timers::ElapsedMs()
{
	return GetMs() - timer_start;
}

int Timers::Expired(u_int64_t ms)
{
	if(ms == 0)
		return 1;  //zero elapsed time means always expired

	if((GetMs() - timer_start) >= ms)	
	{
		return 1;
	}
	return 0;
}

