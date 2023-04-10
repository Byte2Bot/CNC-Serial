#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <stdlib.h>
#include <time.h>

class Timers
{
	private:
		u_int64_t timer_start;
	public:
		Timers();
		void Start();
		u_int64_t ElapsedMs();
		u_int64_t GetMs();  //returns current time in ms
		int Expired( u_int64_t ms);
	
};

#endif

