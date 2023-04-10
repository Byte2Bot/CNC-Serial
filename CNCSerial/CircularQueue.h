//CircularQueue.h
#ifndef __CIRCULAR_QUEUE_H__
#define __CIRCULAR_QUEUE_H__

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#define CIRCULAR_QUEUE_SIZE 4096

#define CIRC_QUEUE_ERROR 0
#define CIRC_QUEUE_OK	1
#define CIRC_QUEUE_FULL 2

class CircularQueue
{
	private:
		unsigned char queue[CIRCULAR_QUEUE_SIZE];
		unsigned int front;
		unsigned int rear;
		unsigned int itemCount;
		unsigned int maxSize;
		
		pthread_mutex_t queueMutex = PTHREAD_MUTEX_INITIALIZER;  // Initialize the mutex

	public:
		CircularQueue();
		~CircularQueue();
		
		//these return int (boolean 1 or 0)
		int isEmpty();
		int isFull(); 
		unsigned int getSize();
		unsigned int getMaxSize();
		unsigned int spaceLeft();  //returns how many bytes are left in the queue
		unsigned int putData(unsigned char *data, unsigned int length);
		
		//returns the number of bytes extracted from the queue
		unsigned int getData(unsigned char *data, unsigned int length);
		
		void reset();
		void printStatus();
		
};

#endif  //__CIRCULAR_QUEUE_H__
