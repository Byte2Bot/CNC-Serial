//CircularQueue.c

#include "CircularQueue.h"


CircularQueue::CircularQueue()
{
	reset();
}

CircularQueue::~CircularQueue()
{
	//not sure if this is needed or if it causes problems
	//pthread_mutex_unlock(&itemCountMutex);  // Release the mutex
}

void CircularQueue::reset()
{
	front = 0;
	rear = -1;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	itemCount = 0;
	maxSize = 0;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex	
}

void CircularQueue::printStatus()
{
	unsigned int ic, ms;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	ic = itemCount;
	ms = maxSize;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	
	printf("Queue size: %d, max: %d\n", ic, ms);
}

unsigned int CircularQueue::getMaxSize()
{
	int retval;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	retval = maxSize;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	return retval;
}

// Function to check if the queue is empty
int CircularQueue::isEmpty() 
{
	unsigned int retval;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	retval = itemCount;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	
    return (retval == 0);
}

unsigned int CircularQueue::spaceLeft()
{
	unsigned int retval;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	retval = itemCount;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex	
	
	return (CIRCULAR_QUEUE_SIZE - retval - 1);
	
}

// Function to check if the queue is full
int CircularQueue::isFull() 
{
	int retval;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	retval = itemCount;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	
    return (retval == CIRCULAR_QUEUE_SIZE);
}

// Function to get the size of the queue
unsigned int CircularQueue::getSize() 
{
	unsigned int retval;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	retval = itemCount;
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	
    return retval;
}

// Function to add item(s) to the rear of the queue
unsigned int CircularQueue::putData(unsigned char *data, unsigned int length) 
{
	unsigned int retval = CIRC_QUEUE_OK;
	//add the data byte by byte for now..
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	for(unsigned int n = 0; n < length; n++)
	{
		if(itemCount < CIRCULAR_QUEUE_SIZE) 
		{
			if(rear == CIRCULAR_QUEUE_SIZE-1) 
			{
				rear = -1;
			}
			queue[++rear] = data[n];

			itemCount++;
			if(itemCount > maxSize)
				maxSize = itemCount;
		}
		else
		{
			retval = CIRC_QUEUE_FULL;
			n = length;  //get out of the for loop
		}
	}
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
	return retval;
}

// Function to remove item(s) from the front of the queue
unsigned int CircularQueue::getData(unsigned char *data, unsigned int length)  //returns number of bytes recieved
{
	unsigned int n = 0;
	unsigned int retval = length;
	pthread_mutex_lock(&queueMutex);  // Obtain the mutex
	for(n = 0; n < length; n++)
	{
		if(itemCount > 0)
		{
			data[n] = queue[front++];
			if(front == CIRCULAR_QUEUE_SIZE) 
			{
				front = 0;
			}			
			itemCount--;			
		}
		else
		{
			retval = n;  //this is how many bytes we pulled from the queue before it was empty.
			n = length;
		}
	}
	pthread_mutex_unlock(&queueMutex);  // Release the mutex
    return retval;
}




