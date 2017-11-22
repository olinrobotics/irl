#include "Mutex.h"
#include <pthread.h>
#include <exception>

int Mutex::id_cnt = 0;

Mutex::Mutex()
{
	id = id_cnt;
	++id_cnt;
    if ( pthread_mutex_init(&_mutex, NULL) )
    {
        throw "Error: Initializing _mutex";
    }
}

Mutex::~Mutex()
{
    pthread_mutex_destroy(&_mutex);
}

Mutex::Mutex(const Mutex & obj)
{
	_mutex = obj._mutex;
	id = obj.id;
}

Mutex & Mutex::operator=(const Mutex & obj)
{
	_mutex = obj._mutex;
	id = obj.id;
	return *this;
}

bool Mutex::operator==(const Mutex & obj)
{
	return id == obj.id;
}

bool Mutex::operator!=(const Mutex & obj)
{
	return !(*this == obj);
}

void Mutex::lock ()
{
    pthread_mutex_lock( &_mutex );
}

void Mutex::unlock ()
{
    pthread_mutex_unlock( &_mutex );
}

void Mutex::lock ( int lock, Mutex * mtx)
{
	if (lock)
	{
		mtx->lock();
	}
	else
	{
		mtx->unlock();
	}
}

