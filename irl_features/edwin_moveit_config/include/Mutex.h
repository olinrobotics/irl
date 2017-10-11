#ifndef MUTEX_H_
#define MUTEX_H_

#include <pthread.h>

class Mutex
{
private:
	static int id_cnt;
	int id;
    pthread_mutex_t _mutex;
    Mutex & operator=(const Mutex & obj);
    Mutex(const Mutex & obj);

public:
    Mutex();
    virtual ~Mutex();
    bool operator==(const Mutex & obj);
    bool operator!=(const Mutex & obj);


    void lock();
    void unlock();

    static void lock ( int lock, Mutex * mtx);

};


#endif
