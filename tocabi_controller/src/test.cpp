/*
   Demo of a simple periodic thread using Posix
   under 'standard' Linux
   and under Xenomai versions 2 and 3 using posix skin
   Marc Le Douarain, august 2015
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/timerfd.h>

#define PERIOD_MICROSECS 10000 //10millisecs
#define START_DELAY_SECS 1 //1sec

pthread_t MyPosixThread;
int TimerFdForThread = -1;
char ThreadRunning = 0;
int ResultIncValue = 0;

int CreatePosixTask( char * TaskName, int Priority, int StackSizeInKo, unsigned int PeriodMicroSecs, void * (*pTaskFunction)(void *) )
{
	pthread_attr_t ThreadAttributes;
	int err = pthread_attr_init(&ThreadAttributes);
	if ( err )
	{
		printf("pthread attr_init() failed for thread '%s' with err=%d\n", TaskName, err );
		return -10;
	}
#ifdef __COBALT__
	err = pthread_attr_setinheritsched( &ThreadAttributes, PTHREAD_EXPLICIT_SCHED );
	if ( err )
	{
		printf("pthread set explicit sched failed for thread '%s' with err=%d\n", TaskName, err );
		return -11;
	}
#endif
	err = pthread_attr_setdetachstate(&ThreadAttributes, PTHREAD_CREATE_DETACHED /*PTHREAD_CREATE_JOINABLE*/);
	if ( err )
	{
		printf("pthread set detach state failed for thread '%s' with err=%d\n", TaskName, err );
		return -12;
	}
#if defined(__XENO__) || defined(__COBALT__)
	err = pthread_attr_setschedpolicy(&ThreadAttributes, SCHED_FIFO);
	if ( err )
	{
		printf("pthread set scheduling policy failed for thread '%s' with err=%d\n", TaskName, err );
		return -13;
	}
	struct sched_param paramA = { .sched_priority = Priority };
	err = pthread_attr_setschedparam(&ThreadAttributes, &paramA);
	if ( err )
	{
		printf("pthread set priority failed for thread '%s' with err=%d\n", TaskName, err );
		return -14;
	}
#endif
	if ( StackSizeInKo>0 )
	{
		err = pthread_attr_setstacksize(&ThreadAttributes, StackSizeInKo*1024);
		if ( err )
		{
			printf("pthread set stack size failed for thread '%s' with err=%d\n", TaskName, err );
			return -15;
		}
	}

	// calc start time of the periodic thread
	struct timespec start_time;
	if ( clock_gettime( CLOCK_MONOTONIC, &start_time ) )
	{
		printf( "Failed to call clock_gettime\n" );
		return -20;
	}
	/* Start one seconde later from now. */
	start_time.tv_sec += START_DELAY_SECS ;
	
	// if a timerfd is used to make thread periodic (Linux / Xenomai 3),
	// initialize it before launching thread (timer is read in the loop)

	struct itimerspec period_timer_conf;
	TimerFdForThread = timerfd_create(CLOCK_MONOTONIC, 0);
	if ( TimerFdForThread==-1 )
	{
		printf( "Failed to create timerfd for thread '%s'\n", TaskName);
		return -21;
	}
	period_timer_conf.it_value = start_time;
	period_timer_conf.it_interval.tv_sec = 0;
	period_timer_conf.it_interval.tv_nsec = PeriodMicroSecs*1000;
	if ( timerfd_settime(TimerFdForThread, TFD_TIMER_ABSTIME, &period_timer_conf, NULL) )
	{
		printf( "Failed to set periodic tor thread '%s' with errno=%d\n", TaskName, errno);
		return -22;
	}

	
	ThreadRunning = 1;
	err = pthread_create( &MyPosixThread, &ThreadAttributes, (void *(*)(void *))pTaskFunction, (void *)NULL );
	if ( err )
	{
		printf( "Failed to create thread '%s' with err=%d !!!!!\n", TaskName, err );
		return -1;
	}
	else
	{
		// make thread periodic for Xenomai 2 with pthread_make_periodic_np() function.
		pthread_attr_destroy(&ThreadAttributes);
		err = pthread_setname_np( MyPosixThread, TaskName );
		if ( err )
		{
			printf("pthread set name failed for thread '%s', err=%d\n", TaskName, err );
			return -40;
		}
		printf( "Created thread '%s' period=%dusecs ok.\n", TaskName, PeriodMicroSecs );
		return 0;
	}
}

void WaitPeriodicPosixTask( )
{
	int err = 0;
	uint64_t ticks;
	err = read(TimerFdForThread, &ticks, sizeof(ticks));
	if ( err<0 )
	{
		printf( "TimerFd wait period failed for thread with errno=%d\n", errno );
	}
	if ( ticks>1 )
	{
		printf( "TimerFd wait period missed for thread: overruns=%lu\n", (long unsigned int)ticks );
	}
}

// our really simple thread just incrementing a variable in loop !
void * MySimpleTask( void * dummy )
{
	while( ThreadRunning )
	{
		WaitPeriodicPosixTask( );
		ResultIncValue++;
	}
	return 0;
}

int main( int argc, char * argv[] )
{
	int err;
	printf("Simple periodic thread using Posix.\n");
#if defined( __XENO__ ) || defined( __COBALT__ )
	printf("Version compiled for Xenomai 2 or 3.\n");
	mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
	err = CreatePosixTask( "DemoPosix", 1/*Priority*/, 16/*StackSizeInKo*/, PERIOD_MICROSECS/*PeriodMicroSecs*/, MySimpleTask );
	if ( err!=0 )
	{
		printf( "Init task error (%d)!\n",err );
	}
	else
	{
		printf( "Wait 10 seconds before ending...\n" );
		sleep( 10 );
		ThreadRunning = 0;
		printf( "Increment value during 10 secs = %d (should be %d)\n", ResultIncValue, ((10-START_DELAY_SECS)*1000*1000)/PERIOD_MICROSECS );
	}
	return 0;
}