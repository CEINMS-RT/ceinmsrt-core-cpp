#ifndef GETTIME_H
#define GETTIME_H
#ifdef WIN32
#include <windows.h>
#endif

#ifdef __unix__
#include <sys/time.h>
#endif

namespace rtb {

    inline double getTime()
    {
        double timeNow;
#ifdef __unix__
        struct timeval now;

        gettimeofday(&now, NULL);

        timeNow = (now.tv_sec) + 0.000001 * now.tv_usec;
#endif
#ifdef WIN32
        /*static const unsigned long long EPOCH = ((unsigned long long)116444736000000000ULL);

        SYSTEMTIME  system_time;
        FILETIME    file_time;
        unsigned long long    time;
        GetSystemTime(&system_time);
        SystemTimeToFileTime(&system_time, &file_time);
        time = ((unsigned long long)file_time.dwLowDateTime);
        time += ((unsigned long long)file_time.dwHighDateTime) << 32;

        long tv_sec = (long)((time - EPOCH) / 10000000L);
        long tv_usec = (long)(system_time.wMilliseconds * 1000);

        timeNow = (tv_sec)+0.000001 * tv_usec;*/

        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        unsigned long long tt = ft.dwHighDateTime;
        tt <<= 32;
        tt |= ft.dwLowDateTime;
        tt /= 10;
        tt -= 11644473600000000ULL;
        timeNow = (double)tt / 1000000.0;
#endif
        return timeNow;
    }
}
#endif
