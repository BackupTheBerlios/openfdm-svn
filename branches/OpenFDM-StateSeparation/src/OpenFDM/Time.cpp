/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Time.h"

#include <time.h>
#include <cerrno>
#include <cstdio>
#include <cmath>

namespace OpenFDM {

#define HAVE_POSIX_TIMERS 
#define HAVE_CLOCK_MONOTONIC
#define HAVE_CLOCK_THREAD_CPUTIME

#if defined(HAVE_POSIX_TIMERS)
static clockid_t
typeToClockId(Time::Type type)
{
  switch (type) {
#ifdef HAVE_CLOCK_MONOTONIC
  case Time::MonotonicTime:
    return CLOCK_MONOTONIC;
#endif
#ifdef HAVE_CLOCK_THREAD_CPUTIME
  case Time::CPUTime:
    return CLOCK_THREAD_CPUTIME_ID;
#endif
  default:
    return CLOCK_REALTIME;
  }
}
#endif

Time
Time::now(Type type)
{
#if defined(HAVE_POSIX_TIMERS)
  struct timespec ts;
  if (0 != clock_gettime(typeToClockId(type), &ts))
    perror("Error in clock_gettime: ");
  Time t;
  t.setTimeUnchecked(ts.tv_sec, ts.tv_nsec);
  return t;
#else
  if (type == CPUTime) {
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return Time(ru.ru_utime.tv_sec, 1000*ru.ru_utime.tv_usec);
  } else {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return Time(tv.tv_sec, 1000*tv.tv_usec);
  }
#endif
}

Time
Time::resolution(Type type)
{
#if defined(HAVE_POSIX_TIMERS)
  struct timespec ts;
  if (0 != clock_getres(typeToClockId(type), &ts))
    perror("Error in clock_getres: ");
  Time t;
  t.setTimeUnchecked(ts.tv_sec, ts.tv_nsec);
  return t;
#else
  // Cannot see something better
  return Time(0, 1000);
#endif
}

}
