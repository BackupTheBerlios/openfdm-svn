/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Time_H
#define OpenFDM_Time_H

#include <ctime>
#include <sstream>
#include <istream>
#include <ostream>
#include <iomanip>
#include "Math.h"

// FIXME move into own tools library.

namespace OpenFDM {

class Time {
  // Just to avoid typos with that magnitude of zeros
  enum { NanoSecondsPerSecond = 1000000000 };

public:
  enum Type {
    RealTime,
    MonotonicTime,
    CPUTime
  };

  typedef std::time_t sec_type;
  typedef long nsec_type;

  Time()
  { setTimeUnchecked(0, 0); }
  Time(const sec_type& sec, const nsec_type& nsec)
  { setTime(sec, nsec); }
  Time(const sec_type& sec)
  { setTimeUnchecked(sec, 0); }
  Time(const double& sec)
  { setTime(sec); }

  static Time now(Type type = RealTime);
  static Time resolution(Type type = RealTime);

  operator double() const
  { return (1.0/NanoSecondsPerSecond)*_nsec + _sec; }

  void setTime(const double& seconds)
  {
    sec_type wholeSecs = sec_type(floor(seconds));
    nsec_type reminder;
    reminder = nsec_type(floor((seconds - wholeSecs)*NanoSecondsPerSecond));
    setTimeUnchecked(wholeSecs, reminder);
  }
  void setTime(const sec_type& sec, const nsec_type& nsec)
  {
    if (nsec < 0) {
      nsec_type carry = nsec/NanoSecondsPerSecond - 1;
      _nsec = nsec - NanoSecondsPerSecond*carry;
      _sec = sec + carry;
    } else {
      nsec_type carry = nsec/NanoSecondsPerSecond;
      _nsec = nsec - NanoSecondsPerSecond*carry;
      _sec = sec + carry;
    }
  }
  const sec_type& getSeconds() const
  { return _sec; }
  const nsec_type& getNanoSeconds() const
  { return _nsec; }

  Time& operator+=(const Time& c)
  { setTime(_sec + c._sec, _nsec + c._nsec); return *this; }
  Time& operator-=(const Time& c)
  { setTime(_sec - c._sec, _nsec - c._nsec); return *this; }

private:
  void setTimeUnchecked(const sec_type& sec, const nsec_type& nsec)
  { _sec = sec; _nsec = nsec; }

  sec_type _sec;
  nsec_type _nsec;

  friend bool operator==(const Time& c1, const Time& c2);
  friend bool operator<(const Time& c1, const Time& c2);
};

inline bool
operator==(const Time& c1, const Time& c2)
{ return c1._nsec == c2._nsec && c1._sec == c2._sec; }

inline bool
operator!=(const Time& c1, const Time& c2)
{ return !operator==(c1, c2); }

inline bool
operator<(const Time& c1, const Time& c2)
{
  if (c1._sec < c2._sec)
    return true;
  else
    return c1._sec == c2._sec && c1._nsec < c2._nsec;
}

inline bool
operator>(const Time& c1, const Time& c2)
{ return c2 < c1; }

inline bool
operator>=(const Time& c1, const Time& c2)
{ return !(c1 < c2); }

inline bool
operator<=(const Time& c1, const Time& c2)
{ return !(c1 > c2); }

inline Time
operator+(const Time& c1)
{ return c1; }

inline Time
operator-(const Time& c1)
{ return Time(0, 0) -= c1; }

inline Time
operator+(const Time& c1, const Time& c2)
{ return Time(c1) += c2; }

inline Time
operator-(const Time& c1, const Time& c2)
{ return Time(c1) -= c2; }

template<typename char_type, typename traits_type>
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const Time& t)
{
  if (0 <= t.getSeconds()) {
    std::basic_stringstream<char_type, traits_type> stream;
    stream << t.getSeconds() << stream.widen('.')
           << std::setw(9) << std::right << std::setfill('0')
           << t.getNanoSeconds();
    return os << stream.str();
  } else {
    return os << os.widen('-') << -t;
  }
}

class TimeCounter {
public:
  TimeCounter(Time::Type type = Time::CPUTime) :
    mType(type),
    mTime(0, 0),
    mRunCounter(0)
  { }
  
  Time getTime() const
  {
    if (0 < mRunCounter)
      return Time::now(mType) + mTime;
    else
      return mTime;
  }
  
  void start()
  {
    if (1 != ++mRunCounter)
      return;
    mTime -= Time::now(mType);
  }
  
  void stop()
  {
    if (--mRunCounter != 0)
      return;
    mTime += Time::now(mType);
  }
  
private:
  Time::Type mType;
  Time mTime;
  int mRunCounter;
};

class ScopeTimeCounter {
public:
  ScopeTimeCounter(TimeCounter& timer) : mTimeCounter(timer)
  { mTimeCounter.start(); }
  ~ScopeTimeCounter(void)
  { mTimeCounter.stop(); }

private:
  ScopeTimeCounter(void);
  ScopeTimeCounter(const ScopeTimeCounter&);
  ScopeTimeCounter& operator=(const ScopeTimeCounter&);

  TimeCounter& mTimeCounter;
};

class ScopeUnTimeCounter {
public:
  ScopeUnTimeCounter(TimeCounter& timer) : mTimeCounter(timer)
  { mTimeCounter.stop(); }
  ~ScopeUnTimeCounter(void)
  { mTimeCounter.start(); }

private:
  ScopeUnTimeCounter(void);
  ScopeUnTimeCounter(const ScopeUnTimeCounter&);
  ScopeUnTimeCounter& operator=(const ScopeUnTimeCounter&);

  TimeCounter& mTimeCounter;
};

}

#endif
