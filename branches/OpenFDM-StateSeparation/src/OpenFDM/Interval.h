/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Interval_H
#define OpenFDM_Interval_H

#include "Limits.h"

namespace OpenFDM {

template<typename T>
class Interval {
public:
  typedef T value_type;

  Interval() :
    mBegin(Limits<value_type>::min_value()),
    mEnd(Limits<value_type>::max_value())
  { }
  Interval(const value_type& value) :
    mBegin(value), mEnd(value)
  { }
  Interval(const value_type& begin, const value_type& end) :
    mBegin(begin), mEnd(end)
  { }

  void setBegin(const value_type& begin) { mBegin = begin; }
  const value_type& getBegin() const { return mBegin; }

  void setEnd(const value_type& end) { mBegin = end; }
  const value_type& getEnd() const { return mEnd; }

  bool empty() const
  { return mEnd < mBegin; }
  value_type getLength() const
  { return mEnd - mBegin; }

  bool isLeftOf(const value_type& value) const
  { return mEnd <= value; }
  bool isStrictlyLeftOf(const value_type& value) const
  { return mEnd < value; }
  bool isRightOf(const value_type& value) const
  { return value <= mBegin; }
  bool isStrictlyRightOf(const value_type& value) const
  { return value < mBegin; }
  bool contains(const value_type& value) const
  { return mBegin <= value && value <= mEnd; }

  static Interval all()
  { return Interval(); }
  static Interval noting()
  { return Interval(1, -1); }

private:
  value_type mBegin;
  value_type mEnd;
};

} // namespace OpenFDM

#endif
