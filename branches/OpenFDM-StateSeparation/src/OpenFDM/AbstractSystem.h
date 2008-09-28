/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractSystem_H
#define OpenFDM_AbstractSystem_H

#include "Interval.h"
#include "Referenced.h"
#include "Types.h"

namespace OpenFDM {

class AbstractSystem : public Referenced {
public:
  AbstractSystem() :
    mValidityInterval(TimeInterval::nothing()),
    mTime(Limits<real_type>::min_value())
  { }
  virtual ~AbstractSystem() {}

  const TimeInterval& getValidityInterval() const
  { return mValidityInterval; }
  const real_type& getTime() const
  { return mTime; }

  void outputAt(const real_type& t)
  {
    // Cry if we cannot do anything!
    OpenFDMAssert(!getValidityInterval().empty());
    // update until our requested end time is in the current interval.
    while (needUpdate(t))
      update(t);
    if (t != mTime)
      output(t);
  }

  // Not sure yet who is responsible for calling them at the right times ...
  void init(const real_type& t)
  {
    mValidityInterval = TimeInterval(t);
    mTime = t;
    initImplementation(t);
    Log(Schedule,Info) << "Initialized for time Interval from t = "
                       << mValidityInterval.getBegin() << " to t = "
                       << mValidityInterval.getEnd() << std::endl;
  }

  void update(const real_type& tEndHint)
  {
    updateImplementation(tEndHint);
    Log(Schedule,Info) << "Updated to time Interval from t = "
                       << mValidityInterval.getBegin() << " to t = "
                       << mValidityInterval.getEnd() << std::endl;
  }
  void output(const real_type& t)
  {
    OpenFDMAssert(getValidityInterval().contains(t));
    mTime = t;
    outputImplementation(mTime);
    Log(Schedule,Info) << "Output for time t =  " << t << std::endl;
  }
protected:

  void setValidityInterval(const TimeInterval& validityInterval)
  { mValidityInterval = validityInterval; }

  virtual void initImplementation(const real_type& t) = 0;

  bool needUpdate(const real_type& t) const
  { return getValidityInterval().isStrictlyLeftOf(t); }
  virtual void updateImplementation(const real_type& tEndHint) = 0;

  virtual void outputImplementation(const real_type& t) = 0;

private:
  TimeInterval mValidityInterval;
  real_type mTime;
};

} // namespace OpenFDM

#endif
