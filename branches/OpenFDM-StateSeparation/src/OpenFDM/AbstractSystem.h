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
    mTime(Limits<real_type>::quiet_NaN())
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

// FIXME: dump them here for now. Will be required later ...
class EnabledSystem : public AbstractSystem {
};

class GroupedSystem : public AbstractSystem {
public:
  unsigned getNumChildren() const
  { return mAbstractSystemList.size(); }
  AbstractSystem* getChild(unsigned index)
  {
    if (mAbstractSystemList.size() <= index)
      return 0;
    return mAbstractSystemList[index];
  }
  unsigned addChild(AbstractSystem* abstractSystem)
  {
    unsigned index = mAbstractSystemList.size();
    mAbstractSystemList.push_back(abstractSystem);
    return index;
  }
  void removeChild(AbstractSystem* abstractSystem)
  {
    AbstractSystemList::iterator i;
    i = std::find(mAbstractSystemList.begin(), mAbstractSystemList.end(),
                  abstractSystem);
    if (i == mAbstractSystemList.end())
      return;
    mAbstractSystemList.erase(i);
  }

protected:
  virtual void initImplementation(const real_type& t)
  {
    AbstractSystemList::const_iterator i;
    for (i = mAbstractSystemList.begin(); i != mAbstractSystemList.end(); ++i) {
      (*i)->init(t);
    }
  }
  virtual void updateImplementation(const real_type& tEndHint)
  {
    // initially set to all
    TimeInterval validityInterval = TimeInterval::all();
    AbstractSystemList::const_iterator i;
    for (i = mAbstractSystemList.begin(); i != mAbstractSystemList.end(); ++i) {
      (*i)->update(tEndHint);
      if (validityInterval.getBegin() < (*i)->getValidityInterval().getBegin())
        validityInterval.setBegin((*i)->getValidityInterval().getBegin());
      if ((*i)->getValidityInterval().getEnd() < validityInterval.getEnd())
        validityInterval.setEnd((*i)->getValidityInterval().getEnd());
    }
    setValidityInterval(validityInterval);
  }
  virtual void outputImplementation(const real_type& t)
  {
    AbstractSystemList::const_iterator i;
    for (i = mAbstractSystemList.begin(); i != mAbstractSystemList.end(); ++i)
      (*i)->output(t);
  }
private:
  typedef std::vector<SharedPtr<AbstractSystem> > AbstractSystemList;
  AbstractSystemList mAbstractSystemList;
};


} // namespace OpenFDM

#endif
