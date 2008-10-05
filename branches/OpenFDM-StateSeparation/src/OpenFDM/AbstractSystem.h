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

//     if (equal(getTime(), getValidityInterval().getEnd())
//     continousUpdate(tEnd);

    // update until our requested end time is in the current interval.
    while (needUpdate(t)) {
      discreteUpdate();

      real_type tEnd = min(getValidityInterval().getEnd(), t);
      continousUpdate(tEnd);
    }
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

  void discreteUpdate()
  {
    discreteUpdateImplementation();
    Log(Schedule,Info) << "Updated to time Interval from t = "
                       << mValidityInterval.getBegin() << " to t = "
                       << mValidityInterval.getEnd() << std::endl;
  }
  void continousUpdate(const real_type& tEnd)
  {
    continousUpdateImplementation(tEnd);
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

  /// FIXME: make that non virtual, but keep that up to date in the
  /// implementation.
  /// Hmm that interval concept is better????
//   virtual real_type getNextDiscreteSampleHit() const
//   {
//     // Hmm, is this not enough for the sample hits??
//     return mValidityInterval.getEnd();
//   }

protected:

  void setValidityInterval(const TimeInterval& validityInterval)
  { mValidityInterval = validityInterval; }

  virtual void initImplementation(const real_type& t) = 0;

  bool needUpdate(const real_type& t) const
  { return getValidityInterval().isStrictlyLeftOf(t); }
  virtual void discreteUpdateImplementation() = 0;
  virtual void continousUpdateImplementation(const real_type& tEnd) = 0;

  virtual void outputImplementation(const real_type& t) = 0;

private:
  TimeInterval mValidityInterval;
  real_type mTime;
};

class EnabledSystem : public AbstractSystem {
public:
  enum EnableMode {
    /// If disabled, the models output/state is just held.
    /// On reenable, the model just continues to work
    Hold,
    /// If disabled, the models output/state is just held.
    /// On reenable, the model is initialized
    HoldReset,
    /// If disabled, the models output/state is initialized
    ResetHold
  };

  EnableMode getEnableMode(void) const
  { return mEnableMode; }
  void setEnableMode(EnableMode enableMode)
  { mEnableMode = enableMode; }

  void setSystem(AbstractSystem* system)
  { mAbstractSystem = system; }
  AbstractSystem* getSystem() const
  { return mAbstractSystem; }

  void setPortValue(NumericPortValue* portValue)
  { mEnablePort = portValue; }
  NumericPortValue* getPortValue() const
  { return mEnablePort; }

protected:
  virtual void initImplementation(const real_type& t)
  {
    mEnabledState = getEnableInput();
    mAbstractSystem->init(t);
  }
  virtual void discreteUpdateImplementation()
  {
    if (getEnableInput()) {
      if (mEnabledState) {
      } else {
        if (mEnableMode == HoldReset) {
          mAbstractSystem->init(getTime());
        }       
      }
      mEnabledState = true;
      mAbstractSystem->discreteUpdate();
    } else {
      if (mEnabledState) {
        if (mEnableMode == ResetHold) {
          mAbstractSystem->init(getTime());
        }
      } else {
      }
      mEnabledState = false;
    }
  }
  virtual void continousUpdateImplementation(const real_type& tEnd)
  {
    if (!mEnabledState)
      return;
    mAbstractSystem->continousUpdate(tEnd);
  }
  virtual void outputImplementation(const real_type& t)
  {
    if (!mEnabledState)
      return;
    mAbstractSystem->output(t);
  }

  bool getEnableInput() const
  { return 0.5 < abs(mEnablePort->getValue()(0, 0)); }

private:
  SharedPtr<AbstractSystem> mAbstractSystem;

  SharedPtr<NumericPortValue> mEnablePort;
  bool mEnabledState;
  EnableMode mEnableMode;
};

class GroupedSystem : public AbstractSystem {
public:
  // FIXME: that 'nextSampleTime can be computed during the discreteUpdate
  // step ...

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
  virtual void continousUpdateImplementation(const real_type& tEnd)
  {
//     // initially set to all
//     TimeInterval validityInterval = TimeInterval::all();
//     AbstractSystemList::const_iterator i;
//     for (i = mAbstractSystemList.begin(); i != mAbstractSystemList.end(); ++i) {
//       (*i)->update(tEndHint);
//       if (validityInterval.getBegin() < (*i)->getValidityInterval().getBegin())
//         validityInterval.setBegin((*i)->getValidityInterval().getBegin());
//       if ((*i)->getValidityInterval().getEnd() < validityInterval.getEnd())
//         validityInterval.setEnd((*i)->getValidityInterval().getEnd());
//     }
//     setValidityInterval(validityInterval);
  }
  virtual void discreteUpdateImplementation()
  {
    // initially set to all
//     AbstractSystemList::const_iterator i;
//     for (i = mAbstractSystemList.begin(); i != mAbstractSystemList.end(); ++i) {
//       (*i)->update();
//     }
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
