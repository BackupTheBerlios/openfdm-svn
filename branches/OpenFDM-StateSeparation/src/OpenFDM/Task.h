/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Task_H
#define OpenFDM_Task_H

#include "Referenced.h"
#include "MechanicContext.h"
#include "ModelContext.h"

namespace OpenFDM {

class Task : public Referenced {
public:
  virtual ~Task() {}

  const real_type& getTime() const
  { return mTime; }

  /// Hmm, there should be some exec method,
  /// that can be used to do real multitasking here ...

protected:
  void setTime(const real_type& time)
  { mTime = time; }

private:
  real_type mTime;
};

class InitTask : public Task {
public:
  void init(const real_type& t)
  {
    setTime(t);
    // The model outputs before mechanical state propagation
    mModelContextList[0].init(*this);
    // Now the mechanical state propagation
    mMechanicContextList.init(*this);
    mMechanicContextList.velocities(*this);
    // The model outputs before mechanical force propagation
    mModelContextList[1].init(*this);
    // Now the mechanical force propagation
    mMechanicContextList.articulation(*this);
    // The model outputs before mechanical acceleration propagation
    mModelContextList[2].init(*this);
    // Now the mechanical acceleration propagation
    mMechanicContextList.accelerations(*this);
    // The model outputs past mechanical acceleration propagation
    mModelContextList[3].init(*this);
  }

  ModelContextList mModelContextList[4];
  MechanicContextList mMechanicContextList;
};

class DiscreteTask : public Task {
public:
  DiscreteTask(const real_type& stepsize) : mStepsize(stepsize) { }
  virtual ~DiscreteTask() {}

  const real_type& getStepsize() const
  { return mStepsize; }

  void output(const real_type& t)
  {
    setTime(t);
    mModelContextList.output(*this);
    // ?????
//     mMechanicContextList.output(*this);
  }
  void update(const real_type& startTime)
  {
    OpenFDMAssert(equal(startTime, getTime(), 100));
    mModelContextList.update(*this);
    mMechanicContextList.update(*this);
  }

  ModelContextList mModelContextList;
  MechanicContextList mMechanicContextList;

private:
  const real_type mStepsize;
};

} // namespace OpenFDM

#endif
