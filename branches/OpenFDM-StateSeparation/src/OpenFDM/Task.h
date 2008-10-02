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

// protected:
  void setTime(const real_type& time)
  { mTime = time; }

private:
  real_type mTime;
};

class DiscreteTask : public Task {
public:
  DiscreteTask(const real_type& stepsize) : mStepsize(stepsize) { }
  virtual ~DiscreteTask() {}

  const real_type& getStepsize() const
  { return mStepsize; }

  void output()
  {
    mModelContextList.output(*this);
    mMechanicContextList.init(*this);
  }
  void update()
  {
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
