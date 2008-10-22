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

class ContinousTask : public Task {
public:
  class SystemStateValueVector {
  public:
    SystemStateValueVector() : mNumStates(0) {}
    
    void clear()
    {
      mNumStates = 0;
      mStateValues.clear();
    }

    LinAlg::size_type getNumStates() const
    { return mNumStates; }
    
    void push_back(ContinousStateValue* stateValue)
    {
      OpenFDMAssert(stateValue);
      mNumStates += stateValue->getNumStates();
      mStateValues.push_back(stateValue);
    }
    
    void setStateValue(const Vector& v)
    {
      // FIXME: too much copies
      StateStream stateStream(v);
      for (unsigned i = 0; i < mStateValues.size(); ++i)
        mStateValues[i]->setValue(stateStream);
      OpenFDMAssert(stateStream.isAtEnd());
    }
    
    void getStateValue(Vector& v) const
    {
      // FIXME: too much copies
      StateStream stateStream(getNumStates());
      for (unsigned i = 0; i < mStateValues.size(); ++i)
        mStateValues[i]->getValue(stateStream);
      OpenFDMAssert(stateStream.isAtEnd());
      v = stateStream.getState();
    }
    
  private:
    LinAlg::size_type mNumStates;
    std::vector<SharedPtr<ContinousStateValue> > mStateValues;
  };
  
  void output(const real_type& t)
  {
    setTime(t);
    // The model outputs before mechanical state propagation
    mModelContextList[0].output(*this);
    // Now the mechanical state propagation
    mMechanicContextList.velocities(*this);
    // The model outputs before mechanical force propagation
    mModelContextList[1].output(*this);
    // Now the mechanical force propagation
    mMechanicContextList.articulation(*this);
    // The model outputs before mechanical acceleration propagation
    mModelContextList[2].output(*this);
    // Now the mechanical acceleration propagation
    mMechanicContextList.accelerations(*this);
    // The output stage happens in a mechanic system - if it does -
    // past the mechanic system has completed.
//     mMechanicContextList.output(*this);
    // The model outputs past mechanical acceleration propagation
    mModelContextList[3].output(*this);
  }

  void derivative()
  {
    for (unsigned i = 0; i < 4; ++i)
      mModelContextList[i].derivative(*this);
    mMechanicContextList.derivative(*this);
  }

  LinAlg::size_type getNumStates() const
  { return mStateValues.getNumStates(); }

  void setStateValue(const Vector& v)
  { mStateValues.setStateValue(v); }
  void getStateValue(Vector& v) const
  { mStateValues.getStateValue(v); }
  void getStateDerivative(Vector& v) const
  { mDerivativeValues.getStateValue(v); }

  void appendStateValuesFromLeafContext(LeafContext& leafContext)
  {
    unsigned numContStates = leafContext.getNode().getNumContinousStateValues();
    for (unsigned k = 0; k < numContStates; ++k) {
      ContinousStateInfo* continousStateInfo;
      continousStateInfo = const_cast<ContinousStateInfo*>(leafContext.getNode().getContinousStateInfo(k));
      mStateValues.push_back(leafContext.mContinousState.getValue(*continousStateInfo));
      mDerivativeValues.push_back(leafContext.mContinousStateDerivative.getValue(*continousStateInfo));
    }
  }

  ModelContextList mModelContextList[4];
  MechanicContextList mMechanicContextList;
  SystemStateValueVector mStateValues;
  SystemStateValueVector mDerivativeValues;
};

} // namespace OpenFDM

#endif
