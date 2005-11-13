/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Model.h"
#include "Property.h"
#include "Vector.h"
#include "LogStream.h"
#include "ODESolver.h"
#include "ExplicitEuler.h"
#include "Function.h"
#include "Newton.h"
#include "System.h"

namespace OpenFDM {

System::System(const std::string& name) :
  ModelGroup(name),
  mTime(0)
{
  setTimestepper(new ExplicitEuler);
}

System::~System(void)
{
}

bool
System::init(void)
{
  // Initialize the ModelGroup, sort out models according their dependencies
  // and collects sample time information.
  // If it fails to initialize, the system cannot be initialized.
  if (!ModelGroup::init()) {
    Log(Model, Error) << "Error initializing submodels.\nAborting!" << endl;
    return false;
  }

  // Reset the task scheduling stuff
  mDiscreteTaskList.clear();
  mCurrentTaskNum = 0u;
  mCurrentSliceTime = 0;

  // Compute the basic time slice, that is the greatest time that hits all
  // discrete sample times boundaries we have in this system
  real_type gcd = 0;
  real_type scm = 0;
  real_type minSampleTime = Limits<real_type>::max();
  SampleTimeSet sampleTimes = getSampleTimeSet();
  SampleTimeSet::const_iterator it;
  for (it = sampleTimes.begin(); it != sampleTimes.end(); ++it) {
    if (!it->isDiscrete())
      continue;

    real_type sampleTime = it->getSampleTime();
    OpenFDMAssert(0 < sampleTime);
    minSampleTime = min(minSampleTime, sampleTime);
    if (0 < gcd) {
      gcd = greatestCommonDivisor(gcd, sampleTime);
      scm = scm*sampleTime/greatestCommonDivisor(scm, sampleTime);
    } else {
      gcd = sampleTime;
      scm = sampleTime;
    }
  }

  // Now that we know the basic sample time, build the job schedules
  Log(Model, Info) << "Basic time is: " << gcd << endl;
  if (100*gcd < minSampleTime)
    Log(Model, Warning) << "Basic sample time is less than 100 times smaller "
      "than the smalles submodels sample time" << endl;

  // We do not have any discrete sample time, just do continous scheduling
  if (gcd <= 0)
    return true;

  // Check if we can handle that schedule within the bounds of our data types
  // If it does not, the time slices are choosen too bad for realtime
  // simulations anyway
  real_type stepsPerCycle = floor(scm/gcd + 0.5);
  if (Limits<unsigned>::max() <= stepsPerCycle) {
    Log(Model, Error) << "Too many basic steps for our datatypes.\n"
      "You propably want to use sample times fitting together.\n"
      "Aborting!" << endl;
    return false;
  }
  
  // Alloc enough empty task infos.
  mDiscreteTaskList.assign((unsigned)stepsPerCycle, TaskInfo());
  // Put a TaskInfo entry into the list for all time schedules required
  for (it = sampleTimes.begin(); it != sampleTimes.end(); ++it) {
    if (!it->isDiscrete())
      continue;

    unsigned increment = unsigned(floor(it->getSampleTime()/gcd + 0.5));
    for (unsigned i = 0; i < mDiscreteTaskList.size(); i += increment)
      mDiscreteTaskList[i].addSampleTime(*it);
  }
  // FIXME: combine this step with the one above
  TaskList cTL;
  for (unsigned i = 0; i < mDiscreteTaskList.size(); ++i) {
    if (!mDiscreteTaskList[i].getSampleTimeSet().empty())
      cTL.push_back(mDiscreteTaskList[i]);
    cTL.back().setSliceSize(cTL.back().getSliceSize() + gcd);
    cTL.back().setNumBasicSteps(cTL.back().getNumBasicSteps() + 1);
  }
  mDiscreteTaskList.swap(cTL);

  // Just a verbose print here ...
  Log(Model, Info) << "gcd of sample times is: " << gcd
                   << ", scm of sample times is: " << scm << endl;
  for (unsigned i = 0; i < mDiscreteTaskList.size(); ++i)
    Log(Model, Info) << "Task # " << i << ": # basicSteps "
                     << mDiscreteTaskList[i].getNumBasicSteps()
                     << ", sliceSize "
                     << mDiscreteTaskList[i].getSliceSize()
                     << ", sample times "
                     << mDiscreteTaskList[i].getSampleTimeSet() << endl;

  // At the moment we need a timestepper, else the time does not get
  // incremented
  if (!mTimestepper) {
    Log(Model, Error) << "Timestepping method is unset.\nAborting!" << endl;
    return false;
  }

  mTimestepper->setTime(0);
  mTime = 0;
  mTimestepper->setStepsize(gcd);
  return true;
}

bool
System::simulate(real_type tEnd)
{
  // Check if we need an ODE timestepper.
  bool continousStates = 0 < getNumContinousStates();

  // Since it is possible to change the models states in a non physical way
  // outside this method, we need to read that state and set it into the
  // timestepper. The timestepper needs to take care if it needs to be
  // restarted. So just set it here.
  Vector state(getNumContinousStates());
  getState(state, 0);
  // Exact check is currect here, the user does not have to fiddle with
  // the state during simulation, if the state changes despite of that,
  // Just spend that extra effort.
  if (state != mTimestepper->getState()) {
    mTimestepper->setState(state);
    evalFunction(mTimestepper->getTime(), mTimestepper->getState(), state);
  }

  while (getTime() < tEnd) {
    // This is the maximum time we can step in this loop
    real_type loopTEnd = tEnd;

    // Check if we have discrete systems with any discrete slice time
    if (mDiscreteTaskList.empty()) {
      // Leave it to the timestepping algorithm,
      // to choose the size of the steps
      loopTEnd = tEnd;
    } else {
      // need that  ...
      TaskInfo taskInfo = mDiscreteTaskList[mCurrentTaskNum];
      taskInfo.setTime(getTime());
      
      if (mCurrentSliceTime == 0) {
        Log(Model, Info) << "Computing discrete output for Task # "
                         << mCurrentTaskNum << ": # basicSteps "
                         << taskInfo.getNumBasicSteps() << ", sliceSize "
                         << taskInfo.getSliceSize() << ", sample times "
                         << taskInfo.getSampleTimeSet() << endl;
        
        output(taskInfo);
        update(taskInfo);
      }

      // Take the minimum of the current discrete tasks end and the given
      // end time, take care of roundoff
      real_type taskTEnd = mTime - mCurrentSliceTime + taskInfo.getSliceSize();
      if (equal(taskTEnd, tEnd, 100) || taskTEnd <= tEnd) {
        loopTEnd = taskTEnd;
        mCurrentSliceTime = 0;
        ++mCurrentTaskNum;
        if (mDiscreteTaskList.size() <= mCurrentTaskNum)
          mCurrentTaskNum = 0;
      } else {
        loopTEnd = tEnd;
        mCurrentSliceTime += tEnd - getTime();
      }
    }

    if (!continousStates) {
      mTime = loopTEnd;
    } else {
      // Do the pre integration output round
      Log(Model, Info) << "Preparing Models: pre integration step" << endl;
      TaskInfo taskInfo;
      taskInfo.addSampleTime(SampleTime::Continous);
      taskInfo.addSampleTime(SampleTime::PerTimestep);
      taskInfo.setTime(getTime());
      output(taskInfo);

      Log(Model, Info) << "Integration: from time " << mTimestepper->getTime()
                       << " up to time " << loopTEnd
                       << " dt = " << loopTEnd - mTimestepper->getTime()
                       << endl;
      mTimestepper->integrate(loopTEnd);
      mTime = mTimestepper->getTime();
      // It set's the current state into the models and computes the
      // accelerations for the mechanical system
      Log(Model, Info) << "Integration: finished" << endl;
      evalFunction(mTimestepper->getTime(), mTimestepper->getState(), state);
    }

    if (equal(mTime, tEnd, 10))
      mTime = tEnd;
  }
}

class TrimFunction
  : public Function {
public:
  TrimFunction(System& system, const Vector& state)
    : mSystem(system), mState(state) {}
  virtual unsigned inSize(void) const
  { return mSystem.getNumContinousStates(); }
#if 1
  virtual unsigned outSize(void) const
  { return 2*mSystem.getNumContinousStates(); }
  virtual void eval(real_type t, const Vector& v, Vector& out)
  {
    unsigned nStates = mSystem.getNumContinousStates();
    Vector deriv(nStates);
    mSystem.evalFunction(t, v, deriv);

    out.resize(2*nStates);
    out(Range(1, nStates)) = 1e-1*(mState - v);
    out(Range(nStates + 1, 2*nStates)) = deriv;
  }
#else
  virtual unsigned outSize(void) const
  { return mSystem.getNumContinousStates(); }
  virtual void eval(real_type t, const Vector& v, Vector& out)
  {
    mSystem.evalFunction(t, v, out);
  }
#endif
private:
  System& mSystem;
  Vector mState;
};

bool
System::trim(void)
{
  // need to prepare the System especially for the per step tasks
  TaskInfo taskInfo;
  taskInfo.addSampleTime(SampleTime::Continous);
  taskInfo.addSampleTime(SampleTime::PerTimestep);
  output(taskInfo);

  // Get the current state
  Vector state(getNumContinousStates());
  getState(state, 0);

  Vector trimState = state;
  // Buld up the trim function
  TrimFunction trimFunction(*this, trimState);

  // Try to find a minimum
  real_type atol = 1e-3;
  real_type rtol = 1e-8;
  bool ret = GaussNewton(trimFunction, getTime(), trimState, atol, rtol);
  if (ret) {
    setState(trimState, 0);
  } else {
    setState(state, 0);
  }

  return ret;
}

void
System::setTimestepper(ODESolver* timestepper)
{
  real_type t = 0;
  if (mTimestepper) {
    mTimestepper->setModel(0);
    t = mTimestepper->getTime();
  }
  mTimestepper = timestepper;
  if (mTimestepper) {
    mTimestepper->setModel(this);
    mTimestepper->setTime(t);
  }
}

} // namespace OpenFDM
