/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Model.h"
#include "Property.h"
#include "Vector.h"
#include "Environment.h"
#include "LogStream.h"
#include "ODESolver.h"
#include "ExplicitEuler.h"
#include "Function.h"
#include "Newton.h"
#include "ModelVisitor.h"
#include "System.h"

namespace OpenFDM {

System::System(const std::string& name) :
  ModelGroup(name),
  mTime(0)
{
  setTimestepper(new ExplicitEuler);
  mEnvironment = new Environment;
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
    Log(Schedule, Error) << "Error initializing submodels.\nAborting!" << endl;
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
  Log(Schedule, Info) << "Basic time is: " << gcd << endl;
  if (100*gcd < minSampleTime)
    Log(Schedule, Warning) << "Basic sample time is less than 100 times "
      "smaller than the smallest submodels sample time" << endl;

  // We do not have any discrete sample time, just do continous scheduling
  if (gcd <= 0)
    return true;

  // Check if we can handle that schedule within the bounds of our data types
  // If it does not, the time slices are choosen too bad for realtime
  // simulations anyway
  real_type stepsPerCycle = floor(scm/gcd + 0.5);
  if (Limits<unsigned>::max() <= stepsPerCycle) {
    Log(Schedule, Error) << "Too many basic steps for our datatypes.\n"
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
  Log(Schedule, Info) << "gcd of sample times is: " << gcd
                   << ", scm of sample times is: " << scm << endl;
  for (unsigned i = 0; i < mDiscreteTaskList.size(); ++i)
    Log(Schedule, Info) << "Task # " << i << ": # basicSteps "
                     << mDiscreteTaskList[i].getNumBasicSteps()
                     << ", sliceSize "
                     << mDiscreteTaskList[i].getSliceSize()
                     << ", sample times "
                     << mDiscreteTaskList[i].getSampleTimeSet() << endl;

  // At the moment we need a timestepper, else the time does not get
  // incremented
  if (!mTimestepper) {
    Log(Schedule, Error) << "Timestepping method is unset.\nAborting!" << endl;
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
  StateStream stateStream(getNumContinousStates());
  getState(stateStream);
  Vector state = stateStream.getState();
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
        Log(Schedule, Info) << "Computing discrete output for Task # "
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
      Log(Schedule, Info) << "Preparing Models: pre integration step" << endl;
      TaskInfo taskInfo;
      taskInfo.addSampleTime(SampleTime::Continous);
      taskInfo.addSampleTime(SampleTime::PerTimestep);
      taskInfo.setTime(getTime());
      output(taskInfo);

      Log(Schedule, Info) << "Integration: from time "
                          << mTimestepper->getTime()
                          << " up to time " << loopTEnd
                          << " dt = " << loopTEnd - mTimestepper->getTime()
                          << endl;
      // FIXME: check for errors
      mTimestepper->integrate(loopTEnd);
      mTime = mTimestepper->getTime();
      Log(Schedule, Info) << "Integration: finished" << endl;
      // Croase end check when it is too late, we might do stiffness
      // detection at least within dopri in an other way ...
      if (!isFinite(mTimestepper->getState())) {
        Log(TimeStep, Warning) << "Found infinite values in continous state "
          "vector. Consider using an other timestepping method or make your "
          "model less stiff. Aborting!" << endl;
        return false;
      }

      // It set's the current state into the models and computes the
      // accelerations for the mechanical system
      evalFunction(mTimestepper->getTime(), mTimestepper->getState(), state);
    }

    if (equal(mTime, tEnd, 10))
      mTime = tEnd;
  }

  return true;
}

class TrimCollectorVisitor :
    public ModelVisitor {
public:
  TrimCollectorVisitor(unsigned nStates) :
    mStateStream(nStates)
  { }
  virtual ~TrimCollectorVisitor(void)
  { }
  virtual void apply(Model& model)
  { model.getStateDeriv(mStateStream); }
  virtual void apply(ModelGroup& modelGroup)
  { traverse(modelGroup); }
  virtual void apply(MobileRootJoint& mobileRootJoint)
  {
    mGeodPos = mobileRootJoint.getGeodPosition();
    mGeodOr = mobileRootJoint.getGeodOrientation();
    mVel = mobileRootJoint.getRelVel();
    mVelDot = mobileRootJoint.getRelVelDot();
    mPosDot = mobileRootJoint.getPosDot();
    mQDot = mobileRootJoint.getQDot();
    mMobileRootJoint = &mobileRootJoint;
  }
  Geodetic mGeodPos;
  Quaternion mGeodOr;
  Vector6 mVel;
  Vector6 mVelDot;
  Vector3 mPosDot;
  Vector4 mQDot;
  StateStream mStateStream;
  MobileRootJoint* mMobileRootJoint;
};

class TrimFunction :
    public Function {
public:
  TrimFunction(System& system) :
    mSystem(system)
  {
    unsigned nStates = mSystem.getNumContinousStates();
    TrimCollectorVisitor tcv(nStates);
    mSystem.accept(tcv);
    mGeodPos = tcv.mGeodPos;
    mGeodOr = tcv.mGeodOr;
    mVel = tcv.mVel;
    mVelDot = tcv.mVelDot;
    mPosDot = tcv.mPosDot;
    mQDot = tcv.mQDot;
  }
  virtual unsigned inSize(void) const
  { return mSystem.getNumContinousStates(); }
  virtual unsigned outSize(void) const
  { return mSystem.getNumContinousStates(); }
  virtual void eval(real_type t, const Vector& v, Vector& out)
  {
    unsigned nStates = mSystem.getNumContinousStates();
    Vector deriv(nStates);
    mSystem.evalFunction(t, v, deriv);

    TrimCollectorVisitor tcv(nStates);
    mSystem.accept(tcv);
    Vector3 eo = mGeodOr.getEuler();
    Vector3 en = tcv.mGeodOr.getEuler();

    /// 3 dof for the position
    real_type tmp = 1e6*(mGeodPos.longitude - tcv.mGeodPos.longitude);
    tcv.mStateStream.writeSubState(tmp);
    tmp = 1e6*(mGeodPos.latitude - tcv.mGeodPos.latitude);
    tcv.mStateStream.writeSubState(tmp);
    tmp = smoothDeadBand(mGeodPos.altitude - tcv.mGeodPos.altitude, 10.0);
    tcv.mStateStream.writeSubState(tmp);

    // The orientation
    tmp = 1e2*smoothDeadBand(eo(1) - en(1), 20*deg2rad);
    tcv.mStateStream.writeSubState(tmp);
    tmp = 1e2*smoothDeadBand(eo(2) - en(2), 20*deg2rad);
    tcv.mStateStream.writeSubState(tmp);
    tmp = 1e2*(eo(3) - en(3));
    tcv.mStateStream.writeSubState(tmp);

    tcv.mStateStream.writeSubState(1e2*norm(tcv.mQDot) + norm(tcv.mPosDot - mPosDot));
//     tcv.mStateStream.writeSubState(1e2*tcv.mQDot);
//     tcv.mStateStream.writeSubState(tcv.mPosDot);
    tcv.mStateStream.writeSubState(1e6*tcv.mVelDot);

    out = tcv.mStateStream.getState();
  }
private:
  System& mSystem;
  Geodetic mGeodPos;
  Quaternion mGeodOr;
  Vector6 mVel;
  Vector6 mVelDot;
  Vector3 mPosDot;
  Vector4 mQDot;
};

class AltitudeFinderTrimFunction :
    public Function {
public:
  AltitudeFinderTrimFunction(System& system, real_type range) :
    mSystem(system),
    mRange(range)
  {
    unsigned nStates = mSystem.getNumContinousStates();
    TrimCollectorVisitor tcv(nStates);
    mSystem.accept(tcv);
    mGeodPos = tcv.mGeodPos;
    mGeodOr = tcv.mGeodOr;
    mVel = tcv.mVel;
    mVelDot = tcv.mVelDot;
    mMobileRootJoint = tcv.mMobileRootJoint;
  }
  virtual unsigned inSize(void) const
  { return 1; }
  virtual unsigned outSize(void) const
  { return 7; }
  virtual void eval(real_type t, const Vector& v, Vector& out)
  {
    Geodetic geod = mGeodPos;
    geod.altitude = mGeodPos.altitude - mRange*0.5 + v(1);
    mMobileRootJoint->setGeodPosition(geod);
    
    unsigned nStates = mSystem.getNumContinousStates();
    StateStream vv(nStates);
    mSystem.getState(vv);
    Vector deriv(nStates);
    mSystem.evalFunction(t, vv.getState(), deriv);

    TrimCollectorVisitor tcv(nStates + 6);
    mSystem.accept(tcv);

    /// The line search algorithm sees that the gravitation
    /// is less in higher regions, thus we need to add a 'minimum altitude'
    /// criterion
    out.resize(7, 1);
    out(Range(1, 6)) = tcv.mVelDot;
//     out(7) = 1e-1*v(1);
    out(7) = smoothDeadBand(v(1), mRange);
    
//     Log(Model,Error) << trans(v) << endl;
//     Log(Model,Error) << trans(out) << endl;
//     Log(Model,Error) << mGeodPos << geod << endl << endl;
  }
private:
  System& mSystem;
  real_type mRange;
  Geodetic mGeodPos;
  Quaternion mGeodOr;
  Vector6 mVel;
  Vector6 mVelDot;
  MobileRootJoint* mMobileRootJoint;
};

bool
System::trim(void)
{
  // need to prepare the System especially for the per step tasks
  TaskInfo taskInfo = mDiscreteTaskList[0];
//   taskInfo.setTime(getTime());
  taskInfo.addSampleTime(SampleTime::Continous);
  taskInfo.addSampleTime(SampleTime::PerTimestep);
  output(taskInfo);

  /// First try to find an altitude where the acceleration is minimal,
  /// this is most likely a good starting point for the subsequent total trim
  real_type range = 20;
  AltitudeFinderTrimFunction altTrim(*this, range);

  Vector altV(1);
  altV(1) = 0;
  Vector dk(1);
  dk(1) = 1;
  Vector res = LineSearch(altTrim, getTime(), altV, dk, range, 1e-3);
  altTrim.eval(getTime(), res, dk /*dummy*/);
  output(taskInfo);


  // Get the current state
  StateStream stateStream(getNumContinousStates());
  getState(stateStream);
  Vector state = stateStream.getState();

  Vector trimState = stateStream.getState();
  // Buld up the trim function
  TrimFunction trimFunction(*this);

  // Try to find a minimum
  real_type atol = 1e-7;
  real_type rtol = 1e-8;
  bool ret = GaussNewton(trimFunction, getTime(), trimState, atol, rtol);
//   bool ret = LevenbergMarquart(trimFunction, getTime(), trimState, atol, rtol);
  if (ret) {
    stateStream.setState(trimState);
    setState(stateStream);
  } else {
    stateStream.setState(state);
    setState(stateStream);
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

Environment*
System::getEnvironment(void) const
{
  /// Hmmm, FIXME
  return const_cast<Environment*>((const Environment*)mEnvironment);
}

} // namespace OpenFDM
