/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ODESolver_H
#define OpenFDM_ODESolver_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

//class Timestepper
class ODESolver :
    public Object {
public:
  struct Stats {
    Stats(void) : numSteps(0), numFailed(0), numIter(0) {}
    unsigned numSteps;
    unsigned numFailed;
    unsigned numIter;
  };

  ODESolver(void);
  virtual ~ODESolver(void);

  virtual void invalidateHistory(void);
  virtual bool integrate(real_type toTEnd) = 0;
  virtual bool denseOutput(real_type t, Vector& out);

  bool reached(real_type tEnd);
  real_type maxStepsize(real_type tEnd);

  real_type getStepsize(void) const
  { return mStepsize; }
  void setStepsize(real_type stepsize)
  { mStepsize = stepsize; }

  real_type getTime(void) const
  { return mTime; }
  void setTime(real_type t)
  { mTime = t; }

  const Vector& getState(void) const
  { return mState; }
  void setState(const Vector& state)
  { if (mState == state) return; invalidateHistory(); mState = state; }

  const Stats& getStats(void) const
  { return mStats; }

  void evalFunction(real_type t, const Vector& v, Vector& out)
  { mModel->evalFunction(t, v, out); }
  void evalJacobian(real_type t, const Vector& v, Matrix& jac)
  { mModel->evalJacobian(t, v, jac); }

  void setModel(Model* model)
  { mModel = model; }

protected:
  real_type mStepsize;
  real_type mTime;
  Vector mState;

  /// managed_ptr ???
  managed_ptr<Model> mModel;

  Stats mStats;
};

} // namespace OpenFDM

#endif
