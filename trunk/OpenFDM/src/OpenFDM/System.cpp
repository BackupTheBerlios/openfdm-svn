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
#include "System.h"

namespace OpenFDM {

System::System(const std::string& name) :
  ModelGroup(name)
{
  setTimestepper(new ExplicitEuler);
}

System::~System(void)
{
}

bool
System::init(void)
{
  if (!mTimestepper)
    return false;

  mTimestepper->setTime(0);
  return ModelGroup::init();
}

bool
System::simulate(real_type tEnd)
{
  //// FIXME, only works for discrete systems with the same schedule than the
  /// stepsize

  Vector state(getNumContinousStates());

  // Read the state from the systems. Since this is possible to change from
  // outside we need that
  getState(state, 0);
  // FIXME: must check for the correctness of the state here ...
  mTimestepper->setState(state);

  real_type stepsize = mTimestepper->getStepsize();
  while (getTime() < tEnd) {
    real_type thisTEnd = min(tEnd, mTimestepper->getTime() + stepsize);

    /// do the outputs for the ModelGroup
    output();

    // Integrate the continous systems
    mTimestepper->integrate(mTimestepper->getTime() + stepsize);

    setState(mTimestepper->getTime(), mTimestepper->getState(), 0);
    /// FIXME,
    getStateDeriv(state, 0);

    // update the discrete systems ...
    update(stepsize);
  }
}

bool
System::trim(void)
{
  /// FIXME
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
