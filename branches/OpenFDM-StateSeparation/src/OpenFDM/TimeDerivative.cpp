/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "TimeDerivative.h"

#include "Assert.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(TimeDerivative, Model)
  END_OPENFDM_OBJECT_DEF

TimeDerivative::TimeDerivative(const std::string& name) :
  Model(name),
  mInputPort(newMatrixInputPort("input", false)),
  mOutputPort(newMatrixOutputPort("output"))
{
  mDerivativeStateInfo = new MatrixStateInfo;
  addDiscreteStateInfo(mDerivativeStateInfo);
  mOldValueStateInfo = new MatrixStateInfo;
  addDiscreteStateInfo(mOldValueStateInfo);
  mOldTStateInfo = new MatrixStateInfo;
  addDiscreteStateInfo(mOldTStateInfo);
}

TimeDerivative::~TimeDerivative(void)
{
}

bool
TimeDerivative::alloc(ModelContext& leafContext) const
{
  Size sz = size(leafContext.getPortValueList()[mInputPort]);
  Log(Initialization, Debug)
    << "Size for Integrator is detemined by the initial input "
    << "port with size: " << trans(sz) << std::endl;
  if (!leafContext.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
TimeDerivative::init(const Task& task, DiscreteStateValueVector& discreteState,
                     ContinousStateValueVector&,
                     const PortValueList& portValues) const
{
  Size sz = size(portValues[mInputPort]);
  discreteState[*mDerivativeStateInfo].resize(sz(0), sz(1));
  discreteState[*mDerivativeStateInfo].clear();
  discreteState[*mOldValueStateInfo].resize(sz(0), sz(1));
  discreteState[*mOldValueStateInfo] = portValues[mInputPort];
  discreteState[*mOldTStateInfo].resize(1, 1);
  discreteState[*mOldTStateInfo](0, 0) = task.getTime();
}

void
TimeDerivative::output(const Task& task,
                       const DiscreteStateValueVector& discreteState,
                       const ContinousStateValueVector&,
                       PortValueList& portValues) const
{
  real_type tOld = discreteState[*mOldTStateInfo](0, 0);
  real_type dt = task.getTime() - tOld;
  real_type dtMin = sqrt(Limits<real_type>::epsilon());
  if (dt < dtMin) {
    // For times very near at tOld, just use the old derivative
    portValues[mOutputPort] = discreteState[*mDerivativeStateInfo];
  } else {
    // The numerical derivative
    Matrix deriv = portValues[mInputPort] - discreteState[*mOldValueStateInfo];
    deriv *= 1/dt;

    if (dt < 2*dtMin) {
      // For times where numerical derivative starts having a value beyond
      // roundoff interpolate between the old and the numerical derivative
      portValues[mOutputPort]
        = interpolate(dt, dtMin, discreteState[*mDerivativeStateInfo],
                      2*dtMin, deriv);
    } else {
      // For times where numerical derivative provides valid values use the
      // numerical derivative
      portValues[mOutputPort] = deriv;
    }
  }
}

void
TimeDerivative::update(const DiscreteTask& discreteTask,
                       DiscreteStateValueVector& discreteState,
                       const ContinousStateValueVector&,
                       const PortValueList& portValues) const
{
  // Just compute the derivative.
  real_type tOld = discreteState[*mOldTStateInfo](0, 0);
  real_type dt = discreteTask.getTime() - tOld;
  if (Limits<real_type>::safe_min() < fabs(dt)) {
    discreteState[*mDerivativeStateInfo]
      = (1/dt)*(portValues[mInputPort] - discreteState[*mOldValueStateInfo]);
  }

  // Store the old value and this current stepsize
  discreteState[*mOldValueStateInfo] = portValues[mInputPort];
  discreteState[*mOldTStateInfo](0, 0) = discreteTask.getTime();
}

} // namespace OpenFDM
