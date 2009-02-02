/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "DiscreteIntegrator.h"

#include "Assert.h"
#include "Task.h"
#include "ModelContext.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DiscreteIntegrator, Model)
  DEF_OPENFDM_PROPERTY(Matrix, InitialValue, Serialized)
  DEF_OPENFDM_PROPERTY(Matrix, MinSaturation, Serialized)
  DEF_OPENFDM_PROPERTY(Matrix, MaxSaturation, Serialized)
  END_OPENFDM_OBJECT_DEF

DiscreteIntegrator::DiscreteIntegrator(const std::string& name) :
  Model(name),
  mInputPort(newMatrixInputPort("input", false)),
  mOutputPort(newMatrixOutputPort("output")),
  mInitialValue(Matrix::zeros(1, 1))
{
  mMatrixStateInfo = new MatrixStateInfo;
  addDiscreteStateInfo(mMatrixStateInfo);
}

DiscreteIntegrator::~DiscreteIntegrator(void)
{
}

bool
DiscreteIntegrator::alloc(ModelContext& leafContext) const
{
  Size sz;
  if (getEnableInitialValuePort()) {
    sz = size(leafContext.getPortValueList()[mInitialValuePort]);
    Log(Initialization, Debug)
      << "Size for Integrator is detemined by the initial input "
      << "port with size: " << trans(sz) << std::endl;
  } else {
    sz = size(mInitialValue);
    Log(Initialization, Debug)
      << "Size for Integrator is detemined by the static initial value "
      << "with size: " << trans(sz) << std::endl;
  }
  if (!leafContext.getPortValueList().setOrCheckPortSize(mInputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  if (!leafContext.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
DiscreteIntegrator::init(const Task&, DiscreteStateValueVector& discreteState,
                         ContinousStateValueVector&,
                         const PortValueList& portValues) const
{
  if (getEnableInitialValuePort()) {
    // external initial condition
    discreteState[*mMatrixStateInfo] = portValues[mInitialValuePort];
  } else {
    // internal initial condition
    discreteState[*mMatrixStateInfo] = mInitialValue;
  }
}

void
DiscreteIntegrator::output(const Task&,
                           const DiscreteStateValueVector& discreteState,
                           const ContinousStateValueVector&,
                           PortValueList& portValues) const
{
  portValues[mOutputPort] = discreteState[*mMatrixStateInfo];
}

void
DiscreteIntegrator::doUpdate(Matrix& integralValue, const Matrix& derivative,
                             const real_type& dt) const
{
  integralValue += dt*derivative;

  if (size(mMaxSaturation) == size(integralValue))
    integralValue = LinAlg::min(integralValue, mMaxSaturation);
  if (size(mMinSaturation) == size(integralValue))
    integralValue = LinAlg::max(integralValue, mMinSaturation);
}

void
DiscreteIntegrator::update(const DiscreteTask& discreteTask,
                           DiscreteStateValueVector& discreteState,
                           const ContinousStateValueVector&,
                           const PortValueList& portValues) const
{
  // Just compute the integral.
  real_type dt = discreteTask.getStepsize();
  doUpdate(discreteState[*mMatrixStateInfo], portValues[mInputPort], dt);
}

const Matrix&
DiscreteIntegrator::getInitialValue(void) const
{
  return mInitialValue;
}

void
DiscreteIntegrator::setInitialValue(const Matrix& value)
{
  mInitialValue = value;
}

bool
DiscreteIntegrator::getEnableInitialValuePort() const
{
  return !mInitialValuePort.empty();
}

void
DiscreteIntegrator::setEnableInitialValuePort(bool enable)
{
  if (enable == getEnableInitialValuePort())
    return;

  if (enable)
    mInitialValuePort = newMatrixInputPort("initialValue", true);
  else 
    mInitialValuePort.clear();
}

const Matrix&
DiscreteIntegrator::getMinSaturation(void) const
{
  return mMinSaturation;
}

void
DiscreteIntegrator::setMinSaturation(const Matrix& value)
{
  mMinSaturation = value;
}

const Matrix&
DiscreteIntegrator::getMaxSaturation(void) const
{
  return mMaxSaturation;
}

void
DiscreteIntegrator::setMaxSaturation(const Matrix& value)
{
  mMaxSaturation = value;
}

} // namespace OpenFDM
