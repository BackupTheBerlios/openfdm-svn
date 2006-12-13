/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Integrator.h"

#include "Assert.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Integrator, Model)
  DEF_OPENFDM_PROPERTY(Matrix, InitialValue, Serialized)
  END_OPENFDM_OBJECT_DEF

Integrator::Integrator(const std::string& name) :
  Model(name)
{
  addSampleTime(SampleTime::Continous);

  setNumInputPorts(1);
  setInputPortName(0, "derivatirve");

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Integrator::getIntegralOutput);
}

Integrator::~Integrator(void)
{
}

bool
Integrator::init(void)
{
  mDerivativeHandle = getInputPort(0)->toMatrixPortHandle();
  if (!mDerivativeHandle.isConnected())
    return false;

  // The initial value defaults to zero
  if (rows(mInitialValue) == 0 || cols(mInitialValue) == 0) {
    mInitialValue.resize(mDerivativeHandle.getMatrixValue());
    mInitialValue.clear();
  }

  setNumContinousStates(rows(mInitialValue)*cols(mInitialValue));

  mIntegralState = mInitialValue;
  mIntegralOutput = mIntegralState;
  return Model::init();
}

void
Integrator::output(const TaskInfo&)
{
  mIntegralOutput = mIntegralState;
}

void
Integrator::setState(const StateStream& state)
{
  state.readSubState(mIntegralState);
}

void
Integrator::getState(StateStream& state) const
{
  state.writeSubState(mIntegralState);
}

void
Integrator::getStateDeriv(StateStream& stateDeriv)
{
  // Just return the derivative
  OpenFDMAssert(mDerivativeHandle.isConnected());
  const Matrix& input = mDerivativeHandle.getMatrixValue();
  OpenFDMAssert(size(input) == size(mIntegralState));
  stateDeriv.writeSubState(input);
}

const Matrix&
Integrator::getInitialValue(void) const
{
  return mInitialValue;
}

void
Integrator::setInitialValue(const Matrix& value)
{
  mInitialValue = value;
}

const Matrix&
Integrator::getIntegralOutput(void) const
{
  return mIntegralOutput;
}

} // namespace OpenFDM
