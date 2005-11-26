/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Integrator.h"

namespace OpenFDM {

Integrator::Integrator(const std::string& name) :
  Model(name)
{
  addSampleTime(SampleTime::Continous);

  setNumInputPorts(1);
  setInputPortName(0, "derivatirve");

  setNumOutputPorts(1);
  setOutputPort(0, "output",
                Property(this, &Integrator::getIntegralOutput));

  addProperty("initialValue",
              Property(this, &Integrator::getInitialValue,
                       &Integrator::setInitialValue));
  addProperty("output",
              Property(this, &Integrator::getIntegralOutput));
}

Integrator::~Integrator(void)
{
}

bool
Integrator::init(void)
{
  OpenFDMAssert(getInputPort(0)->isConnected());

  // The initial value defaults to zero
  if (rows(mInitialValue) == 0 || cols(mInitialValue) == 0) {
    mInitialValue.resize(getInputPort(0)->getValue().toMatrix());
    mInitialValue.clear();
  }

  setNumContinousStates(rows(mInitialValue)*cols(mInitialValue));

  mIntegralState = mInitialValue;
  mIntegralOutput = mIntegralState;
  return true;
}

void
Integrator::output(const TaskInfo&)
{
  mIntegralOutput = mIntegralState;
}

void
Integrator::setState(const Vector& state, unsigned offset)
{
  // FIXME reshape ...
  for (unsigned j = 1; j <= cols(mIntegralState); ++j) {
    for (unsigned i = 1; i <= rows(mIntegralState); ++i) {
      mIntegralState(i, j) = state(offset + i + (j-1)*rows(mIntegralState));
    }
  }
}

void
Integrator::getState(Vector& state, unsigned offset) const
{
  // FIXME reshape ...
  for (unsigned j = 1; j <= cols(mIntegralState); ++j) {
    for (unsigned i = 1; i <= rows(mIntegralState); ++i) {
      state(offset + i + (j-1)*rows(mIntegralState)) = mIntegralState(i, j);
    }
  }
}

void
Integrator::getStateDeriv(Vector& stateDeriv, unsigned offset)
{
  OpenFDMAssert(getInputPort(0)->isConnected());

  // Just compute the integral.
  Matrix input = getInputPort(0)->getValue().toMatrix();
  OpenFDMAssert(size(input) == size(mIntegralState));

  // FIXME reshape ...
  for (unsigned j = 1; j <= cols(input); ++j) {
    for (unsigned i = 1; i <= rows(input); ++i) {
      stateDeriv(offset + i + (j-1)*rows(input)) = input(i, j);
    }
  }
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
