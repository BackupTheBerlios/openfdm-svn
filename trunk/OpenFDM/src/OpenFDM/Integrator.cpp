/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Integrator.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Integrator, Model)
  END_OPENFDM_OBJECT_DEF

Integrator::Integrator(const std::string& name) :
  Model(name)
{
  addSampleTime(SampleTime::Continous);

  setNumInputPorts(1);
  setInputPortName(0, "derivatirve");

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Integrator::getIntegralOutput);

  addStoredProperty("initialValue",
                    Property(this, &Integrator::getInitialValue,
                             &Integrator::setInitialValue));
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
  OpenFDMAssert(getInputPort(0)->isConnected());

  // Just compute the integral.
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  const Matrix& input = mh.getMatrixValue();
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
