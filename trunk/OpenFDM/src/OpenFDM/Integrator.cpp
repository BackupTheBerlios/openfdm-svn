/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Expression.h"
#include "Integrator.h"

namespace OpenFDM {

DiscreteIntegrator::DiscreteIntegrator(const std::string& name) :
  Model(name)
{
  setNumInputPorts(1);
  addProperty("initialValue", Property(this, &DiscreteIntegrator::getInitialValue, &DiscreteIntegrator::setInitialValue));

  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &DiscreteIntegrator::getIntegralOutput));
  addProperty("output", Property(this, &DiscreteIntegrator::getIntegralOutput));
}

DiscreteIntegrator::~DiscreteIntegrator(void)
{
}

bool
DiscreteIntegrator::init(void)
{
  OpenFDMAssert(getInputPort(0).isValid());
  if (mInitialValue.rows() == 0 || mInitialValue.cols() == 0) {
    mInitialValue.resize(getInputPort(0).getValue().toMatrix());
    mInitialValue.clear();
  }
  mIntegralState = mInitialValue;
  mIntegralOutput = mIntegralState;
  return true;
}

void
DiscreteIntegrator::output(void)
{
  mIntegralOutput = mIntegralState;
}

void
DiscreteIntegrator::update(real_type dt)
{
  OpenFDMAssert(getInputPort(0).isValid());

  // Just compute the integral.
  Matrix input = getInputPort(0).getValue().toMatrix();
  OpenFDMAssert(size(input) == size(mIntegralState));
  if (size(input) == size(mIntegralState))
    mIntegralState += dt*input;
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

const Matrix&
DiscreteIntegrator::getIntegralOutput(void) const
{
  return mIntegralOutput;
}

} // namespace OpenFDM
