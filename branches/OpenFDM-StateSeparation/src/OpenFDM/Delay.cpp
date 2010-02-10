/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Delay.h"

#include "Assert.h"
#include "Object.h"
#include "Model.h"
#include "ModelContext.h"
#include "TypeInfo.h"
#include "Variant.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Delay, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, Delay, Serialized)
  END_OPENFDM_OBJECT_DEF

Delay::Delay(const std::string& name, unsigned delay) :
  Model(name),
  mInputPort(newMatrixInputPort("input", true)),
  mOutputPort(newMatrixOutputPort("output")),
  mDelay(0),
  mInitialValue(Matrix::zeros(1, 1))
{
  mMatrixStateInfo = new MatrixListStateInfo;
  addDiscreteStateInfo(mMatrixStateInfo);

  setDelay(delay);
}

Delay::~Delay()
{
}

bool
Delay::alloc(ModelContext& context) const
{
  Size sz = size(mInitialValue);
  Log(Initialization, Debug)
    << "Size for Delay is detemined by the static initial value "
    << "with size: " << trans(sz) << std::endl;
  if (!context.getPortValueList().setOrCheckPortSize(mInputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  if (!context.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for output port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
Delay::init(const Task&,DiscreteStateValueVector& discreteState,
            ContinousStateValueVector&,const PortValueList&) const
{
  discreteState[*mMatrixStateInfo].clear();
  for (unsigned i = 0; i <= mDelay; ++i)
    discreteState[*mMatrixStateInfo].push_back(mInitialValue);
}

void
Delay::output(const Task&,const DiscreteStateValueVector& discreteState,
              const ContinousStateValueVector&,
              PortValueList& portValues) const
{
  portValues[mOutputPort] = discreteState[*mMatrixStateInfo].front();
}

void
Delay::update(const DiscreteTask&, DiscreteStateValueVector& discreteState,
              const ContinousStateValueVector&,
              const PortValueList& portValues) const
{
  discreteState[*mMatrixStateInfo].rotate(portValues[mInputPort]);
}

unsigned
Delay::getDelay(void) const
{
  return mDelay;
}

void
Delay::setDelay(unsigned delay)
{
  mDelay = delay;
  mInputPort.setDirectInput(mDelay == 0);
}

const Matrix&
Delay::getInitialValue() const
{
  return mInitialValue;
}

void
Delay::setInitialValue(const Matrix& initialValue)
{
  mInitialValue = initialValue;
}

} // namespace OpenFDM
