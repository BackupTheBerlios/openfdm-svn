/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Delay.h"

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "LeafContext.h"
#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Delay, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, Delay, Serialized)
  END_OPENFDM_OBJECT_DEF

Delay::Delay(const std::string& name) :
  Model(name),
  mInputPort(newMatrixInputPort("input")),
  mOutputPort(newMatrixOutputPort("output")),
  mDelay(0)
{
  mMatrixStateInfo = new MatrixListStateInfo;
  addDiscreteStateInfo(mMatrixStateInfo);
}

Delay::~Delay()
{
}

bool
Delay::alloc(LeafContext& leafContext) const
{
  Size sz = size(leafContext.mPortValueList[mInputPort]);
  leafContext.mPortValueList.setPortSize(mOutputPort, sz);
  leafContext.mDiscreteState.setValue(*mMatrixStateInfo, leafContext);
  return true;
}

void
Delay::init(DiscreteStateValueVector& discreteState,
            ContinousStateValueVector&) const
{
  discreteState[*mMatrixStateInfo].clear();
  for (unsigned i = 0; i <= mDelay; ++i)
    discreteState[*mMatrixStateInfo].push_back(mInitialValue);
}

void
Delay::output(const DiscreteStateValueVector& discreteState,
              const ContinousStateValueVector&,
              PortValueList& portValues) const
{
  portValues[mOutputPort] = discreteState[*mMatrixStateInfo].front();
}

void
Delay::update(DiscreteStateValueVector& discreteState,
              ContinousStateValueVector&,
              const PortValueList& portValues) const
{
  discreteState[*mMatrixStateInfo].rotate(portValues[mInputPort]);
}

bool
Delay::dependsOn(const PortId& in, const PortId& out) const
{
  if (mDelay != 0)
    return false;
  return in == getPortId(mInputPort.getPortIndex())
    && out == getPortId(mOutputPort.getPortIndex());
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
