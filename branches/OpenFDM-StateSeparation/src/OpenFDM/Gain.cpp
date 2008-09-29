/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Gain.h"

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "LeafContext.h"
#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Gain, Model)
  DEF_OPENFDM_PROPERTY(Real, Gain, Serialized)
  END_OPENFDM_OBJECT_DEF

Gain::Gain(const std::string& name, const real_type& gain) :
  Model(name),
  mInputPort(newMatrixInputPort("input", true)),
  mOutputPort(newMatrixOutputPort("output")),
  mGain(gain)
{
}

Gain::~Gain(void)
{
}

bool
Gain::alloc(LeafContext& leafContext) const
{
  Size sz = size(leafContext.mPortValueList[mInputPort]);
  Log(Initialization, Debug)
    << "Size for Gain is detemined by the input port with size: "
    << trans(sz) << std::endl;
  if (!leafContext.mPortValueList.setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for output port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
Gain::output(const DiscreteStateValueVector&, const ContinousStateValueVector&,
             PortValueList& portValues) const
{
  portValues[mOutputPort] = mGain*portValues[mInputPort];
}

const real_type&
Gain::getGain(void) const
{
  return mGain;
}

void
Gain::setGain(const real_type& gain)
{
  mGain = gain;
}

} // namespace OpenFDM
