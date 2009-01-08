/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Gain.h"

#include <string>
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Gain, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Real, Gain, Serialized)
  END_OPENFDM_OBJECT_DEF

Gain::Gain(const std::string& name, const real_type& gain) :
  SimpleDirectModel(name),
  mGain(gain)
{
  addInputPort("input");
}

Gain::~Gain(void)
{
}

void
Gain::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      context.getOutputValue()(j, k) = mGain*context.getInputValue(0)(j, k);
    }
  }
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
