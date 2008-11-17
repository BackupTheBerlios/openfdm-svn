/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Gain.h"

#include <string>
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Gain, UnaryModel)
  DEF_OPENFDM_PROPERTY(Real, Gain, Serialized)
  END_OPENFDM_OBJECT_DEF

Gain::Gain(const std::string& name, const real_type& gain) :
  UnaryModel(name),
  mGain(gain)
{
}

Gain::~Gain(void)
{
}

ModelContext*
Gain::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
Gain::output(const Matrix& inputValue, Matrix& outputValue) const
{
  outputValue = mGain*inputValue;
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
