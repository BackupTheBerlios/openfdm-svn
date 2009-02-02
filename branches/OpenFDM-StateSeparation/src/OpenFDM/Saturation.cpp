/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Saturation.h"

#include "Matrix.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Saturation, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Matrix, MinSaturation, Serialized)
  DEF_OPENFDM_PROPERTY(Matrix, MaxSaturation, Serialized)
  END_OPENFDM_OBJECT_DEF

Saturation::Saturation(const std::string& name) :
  SimpleDirectModel(name)
{
  addInputPort("input");
}

Saturation::~Saturation(void)
{
}
  
void
Saturation::output(Context& context) const
{
  context.getOutputValue() = context.getInputValue(0);
  if (0 < rows(mMaxSaturation) && 0 < cols(mMaxSaturation))
    context.getOutputValue()
      = LinAlg::min(context.getOutputValue(), mMaxSaturation);
  if (0 < rows(mMinSaturation) && 0 < cols(mMinSaturation))
    context.getOutputValue()
      = LinAlg::max(context.getOutputValue(), mMinSaturation);
}

const Matrix&
Saturation::getMinSaturation(void) const
{
  return mMinSaturation;
}

void
Saturation::setMinSaturation(const Matrix& minSaturation)
{
  mMinSaturation = minSaturation;
}

const Matrix&
Saturation::getMaxSaturation(void) const
{
  return mMaxSaturation;
}

void
Saturation::setMaxSaturation(const Matrix& maxSaturation)
{
  mMaxSaturation = maxSaturation;
}

} // namespace OpenFDM
