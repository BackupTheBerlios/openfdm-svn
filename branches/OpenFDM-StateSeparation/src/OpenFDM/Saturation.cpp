/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Saturation.h"

#include "Matrix.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Saturation, UnaryModel)
  DEF_OPENFDM_PROPERTY(Matrix, MinSaturation, Serialized)
  DEF_OPENFDM_PROPERTY(Matrix, MaxSaturation, Serialized)
  END_OPENFDM_OBJECT_DEF

Saturation::Saturation(const std::string& name) :
  UnaryModel(name)
{
}

Saturation::~Saturation(void)
{
}
  
ModelContext*
Saturation::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
Saturation::output(const Matrix& inputValue, Matrix& outputValue) const
{
  outputValue = inputValue;
  if (0 < rows(mMaxSaturation) && 0 < cols(mMaxSaturation))
    outputValue = LinAlg::min(outputValue, mMaxSaturation);
  if (0 < rows(mMinSaturation) && 0 < cols(mMinSaturation))
    outputValue = LinAlg::max(outputValue, mMinSaturation);
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
