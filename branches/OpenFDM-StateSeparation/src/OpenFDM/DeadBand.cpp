/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "DeadBand.h"

#include "Matrix.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DeadBand, UnaryModel)
  DEF_OPENFDM_PROPERTY(Real, Width, Serialized)
  END_OPENFDM_OBJECT_DEF

DeadBand::DeadBand(const std::string& name, const real_type& width) :
  UnaryModel(name),
  mWidth(width)
{
}

DeadBand::~DeadBand(void)
{
}
  
ModelContext*
DeadBand::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
DeadBand::output(const Matrix& inputValue, Matrix& outputValue) const
{
  for (unsigned i = 0; i < rows(inputValue); ++i)
    for (unsigned j = 0; j < cols(inputValue); ++j)
      outputValue(i, j) = deadBand(inputValue(i, j), mWidth);
}

const real_type&
DeadBand::getWidth(void) const
{
  return mWidth;
}

void
DeadBand::setWidth(const real_type& width)
{
  mWidth = width;
}

} // namespace OpenFDM
