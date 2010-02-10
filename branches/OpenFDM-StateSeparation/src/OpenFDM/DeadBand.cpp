/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "DeadBand.h"

#include "Matrix.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DeadBand, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Real, Width, Serialized)
  END_OPENFDM_OBJECT_DEF

DeadBand::DeadBand(const std::string& name, const real_type& width) :
  SimpleDirectModel(name),
  mWidth(width)
{
  addInputPort("input");
}

DeadBand::~DeadBand(void)
{
}
  
void
DeadBand::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned i = 0; i < sz(0); ++i)
    for (unsigned j = 0; j < sz(1); ++j)
      context.getOutputValue()(i, j)
        = deadBand(context.getInputValue(0)(i, j), mWidth);
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
