/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
 *
 */

#include "BreakPointLookup.h"

#include "Matrix.h"
#include "Vector.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(BreakPointLookup, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(BreakPointLookup, BreakPointVector, Serialized)
  END_OPENFDM_OBJECT_DEF

BreakPointLookup::BreakPointLookup(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumInputPorts(1);
}

BreakPointLookup::~BreakPointLookup(void)
{
}

void
BreakPointLookup::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      real_type input = context.getInputValue(0)(j, k);
      context.getOutputValue()(j, k) = mBreakPointVector.lookup(input);
    }
  }
}

} // namespace OpenFDM
