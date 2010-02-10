/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "SafeReciprocal.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SafeReciprocal, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Real, Epsilon, Serialized)
  END_OPENFDM_OBJECT_DEF

SafeReciprocal::SafeReciprocal(const std::string& name, const real_type& eps) :
  SimpleDirectModel(name),
  mEpsilon(eps)
{
  addInputPort("input");
}

SafeReciprocal::~SafeReciprocal(void)
{
}

void
SafeReciprocal::output(Context& context) const
{
  // FIXME, optimize, move that into a proper context ...
  // For now make it work
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      real_type value = context.getInputValue(0)(i, j);
      real_type rvalue = copysign(real_type(1)/(fabs(value) + mEpsilon), value);
      context.getOutputValue()(i, j) = rvalue;
    }
  }
}

} // namespace OpenFDM
