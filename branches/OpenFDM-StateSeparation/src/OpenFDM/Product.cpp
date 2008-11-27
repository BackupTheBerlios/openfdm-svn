/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Product.h"

#include <string>
#include "Types.h"
#include "Matrix.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Product, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, NumFactors, Serialized)
  END_OPENFDM_OBJECT_DEF

Product::Product(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumFactors(2);
}

Product::~Product(void)
{
}

void
Product::output(Context& context) const
{
  if (!getNumInputPorts())
    return;
  Size sz = size(context.getInputValue(0));
  context.getOutputValue() = context.getInputValue(0);
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    for (unsigned j = 0; j < sz(0); ++j)
      for (unsigned k = 0; k < sz(1); ++k)
        context.getOutputValue()(j, k) *= context.getInputValue(i)(j, k);
  }
}

unsigned
Product::getNumFactors(void) const
{
  return getNumInputPorts();
}

void
Product::setNumFactors(unsigned num)
{
  setNumInputPorts(num);
}

} // namespace OpenFDM
