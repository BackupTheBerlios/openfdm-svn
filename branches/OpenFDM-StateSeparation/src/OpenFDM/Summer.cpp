/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Summer.h"

#include "Types.h"
#include "Matrix.h"
#include "Model.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Summer, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, NumSummands, Serialized)
  END_OPENFDM_OBJECT_DEF

Summer::Summer(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumSummands(2);
}

Summer::~Summer(void)
{
}

void
Summer::output(Context& context) const
{
  if (!getNumInputPorts())
    return;
  if (mSigns.front() == Plus)
    context.getOutputValue() = context.getInputValue(0);
  else
    context.getOutputValue() = -context.getInputValue(0);
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    if (mSigns[i] == Plus)
      context.getOutputValue() += context.getInputValue(i);
    else
      context.getOutputValue() -= context.getInputValue(i);
  }
}

unsigned
Summer::getNumSummands(void) const
{
  return getNumInputPorts();
}

void
Summer::setNumSummands(unsigned num)
{
  mSigns.resize(num, Plus);
  setNumInputPorts(num);
}

void
Summer::setInputSign(unsigned num, Sign sign)
{
  if (mSigns.size() <= num)
    return;
  mSigns[num] = sign;
}

Summer::Sign
Summer::getInputSign(unsigned num) const
{
  if (mSigns.size() <= num)
    return Plus;
  return mSigns[num];
}

} // namespace OpenFDM
