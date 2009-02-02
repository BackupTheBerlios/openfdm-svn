/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "MaxModel.h"

#include "Types.h"
#include "Matrix.h"
#include "Model.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MaxModel, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, NumMaxInputs, Serialized)
  END_OPENFDM_OBJECT_DEF

MaxModel::MaxModel(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumMaxInputs(2);
}

MaxModel::~MaxModel(void)
{
}

void
MaxModel::output(Context& context) const
{
  if (!getNumInputPorts())
    return;
  context.getOutputValue() = context.getInputValue(0);
  for (unsigned i = 1; i < getNumInputPorts(); ++i)
    context.getOutputValue()
      = LinAlg::max(context.getOutputValue(), context.getInputValue(i));
}

unsigned
MaxModel::getNumMaxInputs(void) const
{
  return getNumInputPorts();
}

void
MaxModel::setNumMaxInputs(unsigned num)
{
  setNumInputPorts(num);
}

} // namespace OpenFDM
