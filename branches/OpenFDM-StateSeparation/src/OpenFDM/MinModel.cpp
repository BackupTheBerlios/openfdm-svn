/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "MinModel.h"

#include <string>
#include "Types.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MinModel, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, NumMinInputs, Serialized)
  END_OPENFDM_OBJECT_DEF

MinModel::MinModel(const std::string& name) :
  SimpleDirectModel(name)
{
}

MinModel::~MinModel(void)
{
}

void
MinModel::output(Context& context) const
{
  if (!getNumInputPorts())
    return;
  context.getOutputValue() = context.getInputValue(0);
  for (unsigned i = 1; i < getNumInputPorts(); ++i)
    context.getOutputValue()
      = LinAlg::min(context.getOutputValue(), context.getInputValue(i));
}

unsigned
MinModel::getNumMinInputs(void) const
{
  return getNumInputPorts();
}

void
MinModel::setNumMinInputs(unsigned num)
{
//   setNumInputPorts(num);
}

} // namespace OpenFDM
