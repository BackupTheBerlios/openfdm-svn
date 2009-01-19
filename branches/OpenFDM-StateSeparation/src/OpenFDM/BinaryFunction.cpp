/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich
 *
 */

#include "BinaryFunction.h"

#include "ModelContext.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(BinaryFunction, SimpleDirectModel)
  END_OPENFDM_OBJECT_DEF

BinaryFunction::BinaryFunction(const std::string& name, Type type) :
  SimpleDirectModel(name),
  mType(type)
{
  setNumInputPorts(2);
}

BinaryFunction::~BinaryFunction(void)
{
}

void
BinaryFunction::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      real_type input0 = context.getInputValue(0)(i, j);
      real_type input1 = context.getInputValue(1)(i, j);
      switch (mType) {
      case Atan2:
        context.getOutputValue()(i, j) = atan2(input0, input1);
        break;
      case Pow:
        context.getOutputValue()(i, j) = pow(input0, input1);
        break;
      case Div:
        context.getOutputValue()(i, j) = input0 / input1;
        break;
      default:
        OpenFDMAssert(false);
        break;
      }
    }
  }
}

void
BinaryFunction::setType(const BinaryFunction::Type& type)
{
  mType = type;
}

const BinaryFunction::Type&
BinaryFunction::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
