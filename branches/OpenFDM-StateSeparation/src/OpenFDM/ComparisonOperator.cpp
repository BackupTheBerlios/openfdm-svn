/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
 *
 */

#include "ComparisonOperator.h"

#include "ModelContext.h"
#include "PortValueList.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ComparisonOperator, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, Type, Serialized)
  END_OPENFDM_OBJECT_DEF

ComparisonOperator::ComparisonOperator(const std::string& name, Type type) :
  SimpleDirectModel(name),
  mType(type)
{
  setNumInputPorts(2);
}

ComparisonOperator::~ComparisonOperator(void)
{
}

void
ComparisonOperator::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      real_type input0 = context.getInputValue(0)(i, j);
      real_type input1 = context.getInputValue(1)(i, j);
      switch (mType) {
      case Equal:
        context.getOutputValue()(i, j) = (input0 == input1);
        break;
      case NotEqual:
        context.getOutputValue()(i, j) = (input0 != input1);
        break;
      case Less:
        context.getOutputValue()(i, j) = (input0 < input1);
        break;
      case LessEqual:
        context.getOutputValue()(i, j) = (input0 <= input1);
        break;
      case Greater:
        context.getOutputValue()(i, j) = (input0 > input1);
        break;
      case GreaterEqual:
        context.getOutputValue()(i, j) = (input0 >= input1);
        break;
      default:
        OpenFDMAssert(false);
        break;
      }
    }
  }
}

void
ComparisonOperator::setType(const ComparisonOperator::Type& type)
{
  mType = type;
}

const ComparisonOperator::Type&
ComparisonOperator::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
