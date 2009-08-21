/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich
 *
 */

#include "LogicalOperator.h"

#include "ModelContext.h"
#include "PortValueList.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(LogicalOperator, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Unsigned, Type, Serialized)
  END_OPENFDM_OBJECT_DEF

LogicalOperator::LogicalOperator(const std::string& name, Type type) :
  SimpleDirectModel(name),
  mType(type)
{
  setType(type);
}

LogicalOperator::~LogicalOperator(void)
{
}

void
LogicalOperator::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      switch (mType) {
      case LogicalNOT:
        {
          real_type input0 = context.getInputValue(0)(i, j);
          context.getOutputValue()(i, j) = !bool(input0);
        }
        break;
        
      default:
        {
          real_type input0 = context.getInputValue(0)(i, j);
          real_type input1 = context.getInputValue(1)(i, j);
          switch (mType) {
          case LogicalAND:
            context.getOutputValue()(i, j) = (bool(input0) && bool(input1));
            break;
          case LogicalOR:
            context.getOutputValue()(i, j) = (bool(input0) || bool(input1));
            break;
          case LogicalNAND:
            context.getOutputValue()(i, j) = !(bool(input0) && bool(input1));
            break;
          case LogicalNOR:
            context.getOutputValue()(i, j) = !(bool(input0) || bool(input1));
            break;
          case LogicalXOR:
            context.getOutputValue()(i, j) = (bool(input0) ^ bool(input1));
            break;
          default:
            OpenFDMAssert(false);
            break;
          }
        }
        break;
      }
    }
  }
}

void
LogicalOperator::setType(const LogicalOperator::Type& type)
{
  mType = type;
  switch (mType) {
  case LogicalNOT:
    setNumInputPorts(1);
    break;
    
  default:
    setNumInputPorts(2);
    break;
  }
}

const LogicalOperator::Type&
LogicalOperator::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
