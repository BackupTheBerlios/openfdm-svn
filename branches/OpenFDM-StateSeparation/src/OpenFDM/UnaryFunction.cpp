/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "UnaryFunction.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(UnaryFunction, SimpleDirectModel)
  END_OPENFDM_OBJECT_DEF

UnaryFunction::UnaryFunction(const std::string& name, const Type& type) :
  SimpleDirectModel(name),
  mType(type)
{
  addInputPort("input");
}

UnaryFunction::~UnaryFunction(void)
{
}

void
UnaryFunction::output(Context& context) const
{
  // FIXME, optimize, move that into a proper context ...
  // For now make it work
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      switch (mType) {
      case Abs:
        context.getOutputValue()(i, j) = fabs(context.getInputValue(0)(i, j));
        break;
      case Acos:
        context.getOutputValue()(i, j) = acos(context.getInputValue(0)(i, j));
        break;
      case Asin:
        context.getOutputValue()(i, j) = asin(context.getInputValue(0)(i, j));
        break;
      case Atan:
        context.getOutputValue()(i, j) = atan(context.getInputValue(0)(i, j));
        break;
      case Ceil:
        context.getOutputValue()(i, j) = ceil(context.getInputValue(0)(i, j));
        break;
      case Cos:
        context.getOutputValue()(i, j) = cos(context.getInputValue(0)(i, j));
        break;
      case Exp:
        context.getOutputValue()(i, j) = exp(context.getInputValue(0)(i, j));
        break;
      case Floor:
        context.getOutputValue()(i, j) = floor(context.getInputValue(0)(i, j));
        break;
      case Log:
        context.getOutputValue()(i, j) = log(context.getInputValue(0)(i, j));
        break;
      case Log10:
        context.getOutputValue()(i, j) = log10(context.getInputValue(0)(i, j));
        break;
      case Minus:
        context.getOutputValue()(i, j) = -context.getInputValue(0)(i, j);
        break;
      case Sin:
        context.getOutputValue()(i, j) = sin(context.getInputValue(0)(i, j));
        break;
      case Sqr:
        context.getOutputValue()(i, j)
          = context.getInputValue(0)(i, j)*context.getInputValue(0)(i, j);
        break;
      case Sqrt:
        context.getOutputValue()(i, j) = sqrt(context.getInputValue(0)(i, j));
        break;
      case Tan:
        context.getOutputValue()(i, j) = tan(context.getInputValue(0)(i, j));
        break;
      default:
        context.getOutputValue()(i, j) = context.getInputValue(0)(i, j);
        break;
      }
    }
  }
}

void
UnaryFunction::setType(const UnaryFunction::Type& type)
{
  mType = type;
}

const UnaryFunction::Type&
UnaryFunction::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
