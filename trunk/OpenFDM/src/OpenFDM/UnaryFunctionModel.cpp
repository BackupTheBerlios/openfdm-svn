/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "UnaryFunctionModel.h"

namespace OpenFDM {

UnaryFunctionModel::UnaryFunctionModel(const std::string& name,
                                       UnaryExpressionImpl<real_type>* expression) :
  Model(name),
  mUnaryExpression(expression)
{
  setDirectFeedThrough(true);

  setNumInputPorts(1);
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &UnaryFunctionModel::getFunctionValue);
}

UnaryFunctionModel::~UnaryFunctionModel(void)
{
}

bool
UnaryFunctionModel::init(void)
{
  OpenFDMAssert(mUnaryExpression);
  mUnaryExpression->setInputProperty(getInputPort(0)->getProperty());
  OpenFDMAssert(mUnaryExpression->isValid());
  return mUnaryExpression->isValid();
}

void
UnaryFunctionModel::output(const TaskInfo&)
{
  // Evaluate the expression.
  mFunctionValue = mUnaryExpression->getValue();
}

const real_type&
UnaryFunctionModel::getFunctionValue(void) const
{
  return mFunctionValue;
}

} // namespace OpenFDM
