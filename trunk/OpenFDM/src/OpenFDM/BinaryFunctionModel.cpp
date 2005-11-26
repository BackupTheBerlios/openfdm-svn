/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "BinaryFunctionModel.h"

namespace OpenFDM {

BinaryFunctionModel::BinaryFunctionModel(const std::string& name,
                                         BinaryExpressionImpl<real_type>* expression) :
  Model(name),
  mBinaryExpression(expression)
{
  setDirectFeedThrough(true);

  setNumInputPorts(1);
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(expression));
}

BinaryFunctionModel::~BinaryFunctionModel(void)
{
}

bool
BinaryFunctionModel::init(void)
{
  OpenFDMAssert(mBinaryExpression->isValid());
  return mBinaryExpression->isValid();
}

void
BinaryFunctionModel::output(const TaskInfo&)
{
  // Evaluate the expression.
  mFunctionValue = mBinaryExpression->getValue();
}

const real_type&
BinaryFunctionModel::getFunctionValue(void) const
{
  return mFunctionValue;
}

void
BinaryFunctionModel::inputPortChanged(unsigned i)
{
  if (getNumInputPorts() <= i)
    return;
  mBinaryExpression->setInputProperty(i, getInputPort(i)->getProperty());
}

} // namespace OpenFDM
