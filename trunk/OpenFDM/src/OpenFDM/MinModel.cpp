/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "Model.h"
#include "MinModel.h"

namespace OpenFDM {

MinModel::MinModel(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "Input 0");
  setInputPortName(1, "Input 1");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &MinModel::getMin));
  addProperty("output", Property(this, &MinModel::getMin));

  addProperty("numMinInputs", Property(this, &MinModel::getNumMinInputs, &MinModel::setNumMinInputs));
}

MinModel::~MinModel(void)
{
}

bool
MinModel::init(void)
{
  for (unsigned i = 0; i < getNumInputPorts(); ++i)
    OpenFDMAssert(getInputPort(i)->isConnected());
  
  // Make sure it is invalid if sizes do not match.
  mMin.resize(0, 0);
  // Check if the sizes match.
  Matrix a0 = getInputPort(0)->getValue().toMatrix();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    Matrix a = getInputPort(i)->getValue().toMatrix();
    if (size(a0) != size(a))
      return false;
  }
  mMin.resize(a0);
  return true;
}

void
MinModel::output(const TaskInfo&)
{
  mMin = getInputPort(0)->getValue().toMatrix();
  for (unsigned i = 1; i < getNumInputPorts(); ++i)
    mMin = LinAlg::min(mMin, getInputPort(i)->getValue().toMatrix());
}

const Matrix&
MinModel::getMin(void) const
{
  return mMin;
}

unsigned
MinModel::getNumMinInputs(void) const
{
  return getNumInputPorts();
}

void
MinModel::setNumMinInputs(const unsigned& num)
{
  setNumInputPorts(num);
}

} // namespace OpenFDM
