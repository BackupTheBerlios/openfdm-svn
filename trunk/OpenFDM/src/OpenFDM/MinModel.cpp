/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Model.h"
#include "MinModel.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MinModel, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, NumMinInputs, Serialized)
  END_OPENFDM_OBJECT_DEF

MinModel::MinModel(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "Input 0");
  setInputPortName(1, "Input 1");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &MinModel::getMin);
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
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  mMin = mh.getMatrixValue();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    MatrixPortHandle mh = getInputPort(i)->toMatrixPortHandle();
    mMin = LinAlg::min(mMin, mh.getMatrixValue());
  }
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
MinModel::setNumMinInputs(unsigned num)
{
  setNumInputPorts(num);
}

} // namespace OpenFDM
