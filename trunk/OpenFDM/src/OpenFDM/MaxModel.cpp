/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Model.h"
#include "MaxModel.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MaxModel, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, NumMaxInputs, Serialized)
  END_OPENFDM_OBJECT_DEF

MaxModel::MaxModel(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "Input 0");
  setInputPortName(1, "Input 1");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &MaxModel::getMax);
}

MaxModel::~MaxModel(void)
{
}

bool
MaxModel::init(void)
{
  for (unsigned i = 0; i < getNumInputPorts(); ++i)
    OpenFDMAssert(getInputPort(i)->isConnected());
  
  // Make sure it is invalid if sizes do not match.
  mMax.resize(0, 0);
  // Check if the sizes match.
  Matrix a0 = getInputPort(0)->getValue().toMatrix();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    Matrix a = getInputPort(i)->getValue().toMatrix();
    if (size(a0) != size(a))
      return false;
  }
  mMax.resize(a0);
  return Model::init();
}

void
MaxModel::output(const TaskInfo&)
{
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  mMax = mh.getMatrixValue();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    MatrixPortHandle mh = getInputPort(i)->toMatrixPortHandle();
    mMax = LinAlg::max(mMax, mh.getMatrixValue());
  }
}

const Matrix&
MaxModel::getMax(void) const
{
  return mMax;
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
