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
  // Make sure it is invalid if sizes do not match.
  mMax.resize(0, 0);

  mInputPorts.clear();
  unsigned n = getNumInputPorts();
  if (n == 0) {
      Log(Model, Error) << "No input ports for MaxModel \""
                        << getName() << "\"" << endl;
      return false;
  }
  for (unsigned i = 0; i < n; ++i) {
    MatrixPortHandle matrixPort = getInputPort(i)->toMatrixPortHandle();
    if (!matrixPort.isConnected()) {
      Log(Model, Error) << "Found unconnected input Port for MaxModel \""
                        << getName() << "\"" << endl;
      return false;
    }
    mInputPorts.push_back(matrixPort);

    Matrix a = matrixPort.getMatrixValue();
    if (i == 0) {
      mMax.resize(a);
    } else {
      if (size(mMax) != size(a)) {
        Log(Model, Error) << "Input port sizes for MaxModel \""
                          << getName() << "\" do not match." << endl;
        return false;
      }
    }
  }

  return Model::init();
}

void
MaxModel::output(const TaskInfo&)
{
  // the input method guarantees that there is at least one input
  mMax = mInputPorts[0].getMatrixValue();
  for (unsigned i = 1; i < mInputPorts.size(); ++i)
    mMax = LinAlg::max(mMax, mInputPorts[i].getMatrixValue());
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
