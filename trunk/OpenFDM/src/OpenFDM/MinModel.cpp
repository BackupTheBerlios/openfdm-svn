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
  // Make sure it is invalid if sizes do not match.
  mMin.resize(0, 0);

  mInputPorts.clear();

  unsigned n = getNumInputPorts();
  if (n == 0) {
      Log(Model, Error) << "No input ports for MinModel \""
                        << getName() << "\"" << endl;
      return false;
  }
  for (unsigned i = 0; i < n; ++i) {
    MatrixPortHandle matrixPort = getInputPort(i)->toMatrixPortHandle();
    if (!matrixPort.isConnected()) {
      Log(Model, Error) << "Found unconnected input Port for MinModel \""
                        << getName() << "\"" << endl;
      return false;
    }
    mInputPorts.push_back(matrixPort);

    Matrix a = matrixPort.getMatrixValue();
    if (i == 0) {
      mMin.resize(a);
    } else {
      if (size(mMin) != size(a)) {
        Log(Model, Error) << "Input port sizes for MinModel \""
                          << getName() << "\" do not match." << endl;
        return false;
      }
    }
  }

  return Model::init();
}

void
MinModel::output(const TaskInfo&)
{
  // the input method guarantees that there is at least one input
  mMin = mInputPorts[0].getMatrixValue();
  for (unsigned i = 1; i < mInputPorts.size(); ++i)
    mMin = LinAlg::min(mMin, mInputPorts[i].getMatrixValue());
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
