/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Summer.h"

#include <string>
#include "Types.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Summer, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, NumSummands, Serialized)
  END_OPENFDM_OBJECT_DEF

Summer::Summer(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "+");
  setInputPortName(1, "+");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Summer::getSum);
}

Summer::~Summer(void)
{
}

bool
Summer::init(void)
{
  // Make sure it is invalid if sizes do not match.
  mSum.resize(0, 0);

  mPositiveSummandPorts.clear();
  mNegativeSummandPorts.clear();
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    MatrixPortHandle matrixPort = getInputPort(i)->toMatrixPortHandle();
    if (!matrixPort.isConnected()) {
      Log(Model, Error) << "Found unconnected input Port for Summer \""
                        << getName() << "\"" << endl;
      return false;
    }
    if (getInputPortName(i) == "-") {
      mNegativeSummandPorts.push_back(matrixPort);
    } else {
      mPositiveSummandPorts.push_back(matrixPort);
    }

    Matrix a = matrixPort.getMatrixValue();
    if (i == 0) {
      mSum.resize(a);
    } else {
      if (size(mSum) != size(a)) {
        Log(Model, Error) << "Input port sizes for Summer \""
                          << getName() << "\" do not match." << endl;
        return false;
      }
    }
  }

  return Model::init();
}

void
Summer::output(const TaskInfo&)
{
  mSum.clear();

  std::vector<MatrixPortHandle>::iterator it = mNegativeSummandPorts.begin();
  while (it != mNegativeSummandPorts.end()) {
    mSum -= (*it).getMatrixValue();
    ++it;
  }
  it = mPositiveSummandPorts.begin();
  while (it != mPositiveSummandPorts.end()) {
    mSum += (*it).getMatrixValue();
    ++it;
  }

  Log(Model,Debug3) << "Output of Summer \"" << getName() << "\" "
                    << mSum << endl;
}

const Matrix&
Summer::getSum(void) const
{
  return mSum;
}

unsigned
Summer::getNumSummands(void) const
{
  return getNumInputPorts();
}

void
Summer::setNumSummands(unsigned num)
{
  unsigned oldnum = getNumSummands();
  setNumInputPorts(num);
  for (; oldnum < num; ++oldnum)
    setInputPortName(oldnum, "+");
}

void
Summer::setInputSign(unsigned num, Sign sign)
{
  if (getNumSummands() <= num)
    return;
  if (sign == Minus)
    setInputPortName(num, "-");
  else
    setInputPortName(num, "+");
}

Summer::Sign
Summer::getInputSign(unsigned num) const
{
  if (getNumSummands() <= num)
    return Plus;
  if (getInputPortName(num) == "-")
    return Minus;
  else
    return Plus;
}

} // namespace OpenFDM
