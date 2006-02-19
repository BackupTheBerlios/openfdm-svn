/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Model.h"
#include "Summer.h"

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
    if (!getInputPort(i)->isConnected()) {
      Log(Model, Error) << "Found unconnected input Port for Summer \""
                        << getName() << "\"" << endl;
      return false;
    }
    if (getInputPortName(i) == "-") {
      mNegativeSummandPorts.push_back(getInputPort(i)->toMatrixPortHandle());
    } else {
      mPositiveSummandPorts.push_back(getInputPort(i)->toMatrixPortHandle());
    }

    Matrix a = getInputPort(i)->getValue().toMatrix();
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
  return true;
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

} // namespace OpenFDM
