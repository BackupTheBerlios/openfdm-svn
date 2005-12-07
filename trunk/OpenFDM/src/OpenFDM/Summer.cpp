/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "Model.h"
#include "Summer.h"

namespace OpenFDM {

Summer::Summer(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "+");
  setInputPortName(1, "+");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Summer::getSum);

  addProperty("output", Property(this, &Summer::getSum));

  addProperty("numSummands", Property(this, &Summer::getNumSummands, &Summer::setNumSummands));
}

Summer::~Summer(void)
{
}

bool
Summer::init(void)
{
  for (unsigned i = 0; i < getNumInputPorts(); ++i)
    OpenFDMAssert(getInputPort(i)->isConnected());
  
  // Make sure it is invalid if sizes do not match.
  mSum.resize(0, 0);
  // Check if the sizes match.
  Matrix a0 = getInputPort(0)->getValue().toMatrix();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    Matrix a = getInputPort(i)->getValue().toMatrix();
    if (size(a0) != size(a))
      return false;
  }
  mSum.resize(a0);
  return true;
}

void
Summer::output(const TaskInfo&)
{
  mSum.clear();
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    /// FIXME could be preevaluated
    MatrixPortHandle ph = getInputPort(i)->toMatrixPortHandle();
    if (getInputPortName(i) == "-")
      mSum -= ph.getMatrixValue();
    else
      mSum += ph.getMatrixValue();
  }
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
Summer::setNumSummands(const unsigned& num)
{
  unsigned oldnum = getNumSummands();
  setNumInputPorts(num);
  for (; oldnum < num; ++oldnum)
    setInputPortName(oldnum, "+");
}

} // namespace OpenFDM
