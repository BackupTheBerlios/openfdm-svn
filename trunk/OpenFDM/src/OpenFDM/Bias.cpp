/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>
#include <vector>

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"
#include "Model.h"
#include "Bias.h"

namespace OpenFDM {

Bias::Bias(const std::string& name) : Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Bias::getOutput);

  addProperty("output", Property(this, &Bias::getOutput));
  addProperty("bias", Property(this, &Bias::getBias, &Bias::setBias));
}

Bias::~Bias(void)
{
}
  
bool
Bias::init(void)
{
  // Invalidate outputs
  mOutput.resize(0, 0);
  
  // Minimal check of the input port 
  if (!getInputPort(0)->isConnected()) {
    Log(Model, Error) << "Input port of \"" << getName()
                   << "\", is not valid" << endl;
    return false;
  }

  // Size compatibility check
  if (size(getInputPort(0)->getValue().toMatrix()) != size(mBias)) {
    Log(Model, Error) << "Input port of \"" << getName() << "\", does not "
                      << "match the size of the bias property" << endl;
    return false;
  }

  // Make sure it is invalid if sizes do not match.
  mOutput.resize(getInputPort(0)->getValue().toMatrix());
  return true;
}

void
Bias::output(const TaskInfo&)
{
  OpenFDMAssert(getInputPort(0)->isConnected());
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  mOutput = mh.getMatrixValue();
  mOutput += mBias;
}

const Matrix&
Bias::getBias(void) const
{
  return mBias;
}

void
Bias::setBias(const Matrix& bias)
{
  mBias = bias;
}

const Matrix&
Bias::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
