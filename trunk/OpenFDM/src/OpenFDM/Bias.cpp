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

BEGIN_OPENFDM_OBJECT_DEF(Bias)
  END_OPENFDM_OBJECT_DEF

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

  mInputPort = getInputPort(0)->toMatrixPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of Bias model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  // Size compatibility check
  if (size(mInputPort.getMatrixValue()) != size(mBias)) {
    Log(Model, Error) << "Input port of \"" << getName() << "\", does not "
                      << "match the size of the bias property" << endl;
    return false;
  }
  mOutput.resize(mInputPort.getMatrixValue());

  return true;
}

void
Bias::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPort.isConnected());
  mOutput = mInputPort.getMatrixValue();
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
