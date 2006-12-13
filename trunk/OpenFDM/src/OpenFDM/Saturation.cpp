/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Saturation.h"

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Saturation, Model)
  DEF_OPENFDM_PROPERTY(Matrix, MinSaturation, Serialized)
  DEF_OPENFDM_PROPERTY(Matrix, MaxSaturation, Serialized)
  END_OPENFDM_OBJECT_DEF

Saturation::Saturation(const std::string& name) : Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Saturation::getOutput);
}

Saturation::~Saturation(void)
{
}
  
bool
Saturation::init(void)
{
  mInputPort = getInputPort(0)->toMatrixPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of Saturation model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  Matrix inputMatrix = mInputPort.getMatrixValue();
  if (0 < rows(mMaxSaturation) && 0 < cols(mMaxSaturation) &&
      size(mMaxSaturation) != size(inputMatrix))
    mOutput.resize(0, 0);
  if (0 < rows(mMinSaturation) && 0 < cols(mMinSaturation) &&
      size(mMinSaturation) != size(inputMatrix))
    mOutput.resize(0, 0);
  
  mOutput.resize(inputMatrix);

  return Model::init();
}

void
Saturation::output(const TaskInfo&)
{
  mOutput = mInputPort.getMatrixValue();
  if (0 < rows(mMaxSaturation) && 0 < cols(mMaxSaturation))
    mOutput = LinAlg::min(mOutput, mMaxSaturation);
  if (0 < rows(mMinSaturation) && 0 < cols(mMinSaturation))
    mOutput = LinAlg::max(mOutput, mMinSaturation);
}

const Matrix&
Saturation::getMinSaturation(void) const
{
  return mMinSaturation;
}

void
Saturation::setMinSaturation(const Matrix& minSaturation)
{
  mMinSaturation = minSaturation;
}

const Matrix&
Saturation::getMaxSaturation(void) const
{
  return mMaxSaturation;
}

void
Saturation::setMaxSaturation(const Matrix& maxSaturation)
{
  mMaxSaturation = maxSaturation;
}

const Matrix&
Saturation::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
