/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"
#include "Matrix.h"
#include "Model.h"
#include "Saturation.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Saturation)
  END_OPENFDM_OBJECT_DEF

Saturation::Saturation(const std::string& name) : Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Saturation::getOutput);

  addProperty("output", Property(this, &Saturation::getOutput));

  addProperty("minSaturation", Property(this, &Saturation::getMinSaturation, &Saturation::setMinSaturation));
  addProperty("maxSaturation", Property(this, &Saturation::getMaxSaturation, &Saturation::setMaxSaturation));
}

Saturation::~Saturation(void)
{
}
  
bool
Saturation::init(void)
{
  OpenFDMAssert(getInputPort(0)->isConnected());
  
  Matrix inputMatrix = getInputPort(0)->getValue().toMatrix();
  if (0 < rows(mMaxSaturation) && 0 < cols(mMaxSaturation) &&
      size(mMaxSaturation) != size(inputMatrix))
    mOutput.resize(0, 0);
  if (0 < rows(mMinSaturation) && 0 < cols(mMinSaturation) &&
      size(mMinSaturation) != size(inputMatrix))
    mOutput.resize(0, 0);
  
  mOutput.resize(inputMatrix);
  return true;
}

void
Saturation::output(const TaskInfo&)
{
  OpenFDMAssert(getInputPort(0)->isConnected());
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  mOutput = mh.getMatrixValue();
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
