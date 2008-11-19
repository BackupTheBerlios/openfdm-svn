/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Bias.h"

#include <string>
#include <vector>
#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Bias, UnaryModel)
  DEF_OPENFDM_PROPERTY(Matrix, Bias, Serialized)
  END_OPENFDM_OBJECT_DEF

Bias::Bias(const std::string& name, const real_type& bias) :
  UnaryModel(name)
{
  setBias(bias);
}

Bias::~Bias(void)
{
}
  
ModelContext*
Bias::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
Bias::output(const Matrix& inputValue, Matrix& outputValue) const
{
  outputValue = mBias + inputValue;
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

void
Bias::setBias(const real_type& bias)
{
  mBias.resize(1, 1);
  mBias(0, 0) = bias;
}

} // namespace OpenFDM
