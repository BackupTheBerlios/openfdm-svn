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

BEGIN_OPENFDM_OBJECT_DEF(Bias, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Matrix, Bias, Serialized)
  END_OPENFDM_OBJECT_DEF

Bias::Bias(const std::string& name, const real_type& bias) :
  SimpleDirectModel(name)
{
  setBias(bias);
  addInputPort("input");
}

Bias::~Bias(void)
{
}
  
void
Bias::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  // FIXME: can we check that in advance???
  OpenFDMAssert(sz == size(mBias));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      context.getOutputValue()(j, k)
        = mBias(j, k) + context.getInputValue(0)(j, k);
    }
  }
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
