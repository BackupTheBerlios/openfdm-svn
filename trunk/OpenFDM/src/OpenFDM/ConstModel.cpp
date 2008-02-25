/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "ConstModel.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ConstModel, Model)
  DEF_OPENFDM_PROPERTY(Matrix, Value, Serialized)
  END_OPENFDM_OBJECT_DEF

ConstModel::ConstModel(const std::string& name, const Matrix& value) :
  Model(name), mValue(value)
{
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &ConstModel::getValue);
}

ConstModel::~ConstModel(void)
{
}

const Matrix&
ConstModel::getValue(void) const
{
  return mValue;
}

void
ConstModel::setValue(const Matrix& value)
{
  mValue = value;
}

void
ConstModel::setScalarValue(real_type value)
{
  mValue.resize(1, 1);
  mValue(1, 1) = value;
}

} // namespace OpenFDM
