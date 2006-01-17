/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "ConstModel.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ConstModel)
  END_OPENFDM_OBJECT_DEF

ConstModel::ConstModel(const std::string& name, const Matrix& value) :
  Model(name), mValue(value)
{
  addProperty("value", Property(this, &ConstModel::getValue, &ConstModel::setValue));

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

} // namespace OpenFDM
