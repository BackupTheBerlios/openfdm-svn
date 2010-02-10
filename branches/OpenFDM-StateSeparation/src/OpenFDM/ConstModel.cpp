/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "ConstModel.h"

#include "PortValueList.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ConstModel, Model)
  DEF_OPENFDM_PROPERTY(Matrix, Value, Serialized)
  END_OPENFDM_OBJECT_DEF

ConstModel::ConstModel(const std::string& name, const Matrix& value) :
  Model(name),
  mOutputPort(newMatrixOutputPort("output")),
  mValue(value)
{
}

ConstModel::ConstModel(const std::string& name, const real_type& value) :
  Model(name),
  mOutputPort(newMatrixOutputPort("output"))
{
  setScalarValue(value);
}

ConstModel::~ConstModel(void)
{
}

bool
ConstModel::alloc(ModelContext& context) const
{
  Size sz = size(mValue);
  Log(Initialization, Debug)
    << "Size for ConstModel is detemined by the value "
    << "with size: " << trans(sz) << std::endl;
  if (!context.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for output port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
ConstModel::output(const Task& ,const DiscreteStateValueVector&,
                   const ContinousStateValueVector&,
                   PortValueList& portValues) const
{
  portValues[mOutputPort] = mValue;
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
ConstModel::setScalarValue(const real_type& value)
{
  mValue.resize(1, 1);
  mValue(0, 0) = value;
}

} // namespace OpenFDM
