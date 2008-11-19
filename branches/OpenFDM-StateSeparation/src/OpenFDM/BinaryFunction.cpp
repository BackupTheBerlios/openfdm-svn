/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "BinaryFunction.h"

#include "ModelContext.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(BinaryFunction, Model)
  END_OPENFDM_OBJECT_DEF

BinaryFunction::BinaryFunction(const std::string& name, Type type) :
  Model(name),
  mInput0Port(newMatrixInputPort("input0", true)),
  mInput1Port(newMatrixInputPort("input1", true)),
  mOutputPort(newMatrixOutputPort("output"))
{
}

BinaryFunction::~BinaryFunction(void)
{
}

bool
BinaryFunction::alloc(ModelContext& context) const
{
  Size sz = size(context.getPortValueList()[mInput0Port]);
  Log(Initialization, Debug)
    << "Size for BinaryFunction is detemined by input0 "
    << "port with size: " << trans(sz) << std::endl;
  if (!context.getPortValueList().setOrCheckPortSize(mInput1Port, sz)) {
    Log(Initialization, Error)
      << "Size for input1 port does not match!" << std::endl;
    return false;
  }
  if (!context.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for output port does not match!" << std::endl;
    return false;
  }
  return true;
}

void
BinaryFunction::output(const Task&,const DiscreteStateValueVector&,
                       const ContinousStateValueVector&,
                       PortValueList& portValues) const
{
  Size sz = size(portValues[mOutputPort]);
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      switch (mType) {
      case Atan2:
        portValues[mOutputPort](i, j) =
          atan2(portValues[mInput0Port](i, j), portValues[mInput1Port](i, j));
        break;
      case Pow:
        portValues[mOutputPort](i, j) =
          pow(portValues[mInput0Port](i, j), portValues[mInput1Port](i, j));
        break;
      case Div:
        portValues[mOutputPort](i, j) =
          portValues[mInput0Port](i, j) / portValues[mInput1Port](i, j);
        break;
      default:
        OpenFDMAssert(false);
        break;
      }
    }
  }
}

void
BinaryFunction::setType(const BinaryFunction::Type& type)
{
  mType = type;
}

const BinaryFunction::Type&
BinaryFunction::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
