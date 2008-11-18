/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "UnaryModel.h"

#include "LogStream.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(UnaryModel, AbstractModel)
  END_OPENFDM_OBJECT_DEF

UnaryModel::UnaryModel(const std::string& name) :
  AbstractModel(name),
  mInputPort(new InputPortInfo(this, "input", Size(0, 0), true)),
  mOutputPort(new OutputPortInfo(this, "output", Size(0, 0), false))
{
}

UnaryModel::~UnaryModel()
{
}

bool
UnaryModel::alloc(LeafContext& leafContext) const
{
  Size sz;
  sz = size(leafContext.getPortValue(*mInputPort)->getValue());
  Log(Initialization, Debug)
    << "Size for Gain is detemined by the input port with size: "
    << trans(sz) << std::endl;
  if (!leafContext.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for output port does not match!" << std::endl;
    return false;
  }
  return true;
}

} // namespace OpenFDM
