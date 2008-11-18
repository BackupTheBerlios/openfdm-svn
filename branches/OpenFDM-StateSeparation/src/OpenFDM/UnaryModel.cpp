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

} // namespace OpenFDM
