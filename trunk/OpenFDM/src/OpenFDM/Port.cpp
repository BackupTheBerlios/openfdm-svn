/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Port.h"

#include <string>
#include <vector>
#include <algorithm>

#include "LogStream.h"
#include "Object.h"
#include "Variant.h"
#include "Model.h"

namespace OpenFDM {

Port::Port(Model* model) :
  mModel(model)
{
}

Port::~Port()
{
}

void
Port::invalidate()
{
  removeAllConnections();
  mModel = 0;
}

WeakPtr<Model>
Port::getModel() const
{
  return mModel;
}

} // namespace OpenFDM
