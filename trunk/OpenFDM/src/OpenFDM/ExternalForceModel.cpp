/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Vector.h"
#include "ExternalForceModel.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ExternalForceModel)
  END_OPENFDM_OBJECT_DEF

ExternalForceModel::ExternalForceModel(const std::string& name)
  : ExternalForce(name)
{
  setDirectFeedThrough(true);

  setNumInputPorts(1);
  setInputPortName(0, "forceInput");
}

ExternalForceModel::~ExternalForceModel(void)
{
}

bool
ExternalForceModel::init(void)
{
  mInputPort = getInputPort(0)->toMatrixPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of ExternalForceModel model \""
                      << getName() << "\" failed: Input port \""
                      << getInputPortName(0) << "\" is not connected!" << endl;
    return false;
  }
  if (size(mInputPort.getMatrixValue()) != Size(6, 1)) {
    Log(Model, Error) << "Initialization of ExternalForceModel model \""
                      << getName() << "\" failed: Input port \""
                      << getInputPortName(0) << "\" is not of size [6, 1]!"
                      << endl;
    return false;
  }

  return true;
}

void
ExternalForceModel::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPort.isConnected());
  setForce(mInputPort.getMatrixValue());
}

} // namespace OpenFDM
