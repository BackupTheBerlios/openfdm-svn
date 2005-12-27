/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Frame.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "Frame.h"
#include "LogStream.h"
#include "Joint.h"

namespace OpenFDM {

Joint::Joint(const std::string& name)
  : Interact(name, 2)
{
}

Joint::~Joint(void)
{
}

} // namespace OpenFDM
