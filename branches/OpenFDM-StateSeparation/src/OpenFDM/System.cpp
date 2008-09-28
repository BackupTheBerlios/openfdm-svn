/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "System.h"

#include "Object.h"
#include "AbstractSystem.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(System, Object)
  END_OPENFDM_OBJECT_DEF

System::System(const std::string& name, Node* node) :
  Object(name),
  mNode(node)
{
}

System::~System()
{
}

void
System::setNode(Node* node)
{
  clear();
  mNode = node;
}

/// FIXME: here should System::init be

void
System::clear()
{
  mAbstractSystem = 0;
  mNodeInstanceList.clear();
}

/// Simulate the system until the time tEnd
bool
System::simulate(const real_type& t)
{
  if (mAbstractSystem)
    return false;
  mAbstractSystem->outputAt(t);
  return true;
}

/// Bring the system in an equilibrum state near the current state ...
bool
System::trim(void)
{
  return false;
}

/// Return the current simulation time, convenience function
real_type
System::getTime(void) const
{
  if (!mAbstractSystem)
    return Limits<real_type>::quiet_NaN();
  return mAbstractSystem->getTime();
}

} // namespace OpenFDM
