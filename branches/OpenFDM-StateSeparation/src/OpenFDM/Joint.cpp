/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Joint.h"

#include "Assert.h"
#include "ConstNodeVisitor.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "LogStream.h"
#include "PortValueList.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Joint, MechanicNode)
  END_OPENFDM_OBJECT_DEF

Joint::Joint(const std::string& name) :
  MechanicNode(name)
{
}

Joint::~Joint(void)
{
}

void
Joint::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Joint::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
