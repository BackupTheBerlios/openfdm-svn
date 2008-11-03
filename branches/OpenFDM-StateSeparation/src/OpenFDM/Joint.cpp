/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Joint.h"

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "LogStream.h"
#include "PortValueList.h"
#include "MechanicContext.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Joint, Interact)
  END_OPENFDM_OBJECT_DEF

Joint::Joint(const std::string& name) :
  Interact(name),
  mParentLink(newMechanicLink("link0")),
  mChildLink(newMechanicLink("link1"))
{
}

Joint::~Joint(void)
{
}

void
Joint::velocity(const Task&,
                const ContinousStateValueVector& continousState,
                PortValueList& portValues) const
{
  velocity(portValues[mParentLink], portValues[mChildLink],
           continousState, portValues);
}

void
Joint::articulation(const Task&,
                    const ContinousStateValueVector& continousState,
                    PortValueList& portValues, FrameData& frameData) const
{
  articulation(portValues[mParentLink], portValues[mChildLink],
               continousState, portValues, frameData);
}

void
Joint::acceleration(const Task&,
                    const ContinousStateValueVector& continousState,
                    PortValueList& portValues, FrameData& frameData) const
{
  acceleration(portValues[mParentLink], portValues[mChildLink],
               continousState, portValues, frameData);
}

} // namespace OpenFDM
