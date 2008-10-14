/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RigidBody.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RigidBody, MechanicNode)
  END_OPENFDM_OBJECT_DEF

RigidBody::RigidBody(const std::string& name) :
  MechanicNode(name)
{
  mMechanicLinks.push_back(newMechanicLink("link"));
}

RigidBody::~RigidBody()
{
}

void
RigidBody::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
RigidBody::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

void
RigidBody::velocity(const Task&, const ContinousStateValueVector&,
                    PortValueList& portValues, MechanicContext&) const
{
  unsigned numLinkValues = mMechanicLinks.size();

  Vector3 position = portValues[mMechanicLinks.front()].mPosition;
  Quaternion orientation = portValues[mMechanicLinks.front()].mOrientation;
  Vector6 velocity = portValues[mMechanicLinks.front()].mSpatialVelocity;

  for (unsigned i = 1; i < numLinkValues; ++i) {
    portValues[mMechanicLinks[i]].mPosition = position;
    portValues[mMechanicLinks[i]].mOrientation = orientation;
    portValues[mMechanicLinks[i]].mSpatialVelocity = velocity;
  }
}

void
RigidBody::articulation(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues, MechanicContext&) const
{
  unsigned numLinkValues = mMechanicLinks.size();

  SpatialInertia inertia(InertiaMatrix(1, 0, 0, 1, 0, 1), 1);
  Vector6 force = Vector6::zeros();

  for (unsigned i = 1; i < numLinkValues; ++i) {
    inertia += portValues[mMechanicLinks[i]].mArticulatedInertia;
    force += portValues[mMechanicLinks[i]].mArticulatedForce;
  }

  portValues[mMechanicLinks.front()].mArticulatedInertia = inertia;
  portValues[mMechanicLinks.front()].mArticulatedForce = force;
}

void
RigidBody::acceleration(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues, MechanicContext&) const
{
  unsigned numLinkValues = mMechanicLinks.size();
  Vector6 accel = portValues[mMechanicLinks.front()].mSpatialAcceleration;
  for (unsigned i = 1; i < numLinkValues; ++i)
    portValues[mMechanicLinks[i]].mSpatialAcceleration = accel;
}

} // namespace OpenFDM
