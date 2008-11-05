/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RigidBody.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"
#include "PortValueList.h"

namespace OpenFDM {

class RigidBody::Context : public MechanicContext {
public:
  Context(const RigidBody* rigidBody) : mRigidBody(rigidBody) {}
  virtual ~Context() {}

  virtual const RigidBody& getNode() const
  { return *mRigidBody; }

  virtual bool alloc()
  { if (!allocStates()) return false; return mRigidBody->alloc(*this); }
  virtual void initVelocities(const /*Init*/Task& task)
  {
    mRigidBody->init(task, mDiscreteState, mContinousState, mPortValueList);
    mRigidBody->velocity(task, mContinousState, mPortValueList);
  }

  virtual void velocities(const Task& task)
  {
    mRigidBody->velocity(task, mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mRigidBody->articulation(task, mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mRigidBody->acceleration(task, mContinousState, mPortValueList);
  }

  virtual void derivative(const Task&)
  { }
  virtual void update(const DiscreteTask&)
  { }

private:
  SharedPtr<const RigidBody> mRigidBody;
};

BEGIN_OPENFDM_OBJECT_DEF(RigidBody, MechanicNode)
  END_OPENFDM_OBJECT_DEF

RigidBody::RigidBody(const std::string& name) :
  MechanicNode(name)
{
  addLink("link0");
  addLink("link1");
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

MechanicContext*
RigidBody::newMechanicContext() const
{
  return new Context(this);
}

PortId
RigidBody::addLink(const std::string& name)
{
  MechanicLink mechanicLink = newMechanicLink(name);
  mMechanicLinks.push_back(mechanicLink);
  return getPortId(mechanicLink.getPortIndex());
}

void
RigidBody::removeLink(const PortId& portId)
{
  MechanicLinkVector::iterator i = mMechanicLinks.begin();
  while (i != mMechanicLinks.end()) {
    if (getPortIndex(portId) == i->getPortIndex()) {
      i->clear();
      i = mMechanicLinks.erase(i);
    } else
      ++i;
  }
}

void
RigidBody::velocity(const Task&, const ContinousStateValueVector&,
                    PortValueList& portValues) const
{
  unsigned numLinkValues = mMechanicLinks.size();
  const MechanicLinkValue& parentLink = portValues[mMechanicLinks.front()];
  for (unsigned i = 1; i < numLinkValues; ++i)
    portValues[mMechanicLinks[i]].setPosAndVel(parentLink);
}

void
RigidBody::articulation(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues) const
{
  unsigned numLinkValues = mMechanicLinks.size();
  MechanicLinkValue& parentLink = portValues[mMechanicLinks.front()];
  for (unsigned i = 1; i < numLinkValues; ++i)
    parentLink.applyArticulation(portValues[mMechanicLinks[i]]);
}

void
RigidBody::acceleration(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues) const
{
  unsigned numLinkValues = mMechanicLinks.size();
  const MechanicLinkValue& parentLink = portValues[mMechanicLinks.front()];
  for (unsigned i = 1; i < numLinkValues; ++i)
    portValues[mMechanicLinks[i]].setAccel(parentLink);
}

} // namespace OpenFDM
