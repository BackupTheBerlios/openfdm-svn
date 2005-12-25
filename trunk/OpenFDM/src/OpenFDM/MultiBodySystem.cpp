/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Visitor.h"
#include "ModelVisitor.h"
#include "Mass.h"
#include "Force.h"
#include "MultiBodySystem.h"

namespace OpenFDM {

class ForwardDynamicsVisitor
  : public Visitor {
public:
  virtual void apply(RigidBody& body)
  {
    // Note the order. First compute the articulated values on each child.
    traverse(body);
    // Past that, do it on this current rigid body.
    body.computeArtValues();
  }
};

class AccelerationPropagationVisitor
  : public Visitor {
public:
  virtual void apply(RigidBody& body)
  {
    body.computeAccel();
    // Note the order. First compute the acceleration and than traverse
    // to the children.
    traverse(body);
  }
};

MultiBodySystem::MultiBodySystem(RootFrame* rootFrame) :
  ModelGroup("multibodymodel"),
  mRootFrame(rootFrame)
{
  // FIXME
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);
}

MultiBodySystem::~MultiBodySystem(void)
{
}

void
MultiBodySystem::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

// bool
// MultiBodySystem::init(void)
// {
//   StateCountVisitor gsc;
//   mRootFrame->accept(gsc);
//   setNumContinousStates(gsc.getStateCount());
//   return ModelGroup::init();
// }

void
MultiBodySystem::output(const TaskInfo& taskInfo)
{
  // Hmm, just works now ... FIXME
  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    if (!(*it)->getMultiBodyAcceleration())
      (*it)->output(taskInfo);
  }

  // Compute forward dynamics, that is the articulated forces and inertia.
  ForwardDynamicsVisitor fwdVisitor;
  mRootFrame->accept(fwdVisitor);

  // Then compute the articulated inertias and forces.
  AccelerationPropagationVisitor apVisitor;
  mRootFrame->accept(apVisitor);

  // Hmm, just works now ... FIXME
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    if ((*it)->getMultiBodyAcceleration())
      (*it)->output(taskInfo);
  }
}

void
MultiBodySystem::addRigidBody(RigidBody* rigidBody)
{
  if (!rigidBody)
    return;
  RigidBodyList::iterator it = mRigidBodies.begin();
  while (it != mRigidBodies.end()) {
    if ((*it) == rigidBody)
      return;
    ++it;
  }
  mRigidBodies.push_back(rigidBody);
  rigidBody->setParentMultiBodySystem(this);
}

void
MultiBodySystem::removeRigidBody(RigidBody* rigidBody)
{
  RigidBodyList::iterator it = mRigidBodies.begin();
  while (it != mRigidBodies.end()) {
    if ((*it) == rigidBody) {
      it = mRigidBodies.erase(it);
      rigidBody->setParentMultiBodySystem(0);
    }
    else
      ++it;
  }
}

void
MultiBodySystem::addInteract(Interact* interact)
{
  if (!interact)
    return;
  /// Already in the list, might be already attached to an other rigid body
  if (this == interact->getParent())
    return;
  addModel(interact);
}

void
MultiBodySystem::removeInteract(Interact* interact)
{
  if (!interact)
    return;
  if (this != interact->getParent())
    return;
  removeModel(interact);
}

} // namespace OpenFDM
