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
#include "ConstVisitor.h"
#include "ModelVisitor.h"
#include "Mass.h"
#include "Force.h"
#include "MultiBodySystem.h"

namespace OpenFDM {

MultiBodySystem::MultiBodySystem(const std::string& name) :
  ModelGroup(name)
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

bool
MultiBodySystem::init(void)
{
  return ModelGroup::init();
}

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
  mFreeJoint->getOutboardBody()->computeArtValues();
  // Then compute the articulated inertias and forces.
  mFreeJoint->updateAccels(0);

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
  // FIXME incorporate that somehow into the depencencies ...
  if (dynamic_cast<FreeJoint*>(interact)) {
    mFreeJoint = dynamic_cast<FreeJoint*>(interact);
  }
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
