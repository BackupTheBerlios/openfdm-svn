/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "Force.h"
#include "Mass.h"
#include "Joint.h"
#include "MultiBodySystem.h"
#include "RigidBody.h"

namespace OpenFDM {

RigidBody::RigidBody(const std::string& name) :
  Object(name)
{
}

RigidBody::~RigidBody(void)
{
}

void
RigidBody::computeArtValues(void)
{
  Log(ArtBody, Debug) << "Entry of computeArtValues of \"" << getName()
                      << "\"" << endl;

  // At first this is the inertia matrix of the current body.
  mArtInertia = SpatialInertia::zeros();
  mArtForce = Vector6::zeros();

  // Collect all articulated forces and inertias
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it)
    (*it)->interactWith(this);
  
  Log(ArtBody, Debug3) << "On exit of computeArtValues of \"" << getName()
                       << "\" Force is:\n" << trans(mArtForce)
                       << "\nInertia:\n" << mArtInertia << endl;
}

void
RigidBody::computeAccel(void)
{
  Log(ArtBody, Debug) << "Entry of computeAccel of \"" << getName()
                      << "\"" << endl;

  // Update all accelerations, Hmm, is a bit too croase that way ...
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it) {
    (*it)->updateAccels(this);
  }

  Log(ArtBody, Debug3) << "On exit of computeAccel of \"" << getName()
                       << "\"" << endl;
}

void
RigidBody::setParentMultiBodySystem(MultiBodySystem* multiBodySystem)
{
  /// FIXME: rethink that ...
  mParentMultiBodySystem = multiBodySystem;
  if (!multiBodySystem)
    return;
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it) {
    multiBodySystem->addInteract(*it);
  }
  if (mInboardJoint)
    multiBodySystem->addInteract(mInboardJoint);
}

MultiBodySystem*
RigidBody::getParentMultiBodySystem(void)
{
  return mParentMultiBodySystem;
}

bool
RigidBody::setInboardJoint(Joint* joint)
{
  mInboardJoint = joint;
  if (!joint->attachTo(this, true))
    return false;
  if (!mParentMultiBodySystem)
    return true;
  mParentMultiBodySystem->addInteract(joint);
  return true;
}

bool
RigidBody::addInteract(Interact* interact)
{
  mInteracts.push_back(interact);
  if (!interact->attachTo(this, false))
    return false;
  if (!mParentMultiBodySystem)
    return true;
  mParentMultiBodySystem->addInteract(interact);
  return true;
}

bool
RigidBody::removeInteract(Interact* interact)
{
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it) {
    if ((*it) == interact) {
      mInteracts.erase(it);
      if (!interact->detachFrom(this))
        return false;
      return true;
    }
  }
  return false;
}

} // namespace OpenFDM
