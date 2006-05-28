/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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
#include "ModelGroup.h"
#include "RigidBody.h"

namespace OpenFDM {

RigidBody::RigidBody(const std::string& name) :
  Model(name)
{
}

RigidBody::~RigidBody(void)
{
}

void
RigidBody::output(const TaskInfo& taskInfo)
{
  Log(ArtBody, Debug) << "Entry of computeArtValues of \"" << getName()
                      << "\"" << endl;

  // At first this is the inertia matrix of the current body.
  mArtInertia.clear();
  mArtForce.clear();

  // Collect all articulated forces and inertias
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it)
    (*it)->interactWith(this);
  
  Log(ArtBody, Debug3) << "On exit of computeArtValues of \"" << getName()
                       << "\" Force is:\n" << trans(mArtForce)
                       << "\nInertia:\n" << mArtInertia << endl;
}

bool
RigidBody::dependsDirectOn(Model* model)
{
  InteractList::const_iterator i;
  for (i = mInteracts.begin(); i != mInteracts.end(); ++i)
    if ((*i) == model)
      return true;
  return Model::dependsDirectOn(model);
}

bool
RigidBody::setInboardJoint(Joint* joint)
{
  if (!joint->attachTo(this, true))
    return false;
  mInboardJoint = joint;
  return true;
}

bool
RigidBody::addInteract(Interact* interact)
{
  if (!interact->attachTo(this, false))
    return false;
  mInteracts.push_back(interact);
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
