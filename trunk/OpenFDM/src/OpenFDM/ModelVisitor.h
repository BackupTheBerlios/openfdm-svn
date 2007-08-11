/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelVisitor_H
#define OpenFDM_ModelVisitor_H

#include "Model.h"
#include "ModelGroup.h"
#include "RigidBody.h"
#include "MobileRootJoint.h"
#include "System.h"

namespace OpenFDM {

class ModelVisitor {
public:
  virtual ~ModelVisitor(void)
  { }
  virtual void apply(Model& model)
  { }
  virtual void apply(ModelGroup& modelGroup)
  { apply(static_cast<Model&>(modelGroup)); }
  virtual void apply(System& system)
  { apply(static_cast<ModelGroup&>(system)); }
  virtual void apply(Interact& interact)
  { apply(static_cast<Model&>(interact)); }
  virtual void apply(Joint& joint)
  { apply(static_cast<Interact&>(joint)); }
  virtual void apply(MobileRootJoint& mobileRootJoint)
  { apply(static_cast<Joint&>(mobileRootJoint)); }
protected:
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse downward
  inline void traverse(ModelGroup& modelGroup)
  { modelGroup.traverse(*this); }
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse upward
  inline void ascend(Model& model)
  { model.ascend(*this); }
};

} // namespace OpenFDM

#endif
