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

typedef std::list<SharedPtr<Node> > NodePath;

class ModelVisitor {
public:
  virtual ~ModelVisitor(void)
  { }
  virtual void apply(Node& node)
  { }
  virtual void apply(Model& model)
  { apply(static_cast<Node&>(model)); }
  virtual void apply(ModelGroup& modelGroup)
  { apply(static_cast<Node&>(modelGroup)); }
  virtual void apply(System& system)
  { apply(static_cast<ModelGroup&>(system)); }
  virtual void apply(Interact& interact)
  { apply(static_cast<Model&>(interact)); }
  virtual void apply(Joint& joint)
  { apply(static_cast<Interact&>(joint)); }
  virtual void apply(MobileRootJoint& mobileRootJoint)
  { apply(static_cast<Joint&>(mobileRootJoint)); }

  const NodePath& getNodePath() const
  { return mNodePath; }

protected:
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse downward
  inline void traverse(ModelGroup& modelGroup)
  {
    mNodePath.push_back(&modelGroup);
    modelGroup.traverse(*this);
    mNodePath.pop_back();
  }
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse upward
  inline void ascend(Node& node)
  {
    mNodePath.push_back(&node);
    node.ascend(*this);
    mNodePath.pop_back();
  }
private:
  // The path that visitor has passed
  NodePath mNodePath;
};

} // namespace OpenFDM

#endif
