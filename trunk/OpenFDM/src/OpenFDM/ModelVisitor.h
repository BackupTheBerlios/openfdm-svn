/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelVisitor_H
#define OpenFDM_ModelVisitor_H

#include "Model.h"
#include "Output.h"
#include "Input.h"
#include "ModelGroup.h"
#include "RigidBody.h"
#include "MobileRootJoint.h"
#include "System.h"

namespace OpenFDM {

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

  virtual void apply(Output& output)
  { apply(static_cast<Model&>(output)); }
  virtual void apply(Input& input)
  { apply(static_cast<Model&>(input)); }
  virtual void apply(Interact& interact)
  { apply(static_cast<Model&>(interact)); }

  virtual void apply(System& system)
  { apply(static_cast<ModelGroup&>(system)); }

  virtual void apply(Joint& joint)
  { apply(static_cast<Interact&>(joint)); }

  virtual void apply(MobileRootJoint& mobileRootJoint)
  { apply(static_cast<Joint&>(mobileRootJoint)); }


  const Node::Path& getNodePath() const
  { return mNodePath; }

protected:
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse downward
  inline void traverse(ModelGroup& modelGroup)
  {
    for (unsigned i = 0; i < modelGroup.getNumModels(); ++i) {
      SharedPtr<Node> node = modelGroup.getModel(i);
      mNodePath.push_back(node);
      node->accept(*this);
      mNodePath.pop_back();
    }
  }
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse upward
  inline void ascend(Node& node)
  {
    for (unsigned i = 0; i < node.getNumParents(); ++i) {
      SharedPtr<ModelGroup> group = node.getParent(i).lock();
      if (!group)
        continue;
      mNodePath.insert(mNodePath.begin(), group);
      group->accept(*this);
      mNodePath.erase(mNodePath.begin());
    }
  }

private:
  // The path that visitor has passed
  Node::Path mNodePath;
};

} // namespace OpenFDM

#endif
