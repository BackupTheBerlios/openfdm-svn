/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelVisitor_H
#define OpenFDM_ModelVisitor_H

#include "Model.h"
#include "ModelGroup.h"
// #include "MultiBodyModel.h"

namespace OpenFDM {

class ModelVisitor {
public:
  virtual ~ModelVisitor(void)
  { }
  virtual void apply(Model& model)
  { }
  virtual void apply(ModelGroup& modelGroup)
  { apply((Model&)modelGroup); }
//   virtual void apply(MultiBodyModel& multiBodyModel)
//   { apply((Model&)multiBodyModel); }
protected:
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse downward
  inline void traverse(ModelGroup& modelGroup)
  { modelGroup.traverse(*this); }
  /// Call this in the apply(MultiBodyModel&) method if you want to
  /// traverse downward
//   inline void traverse(MultiBodyModel& multiBodyModel)
//   { multiBodyModel.traverse(*this); }
  /// Call this in the apply(ModelGroup&) method if you want to
  /// traverse upward
  inline void ascend(Model& model)
  { model.ascend(*this); }
  /// Call this in the apply(MultiBodyModel&) method if you want to
  /// traverse upward
//   inline void ascend(MultiBodyModel& multiBodyModel)
//   { multiBodyModel.ascend(*this); }
};

} // namespace OpenFDM

#endif
