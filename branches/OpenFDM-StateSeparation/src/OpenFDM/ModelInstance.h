/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelInstance_H
#define OpenFDM_ModelInstance_H

#include <list>
#include "AbstractNodeInstance.h"
#include "ModelContext.h"
#include "SharedPtr.h"

namespace OpenFDM {

class ModelInstance : public AbstractNodeInstance {
public:
  ModelInstance(const NodePath& nodePath, const Model* model);
  virtual ~ModelInstance();

  // Return true if this leaf directly depends on one of leafInstance outputs
  bool dependsOn(const ModelInstance& modelInstance) const;

  // FIXME
// protected:
  /// The node context that belongs to this instance.
  virtual ModelContext& getNodeContext();
  virtual const ModelContext& getNodeContext() const;

private:
  SharedPtr<ModelContext> mModelContext;
};

typedef std::list<SharedPtr<ModelInstance> > ModelInstanceList;

} // namespace OpenFDM

#endif