/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeInstance_H
#define OpenFDM_NodeInstance_H

#include "AbstractNodeInstance.h"
#include "NodeContext.h"

namespace OpenFDM {

class NodeInstance : public AbstractNodeInstance {
public:
  NodeInstance(const NodePath& nodePath, const SampleTime& sampleTime,
               const Node* node);
  virtual ~NodeInstance();

protected:
  /// The node context that belongs to this instance.
  virtual AbstractNodeContext& getNodeContext();
  virtual const AbstractNodeContext& getNodeContext() const;

private:
  SharedPtr<AbstractNodeContext> mNodeContext;
};

} // namespace OpenFDM

#endif
