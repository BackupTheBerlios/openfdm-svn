/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeContext_H
#define OpenFDM_NodeContext_H

#include "SharedPtr.h"
#include "AbstractNodeContext.h"

namespace OpenFDM {

/// This one will not show up in any execution list, but will be used
/// to fill NodeContext's for Node's that have nothing to execute,
/// should be reflected to the user of the simulation system. Group's
/// inputs ad outputs and their input and output models are such examples.
class NodeContext : public AbstractNodeContext {
public:
  NodeContext(const Node* node);
  virtual ~NodeContext();

  virtual const Node& getNode() const;

private:
  NodeContext();
  NodeContext(const NodeContext&);
  NodeContext& operator=(const NodeContext&);

  SharedPtr<const Node> mNode;
};

} // namespace OpenFDM

#endif
