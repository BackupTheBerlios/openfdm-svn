/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeContext_H
#define OpenFDM_NodeContext_H

#include "Node.h"
#include "PortValueList.h"

namespace OpenFDM {

/// Hmm, NodeContext is not base of LeafContext for now. FIXME!!!
/// This class might not be user visible. Is implemented somewhere
/// in the simulation backend.
class NodeContext : public Referenced {
public:
  virtual ~NodeContext() { }
  virtual const Node& getNode() const = 0;

  PortValueList& getPortValueList()
  { return mPortValueList; }
  const PortValueList& getPortValueList() const
  { return mPortValueList; }

  // PortValues
  PortValueList mPortValueList;
};

} // namespace OpenFDM

#endif
