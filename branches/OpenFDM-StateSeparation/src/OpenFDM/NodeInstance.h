/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeInstance_H
#define OpenFDM_NodeInstance_H

#include "AbstractNodeInstance.h"

namespace OpenFDM {

class NodeInstance : public AbstractNodeInstance {
public:
  NodeInstance(const SampleTime&, const Node*, const PortValueList&);
  virtual ~NodeInstance();

  virtual const Node& getNode() const;

  virtual const PortValue* getPortValue(const PortInfo&) const;
  virtual const NumericPortValue* getPortValue(const NumericPortInfo&) const;
  virtual const MechanicLinkValue* getPortValue(const MechanicLinkInfo&) const;

private:
  SharedPtr<const Node> mNode;

  PortValueList mPortValueList;
};

} // namespace OpenFDM

#endif
