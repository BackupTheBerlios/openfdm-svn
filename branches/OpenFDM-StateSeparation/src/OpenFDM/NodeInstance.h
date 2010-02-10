/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeInstance_H
#define OpenFDM_NodeInstance_H

#include "AbstractNodeInstance.h"
#include "PortValueList.h"

namespace OpenFDM {

class NodeInstance : public AbstractNodeInstance {
public:
  NodeInstance(const SampleTime&, const Node*, const PortValueList&);
  virtual ~NodeInstance();

  virtual const Node& getNode() const;

  virtual const PortValue* getPortValue(const Port&) const;
  virtual const NumericPortValue* getPortValue(const NumericPort&) const;
  virtual const MechanicLinkValue* getPortValue(const MechanicLink&) const;

private:
  SharedPtr<const Node> mNode;

  PortValueList mPortValueList;
};

} // namespace OpenFDM

#endif
