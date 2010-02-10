/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLink_H
#define OpenFDM_MechanicLink_H

#include "MechanicLinkValue.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicLink : public Port {
public:
  MechanicLink(Node* node, const std::string& name);
  virtual ~MechanicLink();

  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual const MechanicLink* toMechanicLink() const;

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const Port& portInfo) const
  { return portInfo.toMechanicLink(); }

  virtual bool acceptPortValue(PortValue* portValue) const
  { return portValue->toMechanicLinkValue(); }

protected:
  virtual MechanicLinkValue* newValueImplementation() const
  { return new MechanicLinkValue; }
};

} // namespace OpenFDM

#endif
