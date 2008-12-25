/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RigidBody_H
#define OpenFDM_RigidBody_H

#include <string>
#include "MechanicNode.h"

namespace OpenFDM {

class RigidBody : public MechanicNode {
  OPENFDM_OBJECT(RigidBody, MechanicNode);
public:
  RigidBody(const std::string& name);
  virtual ~RigidBody();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  const PortInfo* addLink(const std::string& name);
  void removeLink(const PortInfo* portInfo);

private:
  typedef std::vector<MechanicLink> MechanicLinkVector;
  MechanicLinkVector mMechanicLinks;
};

} // namespace OpenFDM

#endif
