/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
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

  const MechanicLinkInfo* addLink(const std::string& name);
  void removeLink(const MechanicLinkInfo* mechanicLinkInfo);

private:
  typedef std::vector<SharedPtr<MechanicLinkInfo> > MechanicLinkInfoVector;
  MechanicLinkInfoVector mMechanicLinks;
};

} // namespace OpenFDM

#endif
