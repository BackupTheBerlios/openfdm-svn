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

  /// Simple node distributing the positions, velocities and accelerations
  /// from the parent link to the child links. Forces and inertias are summed
  /// over the children and written into the parent link.
  ///
  /// Idea: may be use the same link value for all links and use a
  /// contributeForce/contributeInertia method to add inertia to a
  /// parent link???

  virtual void velocity(const Task&, const ContinousStateValueVector& states,
                        PortValueList& portValues, MechanicContext&) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, MechanicContext&) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, MechanicContext&) const;

private:
  std::vector<MechanicLink> mMechanicLinks;
};

} // namespace OpenFDM

#endif
