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

  PortId addLink(const std::string& name);
  void removeLink(const PortId& portId);

  /// Simple node distributing the positions, velocities and accelerations
  /// from the parent link to the child links. Forces and inertias are summed
  /// over the children and written into the parent link.
  ///
  /// Idea: may be use the same link value for all links and use a
  /// contributeForce/contributeInertia method to add inertia to a
  /// parent link???

  virtual void velocity(const Task&, const ContinousStateValueVector& states,
                        PortValueList& portValues, FrameData&) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, FrameData&) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, FrameData&) const;
private:
  typedef std::vector<MechanicLink> MechanicLinkVector;
  MechanicLinkVector mMechanicLinks;
};

} // namespace OpenFDM

#endif
