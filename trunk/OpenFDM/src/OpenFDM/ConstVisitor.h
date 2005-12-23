/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstVisitor_H
#define OpenFDM_ConstVisitor_H

#include "Frame.h"

namespace OpenFDM {

class Frame;
class RigidBody;
class Force;
class Joint;
class Interact;

class ConstVisitor {
public:
  virtual ~ConstVisitor(void)
  {}
  virtual void apply(const Frame& frame)
  { traverse(frame); }
  virtual void apply(const RigidBody& body)
  { apply((const Frame&)body); }
  virtual void apply(const MultiBodyModel& abNode)
  { traverse(abNode); }
  virtual void apply(const Joint& joint)
  { apply((const MultiBodyModel&)joint); }
  virtual void apply(const Interact& interact)
  { }
  inline void traverse(const Frame& frame)
  { frame.traverse(*this); }
  inline void traverse(const MultiBodyModel& multiBodyModel)
  { multiBodyModel.traverse(*this); }
};

} // namespace OpenFDM

#endif
