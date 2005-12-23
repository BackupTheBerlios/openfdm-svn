/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Visitor_H
#define OpenFDM_Visitor_H

#include "Frame.h"

namespace OpenFDM {

class RigidBody;
class Joint;
class MultiBodyModel;
class Interact;

class Visitor {
public:
  virtual ~Visitor(void)
  {}
  virtual void apply(Frame& frame)
  { traverse(frame); }
  virtual void apply(RigidBody& body)
  { apply((Frame&)body); }
  virtual void apply(MultiBodyModel& abNode)
  { traverse(abNode); }
  virtual void apply(Joint& joint)
  { apply((MultiBodyModel&)joint); }
  virtual void apply(Interact& interact)
  { }
  inline void traverse(Frame& frame)
  { frame.traverse(*this); }
  inline void traverse(MultiBodyModel& multiBodyModel)
  { multiBodyModel.traverse(*this); }
};

} // namespace OpenFDM

#endif
