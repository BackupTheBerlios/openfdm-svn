/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Visitor_H
#define OpenFDM_Visitor_H

#include "Frame.h"

namespace OpenFDM {

class RigidBody;

class Visitor {
public:
  virtual ~Visitor(void)
  {}
  virtual void apply(Frame& frame)
  { traverse(frame); }
  virtual void apply(RigidBody& rigidBody)
  { apply((Frame&)rigidBody); }
  inline void traverse(Frame& frame)
  { frame.traverse(*this); }
};

} // namespace OpenFDM

#endif
