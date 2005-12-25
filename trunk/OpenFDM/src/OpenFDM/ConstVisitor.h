/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstVisitor_H
#define OpenFDM_ConstVisitor_H

#include "Frame.h"

namespace OpenFDM {

class RigidBody;

class ConstVisitor {
public:
  virtual ~ConstVisitor(void)
  {}
  virtual void apply(const Frame& frame)
  { traverse(frame); }
  virtual void apply(const RigidBody& rigidBody)
  { apply((const Frame&)rigidBody); }
  inline void traverse(const Frame& frame)
  { frame.traverse(*this); }
};

} // namespace OpenFDM

#endif
