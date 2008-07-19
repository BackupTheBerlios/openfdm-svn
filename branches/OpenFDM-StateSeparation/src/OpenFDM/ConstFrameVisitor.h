/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstFrameVisitor_H
#define OpenFDM_ConstFrameVisitor_H

#include "Frame.h"

namespace OpenFDM {

class ConstFrameVisitor {
public:
  virtual ~ConstFrameVisitor(void)
  {}
  virtual void apply(const Frame& frame)
  { traverse(frame); }
  inline void traverse(const Frame& frame)
  { frame.traverse(*this); }
};

} // namespace OpenFDM

#endif
