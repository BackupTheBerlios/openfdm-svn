/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FrameVisitor_H
#define OpenFDM_FrameVisitor_H

#include "Frame.h"

namespace OpenFDM {

class FrameVisitor {
public:
  virtual ~FrameVisitor(void)
  {}
  virtual void apply(Frame& frame)
  { traverse(frame); }
  inline void traverse(Frame& frame)
  { frame.traverse(*this); }
};

} // namespace OpenFDM

#endif
