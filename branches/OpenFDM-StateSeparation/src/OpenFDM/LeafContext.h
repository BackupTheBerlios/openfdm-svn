/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LeafContext_H
#define OpenFDM_LeafContext_H

#include "ContinousStateValueVector.h"
#include "DiscreteStateValueVector.h"
#include "NodeContext.h"

namespace OpenFDM {

class LeafContext : public NodeContext {
public:
  // Continous States
  ContinousStateValueVector mContinousState;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;

  // Task id's ???

  // functors to the operations to do??

};

} // namespace OpenFDM

#endif
