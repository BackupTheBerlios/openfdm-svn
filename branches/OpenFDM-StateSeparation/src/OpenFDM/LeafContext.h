/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LeafContext_H
#define OpenFDM_LeafContext_H

#include "ContinousStateValueVector.h"
#include "DiscreteStateValueVector.h"
#include "PortValueList.h"

namespace OpenFDM {

class LeafContext {
public:
  // PortValues
  PortValueList mPortValueList;
  // Hmm, port connect information?? How to store??
  
  // Continous States
  ContinousStateValueVector mContinousState;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;

  // Task id's ???

  // Output Watch, used to monitor outputs during simulation

  // May be complete path in the system?

  // functors to the operations to do??
};

} // namespace OpenFDM

#endif
