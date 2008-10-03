/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LeafContext_H
#define OpenFDM_LeafContext_H

#include "AbstractNodeContext.h"
#include "ContinousStateValueVector.h"
#include "DiscreteStateValueVector.h"
#include "LeafNode.h"
#include "MatrixStateInfo.h"

namespace OpenFDM {

class LeafContext : public AbstractNodeContext {
public:
  virtual ~LeafContext() {}
  virtual const LeafNode& getNode() const = 0;

  void setContinousStateSize(const MatrixStateInfo& stateInfo,
                             const Size& sz)
  {
    mContinousState.setValue(stateInfo, *this);
    mContinousState[stateInfo].resize(sz(0), sz(1));
    mContinousStateDerivative.setValue(stateInfo, *this);
    mContinousStateDerivative[stateInfo].resize(sz(0), sz(1));
  }

// protected:
  // Continous States
  ContinousStateValueVector mContinousState;
  ContinousStateValueVector mContinousStateDerivative;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;

  // functors to the operations to do??

};

} // namespace OpenFDM

#endif
