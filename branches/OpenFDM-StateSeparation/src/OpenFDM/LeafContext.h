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
    mContinousState[stateInfo].resize(sz(0), sz(1));
    mContinousStateDerivative[stateInfo].resize(sz(0), sz(1));
  }

  bool allocStates()
  {
    unsigned numContinousStates = getNode().getNumContinousStateValues();
    for (unsigned i = 0; i < numContinousStates; ++i) {
      const ContinousStateInfo* continousStateInfo;
      continousStateInfo = getNode().getContinousStateInfo(i);
      mContinousState.setValue(*continousStateInfo, *this);
      mContinousStateDerivative.setValue(*continousStateInfo, *this);
    }
    unsigned numDiscreteStates = getNode().getNumDiscreteStateValues();
    for (unsigned i = 0; i < numDiscreteStates; ++i) {
      const StateInfo* stateInfo;
      stateInfo = getNode().getDiscreteStateInfo(i);
      mDiscreteState.setValue(*stateInfo, *this);
    }
    return true;
  }

  /// might vanish???
  PortValueList& getPortValueList()
  { return mPortValueList; }
  const PortValueList& getPortValueList() const
  { return mPortValueList; }

// protected:
  // Continous States
  ContinousStateValueVector mContinousState;
  ContinousStateValueVector mContinousStateDerivative;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;
};

} // namespace OpenFDM

#endif
