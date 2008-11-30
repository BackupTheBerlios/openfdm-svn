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

  virtual ContinousStateValue* getStateValue(const ContinousStateInfo&) = 0;
  virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo&) =0;
};

} // namespace OpenFDM

#endif
