/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LeafNode_H
#define OpenFDM_LeafNode_H

#include <string>
#include "Node.h"
#include "ContinousStateInfoVector.h"
#include "StateInfoVector.h"

namespace OpenFDM {

class ConstNodeVisitor;
class NodeVisitor;

class LeafNode : public Node {
  OPENFDM_OBJECT(LeafNode, Node);
public:
  LeafNode(const std::string& name);

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  unsigned getNumContinousStateValues() const
  { return mContinousStateInfoVector.size(); }
  const ContinousStateInfo* getContinousStateInfo(unsigned index) const
  { return mContinousStateInfoVector.getStateInfo(index); }

  unsigned getNumDiscreteStateValues() const
  { return mDiscreteStateInfoVector.size(); }
  const StateInfo* getDiscreteStateInfo(unsigned index) const
  { return mDiscreteStateInfoVector.getStateInfo(index); }

protected:
  virtual ~LeafNode();

  void addContinousStateInfo(ContinousStateInfo* stateInfo)
  { mContinousStateInfoVector.addStateInfo(stateInfo); }
  void removeContinousStateInfo(ContinousStateInfo* stateInfo)
  { mContinousStateInfoVector.removeStateInfo(stateInfo); }

  void addDiscreteStateInfo(StateInfo* stateInfo)
  { mDiscreteStateInfoVector.addStateInfo(stateInfo); }
  void removeDiscreteStateInfo(StateInfo* stateInfo)
  { mDiscreteStateInfoVector.removeStateInfo(stateInfo); }

private:
  
  // FIXME ???
  ContinousStateInfoVector mContinousStateInfoVector;
  StateInfoVector mDiscreteStateInfoVector;
};

} // namespace OpenFDM

#endif
