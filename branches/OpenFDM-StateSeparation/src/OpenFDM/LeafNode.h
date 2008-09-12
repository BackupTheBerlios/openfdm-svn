/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LeafNode_H
#define OpenFDM_LeafNode_H

#include <string>
#include "Node.h"
#include "ContinousStateInfoVector.h"
#include "StateInfoVector.h"

namespace OpenFDM {

class LeafContext;
class NodeVisitor;
class ContinousStateValueVector;
class DiscreteStateValueVector;

class LeafNode : public Node {
  OPENFDM_OBJECT(LeafNode, Node);
public:
  LeafNode(const std::string& name);

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  // Is done once before a model starts to live
  // Should have connect information here, can setup memory allocations and
  // sizes. Then the PortValueList does not need to have resizable stuff.
  // The same goes for the states.
  virtual bool alloc(LeafContext&) const // = 0;
  { return true; }

  virtual void init(DiscreteStateValueVector&, ContinousStateValueVector&) const // = 0;
  { }

  // for dependency analysis
  virtual bool dependsOn(const PortId& in, const PortId& out) const = 0;

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
