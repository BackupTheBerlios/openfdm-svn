/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MultiBodyModel_H
#define OpenFDM_MultiBodyModel_H

#include "Object.h"
#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

class Frame;
class Joint;
class Visitor;
class ConstVisitor;

// Should read StateNode or something like that.
class MultiBodyModel :
    public Model {
public:
  MultiBodyModel(const std::string& name);
  virtual ~MultiBodyModel(void);

  virtual void accept(Visitor& visitor);
  virtual void traverse(Visitor& visitor);
  virtual void accept(ConstVisitor& visitor) const;
  virtual void traverse(ConstVisitor& visitor) const;

  virtual Joint* toJoint(void);
  virtual const Joint* toJoint(void) const;

  // Set the parent node.
  virtual bool setParentFrame(Frame* parent, unsigned idx) = 0;
  virtual bool removeParentFrame(Frame* parent) = 0;
  // Return the parent node.
  virtual Frame* getParentFrame(unsigned idx) = 0;
  virtual const Frame* getParentFrame(unsigned idx) const = 0;
  virtual bool isParentFrame(const Frame* group) const = 0;

protected:

  template<unsigned numParents>
  class NodeImplementation {
  public:
    bool setParentFrame(Frame* parent, unsigned idx)
    {
      if (numParents <= idx)
        return false;
      _parent[idx] = parent;
      return true;
    }
    bool removeParentFrame(Frame* parent)
    {
      unsigned idx;
      for (idx = 0; idx < numParents; ++idx) {
        if (parent == _parent[idx]) {
          _parent[idx] = 0;
          return true;
        }
      }
      return false;
    }
    // Return the parent node.
    Frame* getParentFrame(unsigned idx)
    { if (numParents <= idx) return 0; return _parent[idx]; }
    const Frame* getParentFrame(unsigned idx) const
    { if (numParents <= idx) return 0; return _parent[idx]; }
    bool isParentFrame(const Frame* group) const
    {
      for (unsigned idx = 0; idx < numParents; ++idx) {
        if (group == _parent[idx])
          return true;
      }
      return false;
    }
    
  private:
    // The parent node.
    // FIXME: May be we should store a list of all parents ???
    WeakPtr<Frame> _parent[numParents];
  };

#define OpenFDM_NodeImplementation(nParents)            \
public:                                                 \
  virtual bool setParentFrame(Frame* parent, unsigned idx)   \
  { return mNodeImpl.setParentFrame(parent, idx); }          \
  virtual bool removeParentFrame(Frame* parent)              \
  { return mNodeImpl.removeParentFrame(parent); }            \
  virtual Frame* getParentFrame(unsigned idx)                \
  { return mNodeImpl.getParentFrame(idx); }                  \
  virtual const Frame* getParentFrame(unsigned idx) const    \
  { return mNodeImpl.getParentFrame(idx); }                   \
  virtual bool isParentFrame(const Frame* group) const       \
  { return mNodeImpl.isParentFrame(group); }                 \
private:                                                \
  NodeImplementation<nParents> mNodeImpl

};

} // namespace OpenFDM

#endif
