/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CSVSystemOutput_H
#define OpenFDM_CSVSystemOutput_H

#include <sstream>
#include <set>
#include <fstream>
#include "Group.h"
#include "SystemOutput.h"
#include "ConstNodeVisitor.h"
#include "NodeInstance.h"

namespace OpenFDM {

class CSVSystemOutput : public SystemOutput {
public:
  CSVSystemOutput(const std::string& filename) :
    mFileStream((filename + ".csv").c_str())
  { }
  virtual ~CSVSystemOutput()
  { }

  virtual void output(const real_type& t)
  {
    mFileStream << t;
    DumperList::iterator i;
    for (i = mDumperList.begin(); i != mDumperList.end(); ++i) {
      mFileStream << ',';
      (*i)->append(mFileStream);
    }
    mFileStream << '\n';
  }

  virtual void attachTo(const System* system)
  {
    mDumperList.clear();
    if (!system)
      return;
    mFileStream << "time";
    Visitor visitor(system, mFileStream);
    system->getNode()->accept(visitor);
    mDumperList = visitor.mDumperList;
    mFileStream << '\n';
  }

private:
  std::ofstream mFileStream;

  struct Dumper : public Referenced {
    virtual ~Dumper() {}
    virtual void append(std::ostream&) = 0;
  };

  struct MatrixDumper : public Dumper {
    MatrixDumper(const NumericPortValue* numericPortValue, 
                 const std::string& name) :
      mNumericPortValue(numericPortValue)
    { OpenFDMAssert(numericPortValue); }
    virtual void append(std::ostream& stream)
    {
      Size s = size(mNumericPortValue->getValue());
      for (unsigned i = 0; i < s(0); ++i)
        for (unsigned j = 0; j < s(1); ++j)
          stream << ' ' << mNumericPortValue->getValue()(i, j);
    }
    SharedPtr<const NumericPortValue> mNumericPortValue;
  };

  typedef std::list<SharedPtr<Dumper> > DumperList;
  DumperList mDumperList;

  class Visitor : public ConstNodeVisitor {
  public:
    Visitor(const System* system, std::ostream& stream) :
      mSystem(system),
      mStream(stream)
    { }
    
    SharedPtr<const System> mSystem;
    std::ostream& mStream;
    
    const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
    {
      if (!mSystem)
        return 0;
      return mSystem->getNodeInstance(nodePath);
    }
    
    virtual void apply(const NumericPortInfo& portInfo)
    {
      const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
      if (!nodeInstance)
        return;
      apply(portInfo, nodeInstance->getPortValue(portInfo));
    }
    
    virtual void apply(const NumericPortInfo& portInfo,
                       const NumericPortValue* numericPortValue)
    {
      std::string name = portInfo.getName();
      mStream << ',' << Node::toNodePathName(getNodePath()) << '/' << name;
      mDumperList.push_back(new MatrixDumper(numericPortValue, name));
    }
    
    void appendPortValues(const Node& node)
    {
      if (!node.getNumPorts())
        return;
      node.traversePorts(*this);
    }
    
    virtual void apply(const Node& node)
    {
      appendPortValues(node);
    }
    virtual void apply(const Group& group)
    {
      appendPortValues(group);
      group.traverse(*this);
    }
    
    typedef std::list<SharedPtr<Dumper> > DumperList;
    DumperList mDumperList;
  };
};

} // namespace OpenFDM

#endif
