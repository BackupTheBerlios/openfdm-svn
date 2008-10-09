/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortInfo_H
#define OpenFDM_PortInfo_H

#include <string>
#include "Assert.h"
#include "PortValue.h"
#include "WeakPtr.h"
#include "WeakReferenced.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"

namespace OpenFDM {

// Some notes on ports:
// Naming:
// PortInfo - The meta data used to see what kind of port we have
// PortValue - The ports value during simulation
// PortHandle?? Hmmm??
// Port : NodeId, PortId ???

class Node;

class NumericPortInfo;
class InputPortInfo;
class OutputPortInfo;
class MechanicLinkInfo;

class PortInfo : public WeakReferenced {
public:
  PortInfo(Node* node, const std::string& name);
  virtual ~PortInfo();

  const std::string& getName() const { return mName; }
  void setName(const std::string& name);

  unsigned getIndex() const { return mIndex; }

  virtual const NumericPortInfo* toNumericPortInfo() const { return 0; }
  virtual const InputPortInfo* toInputPortInfo() const { return 0; }
  virtual const OutputPortInfo* toOutputPortInfo() const { return 0; }
  virtual const MechanicLinkInfo* toMechanicLinkInfo() const { return 0; }

  // May be virtual here ???, identify the fast and the slow path ...
  PortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    unsigned index = getIndex();
    OpenFDMAssert(index < portValueVector.size());
    return portValueVector[index];
  }

  void clear();

  virtual unsigned getMaxConnects() const = 0;
  virtual bool canConnect(const PortInfo& portInfo) const = 0;
  virtual bool acceptPortValue(const PortValue*) const = 0;

  /// Public interface to instantiate a new port value
  PortValue* newValue() const
  { return newValueImplementation(); }

protected:
  virtual PortValue* newValueImplementation() const = 0;

private:
  PortInfo(const PortInfo&);
  PortInfo& operator=(const PortInfo&);

  void setIndex(unsigned index) { mIndex = index; }
  void invalidateIndex() { setIndex(~0u); }

  WeakPtr<Node> mNode;
  std::string mName;
  unsigned mIndex;

  // FIXME: Hmm, can I avoid this??
  friend class Node;
};

class NumericPortInfo : public PortInfo {
public:
  NumericPortInfo(Node* node, const std::string& name, const Size& size) :
    PortInfo(node, name),
    mSize(size)
  { }
  virtual ~NumericPortInfo()
  { }

  virtual const NumericPortInfo* toNumericPortInfo() const
  { return this; }

  virtual bool acceptPortValue(const PortValue* portValue) const
  {
    const NumericPortValue* numericPortValue;
    numericPortValue = portValue->toNumericPortValue();
    if (!numericPortValue)
      return false;
    // May be do a size check here???
    return true;
  }

protected:
  Size mSize;
};

class InputPortInfo : public NumericPortInfo {
public:
  InputPortInfo(Node* node, const std::string& name, const Size& size,
                bool directInput) :
    NumericPortInfo(node, name, size),
    mDirectInput(directInput)
  { }
  virtual ~InputPortInfo()
  { }

  virtual const InputPortInfo* toInputPortInfo() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toOutputPortInfo(); }

  bool getDirectInput() const
  { return mDirectInput; }
  void setDirectInput(bool directInput)
  { mDirectInput = directInput; }

protected:
  virtual NumericPortValue* newValueImplementation() const
  { return 0; }

private:
  bool mDirectInput;
};

class OutputPortInfo : public NumericPortInfo {
public:
  OutputPortInfo(Node* node, const std::string& name, const Size& size) :
    NumericPortInfo(node, name, size)
  { }
  virtual ~OutputPortInfo()
  { }

  virtual const OutputPortInfo* toOutputPortInfo() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return Limits<unsigned>::max(); }

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toInputPortInfo(); }

protected:
  virtual NumericPortValue* newValueImplementation() const
  { return new NumericPortValue(mSize); }
};

class MechanicLinkInfo : public PortInfo {
public:
  MechanicLinkInfo(Node* node, const std::string& name) :
    PortInfo(node, name)
  { }
  virtual ~MechanicLinkInfo()
  { }

  virtual const MechanicLinkInfo* toMechanicLinkInfo() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toMechanicLinkInfo(); }

  virtual bool acceptPortValue(const PortValue* portValue) const
  { return portValue->toMechanicLinkValue(); }

protected:
  virtual MechanicLinkValue* newValueImplementation() const
  { return new MechanicLinkValue; }
};

} // namespace OpenFDM

#endif
