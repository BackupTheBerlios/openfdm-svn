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

class ConstNodeVisitor;
class Node;
class NodeVisitor;

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

  // Note that both are const methods.
  // That is only the owner nodes can modify them.
  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  SharedPtr<const Node> getNode() const
  { return mNode.lock(); }

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
  virtual bool acceptPortValue(PortValue*) const = 0;

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

  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual const NumericPortInfo* toNumericPortInfo() const
  { return this; }

  virtual bool acceptPortValue(PortValue* portValue) const
  {
    NumericPortValue* numericPortValue;
    numericPortValue = portValue->toNumericPortValue();
    if (!numericPortValue)
      return false;
    if (mSize == Size(0, 0))
      return true;
    if (size(numericPortValue->getValue()) == Size(0, 0)) {
      numericPortValue->getValue().resize(mSize(0), mSize(1));
      return true;
    }
    return size(numericPortValue->getValue()) == mSize;
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
  // FIXME Derive??
  bool mDirectInput;
};

class OutputPortInfo : public NumericPortInfo {
public:
  OutputPortInfo(Node* node, const std::string& name, const Size& size,
                 bool accelerationOutput) :
    NumericPortInfo(node, name, size),
    mAccelerationOutput(accelerationOutput)
  { }
  virtual ~OutputPortInfo()
  { }

  virtual const OutputPortInfo* toOutputPortInfo() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return Limits<unsigned>::max(); }

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toInputPortInfo(); }

  bool getAccelerationOutput() const
  { return mAccelerationOutput; }
  void setAccelerationOutput(bool accelerationOutput)
  { mAccelerationOutput = accelerationOutput; }

protected:
  virtual NumericPortValue* newValueImplementation() const
  { return new NumericPortValue(mSize); }

private:
  // FIXME Derive??
  bool mAccelerationOutput;
};

class MechanicLinkInfo : public PortInfo {
public:
  // FIXME: mechanic links are special. Just allow them in MechanicNodes ...
  MechanicLinkInfo(/*Mechanic*/Node* node, const std::string& name) :
    PortInfo(node, name)
  { }
  virtual ~MechanicLinkInfo()
  { }

  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual const MechanicLinkInfo* toMechanicLinkInfo() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toMechanicLinkInfo(); }

  virtual bool acceptPortValue(PortValue* portValue) const
  { return portValue->toMechanicLinkValue(); }

protected:
  virtual MechanicLinkValue* newValueImplementation() const
  { return new MechanicLinkValue; }
};

} // namespace OpenFDM

#endif
