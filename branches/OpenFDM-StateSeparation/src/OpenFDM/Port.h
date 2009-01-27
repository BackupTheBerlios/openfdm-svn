/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Port_H
#define OpenFDM_Port_H

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
// Port - The meta data used to see what kind of port we have
// PortValue - The ports value during simulation

class ConstNodeVisitor;
class Node;
class NodeVisitor;

class NumericPort;
class InputPort;
class OutputPort;
class MechanicLink;

class Port : public WeakReferenced {
public:
  Port(Node* node, const std::string& name);
  virtual ~Port();

  const std::string& getName() const { return mName; }
  void setName(const std::string& name);

  // Note that both are const methods.
  // That is only the owner nodes can modify them.
  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  SharedPtr<const Node> getNode() const
  { return mNode.lock(); }

  unsigned getIndex() const { return mIndex; }

  virtual const NumericPort* toNumericPort() const { return 0; }
  virtual const InputPort* toInputPort() const { return 0; }
  virtual const OutputPort* toOutputPort() const { return 0; }
  virtual const MechanicLink* toMechanicLink() const { return 0; }

  // May be virtual here ???, identify the fast and the slow path ...
  PortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    unsigned index = getIndex();
    OpenFDMAssert(index < portValueVector.size());
    return portValueVector[index];
  }

  void clear();

  virtual unsigned getMaxConnects() const = 0;
  virtual bool canConnect(const Port& portInfo) const = 0;
  virtual bool acceptPortValue(PortValue*) const = 0;

  /// Public interface to instantiate a new port value
  PortValue* newValue() const
  { return newValueImplementation(); }

protected:
  virtual PortValue* newValueImplementation() const = 0;

private:
  Port(const Port&);
  Port& operator=(const Port&);

  void setIndex(unsigned index) { mIndex = index; }
  void invalidateIndex() { setIndex(~0u); }

  WeakPtr<Node> mNode;
  std::string mName;
  unsigned mIndex;

  // FIXME: Hmm, can I avoid this??
  friend class Node;
};

class NumericPort : public Port {
public:
  NumericPort(Node* node, const std::string& name, const Size& size) :
    Port(node, name),
    mSize(size)
  { }
  virtual ~NumericPort()
  { }

  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual const NumericPort* toNumericPort() const
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

class InputPort : public NumericPort {
public:
  InputPort(Node* node, const std::string& name, const Size& size,
                bool directInput) :
    NumericPort(node, name, size),
    mDirectInput(directInput)
  { }
  virtual ~InputPort()
  { }

  virtual const InputPort* toInputPort() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const Port& portInfo) const
  { return portInfo.toOutputPort(); }

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

class OutputPort : public NumericPort {
public:
  OutputPort(Node* node, const std::string& name, const Size& size,
                 bool accelerationOutput) :
    NumericPort(node, name, size),
    mAccelerationOutput(accelerationOutput)
  { }
  virtual ~OutputPort()
  { }

  virtual const OutputPort* toOutputPort() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return Limits<unsigned>::max(); }

  virtual bool canConnect(const Port& portInfo) const
  { return portInfo.toInputPort(); }

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

class MechanicLink : public Port {
public:
  // FIXME: mechanic links are special. Just allow them in MechanicNodes ...
  MechanicLink(/*Mechanic*/Node* node, const std::string& name) :
    Port(node, name)
  { }
  virtual ~MechanicLink()
  { }

  virtual void accept(NodeVisitor& visitor) const;
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual const MechanicLink* toMechanicLink() const
  { return this; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool canConnect(const Port& portInfo) const
  { return portInfo.toMechanicLink(); }

  virtual bool acceptPortValue(PortValue* portValue) const
  { return portValue->toMechanicLinkValue(); }

protected:
  virtual MechanicLinkValue* newValueImplementation() const
  { return new MechanicLinkValue; }
};

} // namespace OpenFDM

#endif
