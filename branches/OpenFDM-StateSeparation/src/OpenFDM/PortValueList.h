/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortValueList_H
#define OpenFDM_PortValueList_H

#include "PortValue.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "MechanicLink.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"

namespace OpenFDM {

// A set of port values that a Model gets in its methods.
// Access control to the way the values are accessed is done through the
// Port type system and this class. So do not add additional accessors to the
// port values unless you know what you are doing ...
// Many methods from this class are in the fast path of the simulation, so
// this methods code paths might be more typesafe implemented with virtuals,
// but this implementation without and partly with static casts is used to
// avoid calling many virtual fuctions in the fast path.
class PortValueList {
public:

  // Accessors for real valued ports
  const real_type& operator[](const RealInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }
  real_type& operator[](const RealOutputPort& port)
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }

  // Accessors for matrix valued ports
  const Matrix& operator[](const MatrixInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue(); }
  // FIXME, make sure that the size cannot change.
  // May be we have some kind of base class of Matrix that has no resize call
  // and no resizing assignment/copy???
  Matrix& operator[](const MatrixOutputPort& port)
  { return port.getPortValue(mPortValueVector)->getValue(); }

  const NumericPortValue* getPortValue(const RealInputPort& port)
  {
    if (port.empty())
      return 0;
    return port.getPortValue(mPortValueVector);
  }
  NumericPortValue* getPortValue(const RealOutputPort& port)
  {
    if (port.empty())
      return 0;
    return port.getPortValue(mPortValueVector);
  }
  const NumericPortValue* getPortValue(const MatrixInputPort& port)
  {
    if (port.empty())
      return 0;
    return port.getPortValue(mPortValueVector);
  }
  NumericPortValue* getPortValue(const MatrixOutputPort& port)
  {
    if (port.empty())
      return 0;
    return port.getPortValue(mPortValueVector);
  }

  // FIXME, avoid this method here. With this method the output stage of a model
  // can change the port values, this should not be available in a model.
  // may be this must be a derived class that provides some more access??
  void setPortValue(unsigned idx, PortValue* portValue)
  {
    if (mPortValueVector.size() <= idx)
      mPortValueVector.resize(idx+1);
    mPortValueVector[idx] = portValue;
  }
  bool setOrCheckPortSize(const NumericPort* portInfo, const Size& sz)
  {
    if (!portInfo)
      return false;
    Size oldSize = size(getPortValue(portInfo)->getValue());
    // If the size is still 0x0, just set to the desired size
    if (oldSize(0) == 0 || oldSize(1) == 0) {
      getPortValue(portInfo)->getValue().resize(sz(0), sz(1));
      return true;
    } else if (oldSize == sz)
      return true;
    else
      return false;
  }
  bool setOrCheckPortSize(const MatrixOutputPort& port, const Size& sz)
  {
    Size oldSize = size(port.getPortValue(mPortValueVector)->getValue());
    // If the size is still 0x0, just set to the desired size
    if (oldSize(0) == 0 || oldSize(1) == 0) {
      port.getPortValue(mPortValueVector)->getValue().resize(sz(0), sz(1));
      return true;
    } else if (oldSize == sz)
      return true;
    else
      return false;
  }
  bool setOrCheckPortSize(const MatrixInputPort& port, const Size& sz)
  {
    Size oldSize = size(port.getPortValue(mPortValueVector)->getValue());
    // If the size is still 0x0, just set to the desired size
    if (oldSize(0) == 0 || oldSize(1) == 0) {
      port.getPortValue(mPortValueVector)->getValue().resize(sz(0), sz(1));
      return true;
    } else if (oldSize == sz)
      return true;
    else
      return false;
  }
  const PortValue* getPortValue(unsigned idx) const
  {
    if (mPortValueVector.size() <= idx)
      return 0;
    return mPortValueVector[idx];
  }
  PortValue* getPortValue(unsigned idx)
  {
    if (mPortValueVector.size() <= idx)
      return 0;
    return mPortValueVector[idx];
  }

  /// Save but partially expensive Accessor for numeric ports
  NumericPortValue* getPortValue(const NumericPort* portInfo)
  {
    if (!portInfo)
      return 0;
    return getPortValue(*portInfo);
  }
  NumericPortValue* getPortValue(const NumericPort& portInfo)
  {
    PortValue* portValue = getPortValue(portInfo.getIndex());
    if (!portValue)
      return 0;
    return portValue->toNumericPortValue();
  }
  const NumericPortValue* getPortValue(const NumericPort* portInfo) const
  {
    if (!portInfo)
      return 0;
    return getPortValue(*portInfo);
  }
  const NumericPortValue* getPortValue(const NumericPort& portInfo) const
  {
    const PortValue* portValue = getPortValue(portInfo.getIndex());
    if (!portValue)
      return 0;
    return portValue->toNumericPortValue();
  }
  /// Save but partially expensive Accessor for numeric ports
  const MechanicLinkValue* getPortValue(const MechanicLink* portInfo) const
  {
    if (!portInfo)
      return 0;
    return getPortValue(*portInfo);
  }
  const MechanicLinkValue* getPortValue(const MechanicLink& portInfo) const
  {
    const PortValue* portValue = getPortValue(portInfo.getIndex());
    if (!portValue)
      return 0;
    return portValue->toMechanicLinkValue();
  }
  MechanicLinkValue* getPortValue(const MechanicLink* portInfo)
  {
    if (!portInfo)
      return 0;
    return getPortValue(*portInfo);
  }
  MechanicLinkValue* getPortValue(const MechanicLink& portInfo)
  {
    PortValue* portValue = getPortValue(portInfo.getIndex());
    if (!portValue)
      return 0;
    return portValue->toMechanicLinkValue();
  }
  /// Save but partially expensive Accessor for numeric ports
  const PortValue* getPortValue(const Port* portInfo) const
  {
    if (!portInfo)
      return 0;
    return getPortValue(portInfo->getIndex());
  }
  const PortValue* getPortValue(const Port& portInfo) const
  {
    return getPortValue(portInfo.getIndex());
  }

protected:
  PortValueVector mPortValueVector;
};

} // namespace OpenFDM

#endif
